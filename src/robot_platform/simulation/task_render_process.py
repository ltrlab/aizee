"""Single in-flight camera snapshot rendered in a separate interpreter.

The simulation owner sends only MuJoCo state; RGB pixels use a fixed owned shared
memory block. Render completion is tied to that exact state and paired receiver
sample. Neither rendering nor image encoding runs under the physics/control GIL.
"""
from __future__ import annotations
import os
from contextlib import nullcontext
from pathlib import Path
import uuid
from multiprocessing import shared_memory
import queue
import subprocess
import sys
import threading
import time
import numpy as np
from robot_platform.data.capture_process import _read, _write


class _WindowsChildJob:
    """Kernel-owned descendant cleanup even when native GL holds the Python GIL."""
    def __init__(self, process):
        self.handle = None
        if os.name != "nt": return
        import ctypes
        from ctypes import wintypes
        class Limits(ctypes.Structure):
            _fields_ = [("process_time",ctypes.c_longlong),("job_time",ctypes.c_longlong),
                ("flags",wintypes.DWORD),("minimum",ctypes.c_size_t),("maximum",ctypes.c_size_t),
                ("active",wintypes.DWORD),("affinity",ctypes.c_size_t),
                ("priority",wintypes.DWORD),("scheduling",wintypes.DWORD)]
        class Extended(ctypes.Structure):
            _fields_ = [("basic",Limits),("io",ctypes.c_ulonglong*6),
                ("process_memory",ctypes.c_size_t),("job_memory",ctypes.c_size_t),
                ("peak_process",ctypes.c_size_t),("peak_job",ctypes.c_size_t)]
        api=ctypes.WinDLL("kernel32",use_last_error=True)
        api.CreateJobObjectW.argtypes=[ctypes.c_void_p,wintypes.LPCWSTR]
        api.CreateJobObjectW.restype=wintypes.HANDLE
        api.SetInformationJobObject.argtypes=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD]
        api.AssignProcessToJobObject.argtypes=[wintypes.HANDLE,wintypes.HANDLE]
        api.CloseHandle.argtypes=[wintypes.HANDLE]
        self.api=api;self.handle=api.CreateJobObjectW(None,None)
        limits=Extended();limits.basic.flags=0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.handle or not api.SetInformationJobObject(self.handle,9,ctypes.byref(limits),ctypes.sizeof(limits)) or not api.AssignProcessToJobObject(self.handle,int(process._handle)):
            error=ctypes.get_last_error();self.close()
            raise OSError(error,"could not establish renderer parent-death cleanup")
    def close(self):
        if self.handle:
            self.api.CloseHandle(self.handle);self.handle=None


class SceneRenderer:
    def __init__(self, scene, camera_roles, *, directory, viewer=False):
        self.scene, self.camera_roles = scene, tuple(camera_roles)
        self._requests = queue.Queue(maxsize=1)
        self._results = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self._submitted_at = None
        self.shape = (len(camera_roles), scene.height, scene.width, 3)
        self._memory = shared_memory.SharedMemory(create=True, size=int(np.prod(self.shape)))
        self._pixels = np.ndarray(self.shape, np.uint8, buffer=self._memory.buf)
        self._config = dict(scene=str(scene.source), joint_map=scene.capabilities['motor_to_model_joint_map'],
            names=list(scene.names), width=scene.width, height=scene.height,
            visual_mode=scene.visual_mode, camera_roles=list(camera_roles), memory=self._memory.name,
            scene_sha256=scene.capabilities['scene_sha256'], viewer=bool(viewer), parent_pid=os.getpid())
        flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
        self.stderr_path = directory / f"camera-renderer-{uuid.uuid4().hex}.stderr.log"
        try:
            self._process = subprocess.Popen([sys.executable, '-m', __name__, '--worker'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                creationflags=flags, bufsize=0)
            self._child_job = _WindowsChildJob(self._process)
        except Exception:
            if hasattr(self, "_process"):
                self._process.kill(); self._process.wait(timeout=1)
            self._pixels = None
            self._memory.close(); self._memory.unlink()
            raise
        self._stderr_thread = threading.Thread(target=self._stderr, daemon=True, name='camera-render-stderr')
        self._stderr_thread.start()
        self._thread = threading.Thread(target=self._run, daemon=True, name='task-render-transport')
        self._thread.start()

    def _stderr(self):
        written = 0
        try:
            with self.stderr_path.open('wb') as log:
                while True:
                    data = self._process.stderr.read(4096)
                    if not data: return
                    if written < 65536:
                        chunk = data[:65536-written]
                        log.write(chunk); written += len(chunk); log.flush()
        except (OSError, ValueError):
            pass

    def submit(self, packet):
        spec = self.scene.mj.mjtState.mjSTATE_INTEGRATION
        state = np.empty(self.scene.mj.mj_stateSize(self.scene.model, spec))
        self.scene.mj.mj_getState(self.scene.model, self.scene.data, state, spec)
        self._requests.put_nowait((state.tolist(), packet))
        self._submitted_at = time.monotonic()

    def result(self):
        if self.error:
            raise RuntimeError(self.error)
        try:
            packet, completed_at = self._results.get_nowait()
            if self._submitted_at is not None and completed_at-self._submitted_at > .25:
                raise RuntimeError("camera renderer exceeded 250 ms capture deadline")
            self._submitted_at = None
            return packet
        except queue.Empty:
            if self._submitted_at is not None and time.monotonic()-self._submitted_at > .25:
                raise RuntimeError("camera renderer exceeded 250 ms capture deadline")
            return None

    def _run(self):
        try:
            _write(self._process.stdin, self._config)
            response = _read(self._process.stdout)
            if not response.get('ready'):
                raise RuntimeError(response.get('error', 'renderer did not initialize'))
            self.ready.set()
            while not self._stop.is_set():
                try:
                    state, packet = self._requests.get(timeout=.1)
                except queue.Empty:
                    continue
                started = time.monotonic()
                _write(self._process.stdin, {'state': state, 'capture': packet is not None})
                response = _read(self._process.stdout)
                if 'error' in response:
                    raise RuntimeError(response['error'])
                if packet is None:
                    self._results.put_nowait(({'preview': True}, time.monotonic()))
                else:
                    packet.update(images={name:self._pixels[i].copy() for i,name in enumerate(self.camera_roles)},
                                  camera_times=response['camera_times'])
                    packet['task_state']['timing']['render_wall_s'] = time.monotonic()-started
                    self._results.put_nowait((packet, time.monotonic()))
        except Exception as exc:
            self.error = f'camera renderer failed: {type(exc).__name__}: {exc}'
            self.ready.set()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=.5)
        # Only close stdin after its sole writer has returned; a hung render is terminated.
        if not self._thread.is_alive():
            self._process.stdin.close()
        try:
            self._process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill(); self._process.wait(timeout=1)
        self._child_job.close()
        self._thread.join(timeout=.5)
        self._stderr_thread.join(timeout=.5)
        for stream in (self._process.stdin, self._process.stdout, self._process.stderr):
            if not stream.closed: stream.close()
        self._pixels = None
        self._memory.close()
        try:
            self._memory.unlink()
        except FileNotFoundError:
            pass


def _worker():
    from .task_scene import MinervaTaskScene
    scene, memory, window = None, None, None
    packets = queue.Queue(maxsize=1)
    finished = threading.Event()
    def read_parent():
        try:
            while True:
                packets.put(_read(sys.stdin.buffer))
        except (EOFError, OSError, ValueError):
            try: packets.put(None, timeout=.1)
            except queue.Full: pass
            # Ancestor death must also terminate a worker stuck inside native GL.
            if not finished.wait(1.): os._exit(1)
    try:
        config = _read(sys.stdin.buffer)
        parent_pid = config.pop("parent_pid")
        if sys.platform.startswith("linux"):
            import ctypes, signal
            if ctypes.CDLL(None).prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
                raise OSError("could not establish renderer parent-death signal")
            if os.getppid() != parent_pid: return
        threading.Thread(target=read_parent, daemon=True, name="renderer-parent-watchdog").start()
        roles = tuple(config.pop('camera_roles'))
        memory = shared_memory.SharedMemory(name=config.pop('memory'))
        expected = config.pop('scene_sha256')
        visible = config.pop('viewer', False)
        scene = MinervaTaskScene(**config)
        if scene.capabilities['scene_sha256'] != expected:
            raise ValueError('renderer and physics scene models differ')
        pixels = np.ndarray((len(roles),scene.height,scene.width,3),np.uint8,buffer=memory.buf)
        scene.render(roles)  # Warm actual GL assets before readiness.
        if visible:
            from mujoco import viewer
            window = viewer.launch_passive(scene.model, scene.data, show_left_ui=False, show_right_ui=False)
            window.opt.geomgroup[:] = scene.visual_groups
            for flag in (scene.mj.mjtRndFlag.mjRND_SHADOW, scene.mj.mjtRndFlag.mjRND_REFLECTION, scene.mj.mjtRndFlag.mjRND_FOG):
                window.user_scn.flags[flag] = False
            window.sync()
        _write(sys.stdout.buffer, {'ready': True})
        while True:
            packet = packets.get()
            if packet is None: break
            state = np.asarray(packet['state'], dtype=float)
            spec = scene.mj.mjtState.mjSTATE_INTEGRATION
            if state.shape != (scene.mj.mj_stateSize(scene.model,spec),) or not np.all(np.isfinite(state)):
                raise ValueError('malformed render state')
            if window is not None and not window.is_running():
                raise RuntimeError('task viewer closed')
            # The worker owns both GL contexts; no viewer lock ever reaches physics.
            with window.lock() if window is not None else nullcontext():
                scene.mj.mj_setState(scene.model,scene.data,state,spec)
                scene.mj.mj_forward(scene.model,scene.data)
                stamps = None
                if packet['capture']:
                    images, stamps = scene.render(roles)
                    for i,name in enumerate(roles): pixels[i] = images[name]
            if window is not None:
                window.sync()
            _write(sys.stdout.buffer, {'camera_times': stamps})
    except EOFError:
        pass
    except Exception as exc:
        try:
            _write(sys.stdout.buffer, {'error': f'{type(exc).__name__}: {exc}'})
        except OSError:
            pass
    finally:
        finished.set()
        if window is not None: window.close()
        if scene is not None: scene.close()
        if memory is not None: memory.close()


if __name__ == '__main__':
    _worker()
