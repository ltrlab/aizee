"""Actual Rust receiver -> process scene -> process cameras -> atomic raw episode.

No USB or motors. Scripted dual leaders and an explicitly stepped simulation
clock make this a finite correctness contract, not a real-time qualification.
The separate local wall-clock probe measures deadlines without relaxing them.
Linux CI supplies MUJOCO_GL=osmesa; this required contract does not skip rendering.
"""
import importlib.util
import json
import math
from pathlib import Path
import time

import h5py
import numpy as np
import pytest
from robot_platform.control.joint_calibration import load_plan
from robot_platform.control.servo_calibration import load_servo
from robot_platform.helm.recording import RecordingError
from robot_platform.helm.teleop import TeleopCore
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.minerva_sim import SCENE
from robot_platform.simulation.task_scene import TaskSceneView

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
pytestmark = pytest.mark.integration
_spec = importlib.util.spec_from_file_location('task_sim_fixtures', Path(__file__).with_name('test_helm_sim.py'))
fixtures = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fixtures)


def test_receiver_rows_and_two_real_rendered_cameras_form_one_episode_and_worker_death_fails_closed(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    plan, _, device, reference, _, _, bindings = load_plan(DEVICES/'minerva-joint-calibration.json')
    # Explicit signed test calibration, matching the current documented directions. The
    # local operator-evidence probe separately validates the saved session's bindings.
    directions = {step['joint']: dict(motor=step['motor'], offset_rad=0., direction_verified=True,
        sign=-1 if step['motor'] in {'right_arm_j1','right_arm_j3','right_arm_j4'} else 1) for step in plan['steps']}
    profile = json.loads((DEVICES/'minerva-teleop-direct.json').read_text())
    routes = {name:dict(sign=entry['sign'],verified=True) for name,entry in profile['routes'].items()}
    clock = fixtures.Clock()
    leaders = fixtures.Leaders(clock)
    bundle = dict(calibration_profile=str(DEVICES/'minerva-joint-calibration.json'), bindings=bindings,
        directions=directions, limits=json.loads((DEVICES/'minerva-production-01-joint-limits.json').read_text()),
        teleop_profile=str(DEVICES/'minerva-teleop-direct.json'), routes_path=None, route_edits=routes,
        asset_id=device['asset_id'], leader_factory=None, receiver={'mode':'virtual'},
        leader_ranges=fixtures.leader_ranges(), servo=load_servo(DEVICES/'minerva-production-01-servo.json'),
        simulation={'viewer':False,'directory':str(tmp_path/'receiver')})
    core = TeleopCore(bundle, open_leaders=lambda _:leaders, clock=clock)
    view = None
    paired = {}
    def tick(key=None):
        core.tick(key, clock()); clock.now += core.period_s
        assert core.fault is None, core.fault
    def wait(predicate, timeout=20., *, moving=True):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if moving:
                t=clock()-started
                for side in fixtures.CENTERS:
                    for k in range(4): leaders.offsets[side][k]=fixtures.HANG+.025*math.sin(4*t+.4*k)
            tick()
            if predicate(): return
            # Deliberately slow wall playback; receiver, scene and image timestamps
            # remain actual simulated source time. No validity threshold is widened.
            time.sleep(.06)
        pytest.fail(f'capture did not reach expected state: {view.snapshot()}')
    try:
        core.build()
        tick('I')
        for _ in range(10): tick()
        tick('E')
        for _ in range(40): tick()
        tick('T')
        for _ in range(2000):
            tick()
            if core.session.lifecycle.state=='TELEOP':break
        assert core.session.lifecycle.state=='TELEOP'
        names=core.harness.plant.names
        view=TaskSceneView(SCENE,core.derived['joint_map'],names,directory=tmp_path/'episodes',viewer=False,
            width=160,height=120,metadata={'collection_mode':'scripted_fixture','receiver_identity':core.derived['config']['identity'],
            'bindings':bindings,'clock_playback':'explicitly stepped non-real-time contract','joint_map':core.derived['joint_map']})
        deadline=time.monotonic()+40
        while not view.snapshot()['available']:
            assert not view.health()['error'],view.health()['error']
            assert time.monotonic()<deadline,'actual scene/renderer startup timeout'
            time.sleep(.02)
        class PairedView:
            def show(self,q,qcmd=None,source_time=None,authority='human',command_valid=True):
                paired[source_time]=(np.asarray(q,np.float32),np.asarray(qcmd,np.float32))
                view.show(q,qcmd,source_time,authority,command_valid)
            def close(self): view.close()
        core.harness.view=core.harness.task_view=PairedView()
        core.task_capture=view
        started=clock()
        wait(lambda:view.health()['capture_ready'])
        core.task_action('start_record')
        wait(lambda:(view.snapshot().get('recording') or {}).get('frames',0)>=6)
        core.task_action('stop_record')
        wait(lambda:(view.snapshot().get('recording') or {}).get('complete',False))
        path=Path(view.snapshot()['recording']['path'])
        with h5py.File(path) as file:
            assert file.attrs['complete'] and not file.attrs['capture_eligible']
            assert not file.attrs['training_promoted'] and not file.attrs['sim2real_validated']
            assert set(file['observations/images'])=={'right_wrist','head'}
            q,qcmd=file['observations/qpos'][:],file['observations/qcmd'][:]
            assert q.shape[1]==14 and q.shape[0]>=6
            for stamp,measured,command in zip(file['timestamps/telem'][:],q,qcmd):
                expected=paired[stamp]
                assert np.array_equal(measured,expected[0]) and np.array_equal(command,expected[1])
            for name in ('left_arm_j1','right_arm_j1'):
                assert np.ptp(qcmd[:,names.index(name)])>.001
            assert file['action_valid'][:-1].all() and not file['action_valid'][-1]
            assert np.array_equal(file['actions'][:-1],qcmd[1:])
            assert np.isnan(file['observations/torques'][:]).all()
            for role in ('right_wrist','head'):
                images=file[f'observations/images/{role}'][:]
                assert images.shape[1:]==(120,160,3) and images.std()>10
                assert np.array_equal(file[f'timestamps/camera_{role}'][:],file['timestamps/scene'][:])
            metadata=json.loads(file.attrs['metadata_json'])
            assert metadata['scene']['visual_mode']=='collision_proxy'
            assert metadata['scene']['attachment_assistance'] is False
            assert metadata['clock_playback']=='explicitly stepped non-real-time contract'
        # A fresh recording whose worker dies may leave a partial file, never a
        # second completed episode; normal control health must expose the failure.
        core.task_action('start_record')
        wait(lambda:(view.snapshot().get('recording') or {}).get('active',False) and (view.snapshot().get('recording') or {}).get('frames',0)>=2)
        renderer_pid=view.snapshot()["renderer"]["pid"]
        view._process.kill(); view._process.wait(timeout=3)
        # OS ownership, not a cooperating render loop, removes the descendant.
        def renderer_alive():
            import os,sys
            if sys.platform=="win32":
                import ctypes
                from ctypes import wintypes
                api=ctypes.WinDLL("kernel32",use_last_error=True)
                api.OpenProcess.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.DWORD]
                api.OpenProcess.restype=wintypes.HANDLE
                api.WaitForSingleObject.argtypes=[wintypes.HANDLE,wintypes.DWORD]
                api.CloseHandle.argtypes=[wintypes.HANDLE]
                handle=api.OpenProcess(0x100000,False,renderer_pid)
                if not handle:return False
                try:return api.WaitForSingleObject(handle,0)==258
                finally:api.CloseHandle(handle)
            try:
                os.kill(renderer_pid,0)
                stat=Path(f"/proc/{renderer_pid}/stat")
                return not stat.exists() or stat.read_text().split()[2]!="Z"
            except ProcessLookupError:return False
        deadline=time.monotonic()+3
        while renderer_alive() and time.monotonic()<deadline:time.sleep(.01)
        assert not renderer_alive(),"camera renderer survived scene-owner death"
        deadline=time.monotonic()+3
        while not view.health()['failed'] and time.monotonic()<deadline: time.sleep(.01)
        assert view.health()['failed'] and not view.health()['capture_ready']
        with pytest.raises(RecordingError): core._check_recording()
        assert len([p for p in (tmp_path/'episodes').glob('*.hdf5') if '.partial.' not in p.name])==1
    finally:
        core.close()
        if view is not None: view.close()
