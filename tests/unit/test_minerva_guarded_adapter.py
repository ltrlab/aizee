"""Offline checks for the existing GUI's guarded virtual adapter."""
import ast
from pathlib import Path
import queue
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def module(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "python/scripts"))
    from collect_minerva_app import guarded
    return guarded


class Runtime:
    fault = None

    def __init__(self):
        self.calls = []
        self.closed = 0

    def tick(self, key=None):
        self.calls.append(key)
        return {"state": "IDLE", "message": "test snapshot"}

    def close(self):
        self.closed += 1


@pytest.mark.parametrize("key", ["I", "E", "T", "H", "X"])
def test_existing_keys_reach_runtime_unchanged(module, key):
    runtime = Runtime()
    adapter = module.ShadowGuiAdapter(runtime)
    assert adapter.tick(key)["guarded_shadow"] is True
    assert runtime.calls == [key]
    adapter.close()
    adapter.close()
    assert runtime.closed == 1


def test_unsupported_action_keeps_tick_but_never_dispatches_legacy_command(module):
    runtime = Runtime()
    adapter = module.ShadowGuiAdapter(runtime)
    snapshot = adapter.tick("K")
    assert runtime.calls == [None]
    assert snapshot["message"] == module.UNSUPPORTED
    assert adapter.toggle_swap() is None
    assert adapter.jog(0, 1) is None
    assert runtime.calls == [None]


def test_tick_exception_closes_then_latches_without_rearm(module):
    runtime = Runtime()
    def failed(key):
        runtime.calls.append(key)
        raise TimeoutError("source stale")
    runtime.tick = failed
    adapter = module.ShadowGuiAdapter(runtime)
    snapshot = adapter.tick("E")
    assert snapshot["state"] == "FAULT"
    assert "source stale" in snapshot["message"]
    assert not snapshot["both_ok"] and snapshot["telem_age"] is None
    assert runtime.closed == 1
    adapter.tick("I")
    adapter.tick("E")
    adapter.close()
    assert runtime.calls == ["E"] and runtime.closed == 1


def test_runtime_fault_snapshot_also_closes_once(module):
    runtime = Runtime()
    runtime.fault = "collision rejection"
    adapter = module.ShadowGuiAdapter(runtime)
    assert adapter.tick()["state"] == "FAULT"
    assert runtime.closed == 1
    adapter.tick("T")
    assert runtime.calls == [None]


def test_close_failure_remains_visible(module):
    runtime = Runtime()
    def failed_close():
        raise RuntimeError("receiver unavailable")
    runtime.close = failed_close
    adapter = module.ShadowGuiAdapter(runtime)
    adapter.close()
    assert adapter.fault
    assert adapter.snapshot["state"] == "FAULT"
    assert "receiver unavailable" in adapter.snapshot["message"]


def test_main_branches_before_any_legacy_setup(module, monkeypatch):
    path = Path(__file__).resolve().parents[2] / "python/scripts/collect_minerva.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    entry = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
    args = SimpleNamespace(guarded_shadow=True, gui=False)
    called = []
    monkeypatch.setattr(module, "run_guarded", lambda value: called.append(value) or 42)
    # Legacy configuration, host discovery and transport names are deliberately absent.
    namespace = {"parse_args": lambda: args}
    exec(compile(ast.Module(body=[entry], type_ignores=[]), str(path), "exec"), namespace)
    assert namespace["main"]() == 42
    assert called == [args]


def test_terminal_uses_existing_keys_and_closes_on_input_eof(module):
    commands = queue.Queue()
    module._terminal_commands(commands, ["i\n", "e\n", "t\n", "x\n"])
    assert [commands.get_nowait() for _ in range(5)] == ["I", "E", "T", "X", "Q"]


def test_runner_closes_on_quit_without_starting_legacy_io(module, monkeypatch):
    runtime = Runtime()
    fake_runtime_module = ModuleType("robot_platform.teleop.shadow_runtime")
    fake_runtime_module.ShadowTeleopRuntime = lambda *args, **kwargs: runtime
    monkeypatch.setitem(sys.modules, fake_runtime_module.__name__, fake_runtime_module)
    commands = queue.Queue()
    commands.put("I")
    commands.put("Q")
    monkeypatch.setattr(module.queue, "Queue", lambda **kwargs: commands)
    monkeypatch.setattr(module.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    args = SimpleNamespace(model="model.json", leader_display_calibration="display.json",
                           leader_url="http://127.0.0.1:8767/", gui=False, output_dir="unused")
    assert module.run_guarded(args) == 0
    assert runtime.calls == ["I"]
    assert runtime.closed == 1
