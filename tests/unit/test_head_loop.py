"""A live head's cycles on a thread of their own (helm.head_teleop.HeadLoop), beside the arms' loop: the arms'
actions in order, its own grid between them, a fault ending it, and a prompt stop."""
import threading
import time

from robot_platform.helm.head_teleop import PERIOD_S, HeadLoop


class Head:
    """HeadTeleop's face to its loop: cycle(action, started) and the next slot it is due. Its cycles wait while
    `busy` is clear, as a cycle waits for its receiver."""

    def __init__(self, failing=None):
        self.fault, self.due, self.failing = None, None, failing
        self.actions, self.cycles = [], 0
        self.busy = threading.Event()
        self.busy.set()

    def cycle(self, action, started):
        self.busy.wait(2.)
        self.cycles += 1
        if action is not None:
            self.actions.append(action)
        if action is not None and action == self.failing:
            self.fault = f'Head: {action} failed'
            raise RuntimeError(self.fault)
        if self.due is None or started >= self.due - PERIOD_S / 4:
            self.due = started + PERIOD_S


def wait_for(condition, timeout=2.):
    deadline = time.perf_counter() + timeout
    while not condition() and time.perf_counter() < deadline:
        time.sleep(.005)
    return condition()


def test_the_head_takes_the_arms_actions_in_order_and_cycles_on_its_grid_between_them():
    head = Head()
    head.busy.clear()  # Its first cycle waits for its receiver while the arms' actions arrive.
    loop = HeadLoop(head).start()
    try:
        for action in ('idle', 'enable', 'enable', 'engage'):  # A repeat (a faulted loop's, each tick) is one.
            loop.post(action)
        loop.post(None)  # Nothing to take.
        assert loop.pending
        head.busy.set()
        assert wait_for(lambda: len(head.actions) == 3 and not loop.pending)
        assert head.actions == ['idle', 'enable', 'engage']
        cycles = head.cycles
        time.sleep(10 * PERIOD_S)
        assert 5 <= head.cycles - cycles <= 15  # About one a period on its own.
        loop.post('disengage')
        assert wait_for(lambda: head.actions[-1:] == ['disengage'])
    finally:
        head.busy.set()
        loop.stop()


def test_a_fault_ends_the_heads_thread_and_it_takes_nothing_after():
    head = Head(failing='engage')
    loop = HeadLoop(head).start()
    try:
        loop.post('idle')
        loop.post('engage')
        assert wait_for(lambda: not loop._thread.is_alive())
        assert head.fault == 'Head: engage failed' and head.actions == ['idle', 'engage']
        loop.post('disable')
        time.sleep(3 * PERIOD_S)
        assert head.actions == ['idle', 'engage']
    finally:
        loop.stop()


def test_stop_ends_the_heads_cycles_at_once():
    head = Head()
    loop = HeadLoop(head).start()
    assert wait_for(lambda: head.cycles >= 2)
    started = time.perf_counter()
    loop.stop()
    assert time.perf_counter() - started < .5 and not loop._thread.is_alive()
    cycles = head.cycles
    time.sleep(3 * PERIOD_S)
    assert head.cycles == cycles


def test_the_heads_thread_moves_off_the_arms_cpus_before_its_first_cycle():
    # Beside the head, the arms' receiver started its ticks up to 9.2 ms late on their shared CPUs and an arm command
    # expired (2026-10-31): the head's thread places itself (control.scheduling) on itself, first.
    head, seen = Head(), []

    def placement():
        seen.append((threading.get_ident(), head.cycles))
        return ['background_cpus']
    loop = HeadLoop(head, placement=placement).start()
    try:
        assert wait_for(lambda: head.cycles >= 2)
        assert seen == [(loop._thread.ident, 0)] and loop.placement == ['background_cpus']
    finally:
        loop.stop()

    def refused():
        raise OSError('refused')
    head = Head()
    loop = HeadLoop(head, placement=refused).start()  # A hint, not a condition: it cycles where it is.
    try:
        assert wait_for(lambda: head.cycles >= 2) and loop.placement == []
    finally:
        loop.stop()
