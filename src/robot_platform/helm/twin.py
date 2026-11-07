"""Helm's digital twin correction, for drawing only (vision.twin): what the newest wrist calibration and the head
camera's saved alignment say about the arms' joint zeros and the head's pan zero, Apply to save it beside the head's
files, Remove to take it away (each one replaced or removed is kept under <history>). Helm's 3D view and the headset's
model draw the measured arms and neck through it at once; the head camera's pose, which places the headset's depth,
takes it from the next session (helm.head_teleop). Nothing here moves a motor or changes a zero.
"""

from contextlib import contextmanager
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import shutil
import threading
import time

import numpy as np

from robot_platform.control.joint_calibration import save_json
from robot_platform.neck import head
from robot_platform.vision import twin
from robot_platform.vision.board import load_board
from robot_platform.vision.chain import Chain
from robot_platform.vision.head_alignment import DEFAULT_RIG, board_middle, load_rig

PROPOSAL_S = 5.0  # The proposal is worked out again at most this often for the page's state,
CHECK_S = 2.0  # and the saved correction checked against the references in use at most this often for the 3D views.


class TwinCorrection:
    """`wrists`: the wrist calibration's runs; `files()`: the head's files (neck.head.files); `path()`: the
    correction's file (vision.twin.path); `arms_reference()`: the arms' mechanical reference digest Helm's session
    binds; `teleop()`: the teleop session's snapshot (helm.teleop). Call apply/clear from worker threads."""

    def __init__(self, *, wrists, files, path, arms_reference, teleop, history, rig=DEFAULT_RIG, clock=time.time):
        config = load_rig(rig)
        self.chain = Chain(config['urdf'], config['link'])  # The head's, to its camera: the pan is its first joint.
        self.middle = board_middle(load_board(config['board']))
        self.wrists = Path(os.path.expanduser(str(wrists)))
        self.history = Path(os.path.expanduser(str(history)))
        self.files, self.path, self.arms_reference, self.teleop = files, path, arms_reference, teleop
        self.clock = clock
        self._lock = threading.Lock()
        self._check = threading.Lock()  # A check under way never outlives an Apply or Remove that follows it.
        self.busy, self.error = False, None
        self._proposal, self._proposed = None, -math.inf
        self._drawing, self._checked = None, -math.inf
        self.revision = 0  # Counts the corrections the 3D views were handed: each change is a pose of its own to them.

    def _head(self):
        """The head's saved camera alignment and reference (None for either not saved)."""
        files = self.files()
        return head.load_document(files['camera']), head.load_document(files['reference'])

    def _saved(self):
        try:
            return twin.load(self.path()), None
        except (OSError, ValueError) as exc:
            return None, f'The saved correction cannot be read: {exc}'

    # -- the proposal -------------------------------------------------------------------------------------------

    def proposal(self, fresh=False):
        """What Apply would save now (vision.twin.propose)."""
        if fresh or self._proposal is None or self.clock() - self._proposed > PROPOSAL_S:
            try:
                camera, reference = self._head()
                self._proposal = twin.propose(
                    self.wrists, self.arms_reference(), camera, reference, self.chain, self.middle
                )
            except (OSError, ValueError) as exc:
                self._proposal = dict(
                    wrists_run=None, arms=None, head=None, notes=[f"The head's files cannot be read: {exc}"]
                )
            self._proposed = self.clock()
        return self._proposal

    @contextmanager
    def _claim(self):
        if not self._lock.acquire(blocking=False):
            raise ValueError('The twin correction is being saved; wait for it')
        try:
            yield
        finally:
            self._lock.release()

    def _keep(self, what, now):
        """The correction in place, kept under the history folder before it is replaced or removed."""
        target = self.path()
        if target.is_file():
            self.history.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, self.history / f'{what}-{now.strftime("%Y%m%dT%H%M%SZ")}-{target.name}')

    def apply(self):
        """Save the proposal as the correction the 3D views draw through (and the head's next session)."""
        with self._claim():
            self.busy, self.error = True, None
            try:
                now = datetime.fromtimestamp(self.clock(), timezone.utc)
                document = twin.document(self.proposal(fresh=True), saved_at=now.isoformat())
                self._keep('replaced', now)
                self.path().parent.mkdir(parents=True, exist_ok=True)
                with self._check:
                    save_json(self.path(), document)
                    self._drawing = None
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.busy = False
        return self.snapshot()

    def clear(self):
        """Remove the correction: the 3D views draw the measured angles again (the head's camera from its next
        session)."""
        with self._claim():
            self.error = None
            if not self.path().is_file():
                raise ValueError('No twin correction is applied')
            self._keep('removed', datetime.fromtimestamp(self.clock(), timezone.utc))
            with self._check:
                self.path().unlink()
                self._drawing = None
        return self.snapshot()

    # -- the drawing --------------------------------------------------------------------------------------------

    def drawing(self):
        """What the 3D views add to measured angles: {'arms': {joint: rad}, 'head': {joint: rad}, 'notes': why a saved
        part does not apply, 'revision'}, checked against the references in use at most every CHECK_S."""
        with self._check:
            if self._drawing is None or self.clock() - self._checked > CHECK_S:
                self._drawing = self._checked_drawing()
                self._checked = self.clock()
            return self._drawing

    def _checked_drawing(self):
        arms, neck = {}, {}
        document, problem = self._saved()
        notes = [problem] if problem else []
        if document is not None and document.get('arms') is not None:
            try:
                arms = twin.arm_offsets(document, self.arms_reference())
            except ValueError as exc:
                notes.append(f'Arms not corrected: {exc}')
        if document is not None and document.get('head') is not None:
            try:
                camera, reference = self._head()
                neck = twin.head_offsets(document, reference, camera)
            except (OSError, ValueError) as exc:
                notes.append(f'Head not corrected: {exc}')
        if self._drawing is None or (arms, neck) != (self._drawing['arms'], self._drawing['head']):
            self.revision += 1
        return dict(arms=arms, head=neck, notes=notes, revision=self.revision)

    def draw(self, pose, *, neck=False):
        """`pose`, a guide the 3D views draw, through the correction: the arms' joints when it is measured (live, from
        the real motors), the neck's when `neck` (its angles measured by the head's session or setup). Anything else
        passes unchanged."""
        if not isinstance(pose, dict) or not isinstance(pose.get('positions'), dict):
            return pose
        drawing = self.drawing()
        offsets = dict(drawing['arms']) if pose.get('live') and pose.get('measured', True) else {}
        if neck:
            offsets.update(drawing['head'])
        positions = pose['positions']
        offsets = {name: value for name, value in offsets.items() if type(positions.get(name)) in (int, float)}
        if not offsets:
            return pose
        badge = pose.get('badge')
        return dict(
            pose,
            positions={**positions, **{name: positions[name] + v for name, v in offsets.items()}},
            stream_id=f'{pose.get("stream_id") or "guide"}+twin{drawing["revision"]}',
            badge=f'{badge} · twin corrected' if badge else 'Twin corrected',
        )

    def camera_turn(self):
        """The turn (4x4, Minerva's frame) the head camera's poses take from the correction: the running live head's
        (it reports the offsets its camera uses), else the saved one's, which the next session takes. A map's world
        moves with it, so a capture's frames and the headset's depth stay in one frame (helm.maps)."""
        view = self.teleop() or {}
        offsets = (view.get('head') or {}).get('camera_offsets') if view.get('phase') == 'running' else None
        if offsets is None:
            offsets = self.drawing()['head']
        return twin.correction(self.chain, offsets) if offsets else np.eye(4)

    # -- state --------------------------------------------------------------------------------------------------

    def snapshot(self):
        document, problem = self._saved()
        drawing = self.drawing()
        proposal = self.proposal()
        applied = (
            None
            if document is None
            else dict(
                saved_at=document.get('saved_at'),
                wrists_run=document.get('wrists_run'),
                arms=document.get('arms'),
                head=document.get('head'),
                current=all(document.get(key) == proposal.get(key) for key in ('arms', 'head')),
            )
        )
        return dict(
            busy=self.busy,
            error=self.error or problem,
            proposal=proposal,
            applied=applied,
            drawing=dict(arms=len(drawing['arms']), head=len(drawing['head']), notes=drawing['notes']),
        )
