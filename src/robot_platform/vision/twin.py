"""The digital twin's correction, for drawing only (docs/development/HEADSET_3D_PLAN.md, Twin correction): where the
arms and the head are, from what their joints report and the table's board.

- The arms: the joint zero offsets the wrist calibration fitted (vision.solve's 'joint_offsets' model, J1-J5 of each
  arm; J6's is its camera's mount), when that model is the recommended one.
- The head: its pan zero, from its camera's saved alignment against the same wrist run's board
  (vision.head_alignment.against_arms): the turn about the pan axis that puts the head's placement of the board on the
  arms'. The tilt's zero is the camera mount's; the rest of the difference (the neck's place on the frame against
  CAD) is reported, not drawn.
A joint is drawn at its measured angle plus its offset, as vision.chain adds a fitted offset. Each part holds only for
what it was made through: the arms' for their mechanical reference (each wrist still records it), the head's for the
head reference and the camera alignment it was compared with. Motor zeros, control and collision keep the measured
angles.
"""
import json
import math
from pathlib import Path

import numpy as np

from robot_platform.neck.head_pose import aligned_mount, reference_digest
from .geometry import invert

KIND = 'minerva_twin_correction'
# A zero further off than this is not a zero error: the stills, or a reference, are wrong.
MAX_OFFSET_DEG = 10.
# Beside the pan, the neck's place on the frame against CAD puts the head's board and the arms' this far apart at most;
# farther, and the board moved between the wrist stills and the head's.
MAX_OTHER_MM, MAX_OTHER_DEG = 30., 3.


def path(directory, asset_id):
    """The correction's file, beside the head's (neck.head.files: the operator's files)."""
    return Path(directory) / f'{asset_id}-twin.json'


def load(file):
    """A saved correction, or None when there is none; ValueError when it is not one."""
    file = Path(file)
    if not file.is_file():
        return None
    return _checked(json.loads(file.read_text(encoding='utf-8')))


def _checked(document):
    if not isinstance(document, dict) or document.get('kind') != KIND or document.get('schema') != 1:
        raise ValueError('not a twin correction')
    return document


def _small(offsets_deg, what):
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in offsets_deg.values()):
        raise ValueError(f'{what} has an offset that is not a finite angle')
    beyond = [name for name, v in offsets_deg.items() if abs(v) > MAX_OFFSET_DEG]
    if beyond:
        raise ValueError(f"{what} would turn {', '.join(beyond)} more than {MAX_OFFSET_DEG:.0f}°, beyond a zero error")


def arms_part(run, result, reference):
    """The arms' part from a solved wrist run (its folder and result.json): its 'joint_offsets' model's offsets
    ({joint: deg}, with their sigmas), every still taken through the arms' mechanical reference `reference` (its
    digest, as Helm's session binds it). None when the plain kinematics fit the stills better; ValueError says why
    the run gives none."""
    if result.get('recommended') != 'joint_offsets':
        return None
    stills = [json.loads(line) for line in (Path(run) / 'stills.jsonl').read_text(encoding='utf-8').splitlines()
              if line.strip()]
    references = {still.get('reference_sha256') for still in stills}
    if None in references:
        raise ValueError(f"run {Path(run).name}'s stills do not say which arms' reference they were taken through "
                         "(before Helm recorded it): capture a new wrist run")
    if references != {reference}:
        raise ValueError(f'the arms were zeroed again since run {Path(run).name}: capture a new wrist run')
    offsets, sigmas = {}, {}
    for camera in result['models']['joint_offsets']['cameras'].values():
        for joint, value in (camera.get('joint_offsets') or {}).items():
            offsets[joint], sigmas[joint] = value['deg'], value.get('sigma_deg')
    if not offsets:
        raise ValueError(f'run {Path(run).name} fitted no joint offsets')
    _small(offsets, f'run {Path(run).name}')
    return dict(run=Path(run).name, reference_sha256=reference, offsets_deg=offsets, sigma_deg=sigmas)


def head_part(camera, reference, placement, chain, middle, wrists_run):
    """The head's part from its saved camera alignment (`camera`, made through the head `reference`) against the arms'
    placement of the board (4x4, from the wrist run named `wrists_run`): the pan zero, and how far apart the two boards
    are beside it. `chain`: the head model's to the camera's link (vision.chain); `middle`: where on the board they are
    compared. ValueError says why there is none."""
    from .head_alignment import PAN, against_arms  # The solve's module brings scipy: the head's process never needs it.
    aligned_mount(camera, chain.tip, reference)
    found = against_arms(np.asarray(camera['board_in_robot']['matrix'], float), placement, chain, middle)
    if found['other_mm'] > MAX_OTHER_MM or found['other_deg'] > MAX_OTHER_DEG:
        raise ValueError(f"beside the pan its board and the arms' are {found['other_mm']:.0f} mm and "
                         f"{found['other_deg']:.1f}° apart (at most {MAX_OTHER_MM:.0f} mm, {MAX_OTHER_DEG:.0f}°): "
                         "was the board moved between the wrist and head stills? Take both with it in one place")
    _small({PAN: found['pan_deg']}, 'the head camera alignment')
    return dict(alignment_run=camera.get('run'), camera_sha256=reference_digest(camera),
                reference_sha256=reference_digest(reference), wrists_run=wrists_run,
                offsets_deg={PAN: found['pan_deg']}, board_mm=found['board_mm'], other_mm=found['other_mm'],
                other_deg=found['other_deg'])


def propose(wrists, arms_reference, camera, head_reference, chain, middle):
    """What Apply saves: {'wrists_run', 'arms': part or None, 'head': part or None, 'notes': why a part is missing},
    from the newest solved wrist run under `wrists`, the arms' reference digest, and the head's saved camera alignment
    and reference (None when not saved)."""
    from .head_alignment import latest_wrist_run
    latest = latest_wrist_run(wrists)
    if latest is None:
        return dict(wrists_run=None, arms=None, head=None,
                    notes=['No solved wrist calibration: capture wrist stills in a live session, then Solve'])
    run, result, placement = latest
    parts, notes = {'arms': None, 'head': None}, []
    try:
        parts['arms'] = arms_part(run, result, arms_reference)
        if parts['arms'] is None:
            notes.append(f"Arms: the URDF's joint zeros fit run {run.name} best (its plain model): nothing to correct")
    except ValueError as exc:
        notes.append(f'Arms: {exc}')
    except (OSError, KeyError, TypeError):
        notes.append(f"Arms: run {run.name}'s stills or joint offsets cannot be read")
    if camera is None or head_reference is None:
        notes.append('Head: no head camera alignment saved' if camera is None else 'Head: the head has no reference')
    else:
        try:
            parts['head'] = head_part(camera, head_reference, placement, chain, middle, run.name)
        except ValueError as exc:
            notes.append(f'Head: {exc}')
        except (KeyError, TypeError):
            notes.append("Head: the head camera's alignment file cannot be read")
    return dict(wrists_run=run.name, **parts, notes=notes)


def document(proposal, *, saved_at):
    """The correction's file from a proposal with a part to apply."""
    if proposal.get('arms') is None and proposal.get('head') is None:
        raise ValueError('Nothing to correct: ' + '; '.join(proposal.get('notes') or ['no proposal']))
    return dict(schema=1, kind=KIND, wrists_run=proposal['wrists_run'], arms=proposal['arms'], head=proposal['head'],
                saved_at=saved_at)


def _radians(part, what):
    offsets = part.get('offsets_deg')
    if not isinstance(offsets, dict) or not offsets:
        raise ValueError(f'its {what} part has no offsets')
    _small(offsets, f'its {what} part')
    return {name: math.radians(value) for name, value in offsets.items()}


def arm_offsets(document, reference):
    """{joint: rad} the arms are drawn turned by beyond their measured angles: a saved correction's arms' part, for the
    arms' mechanical reference in use (its digest). ValueError says why none applies."""
    part = _checked(document).get('arms')
    if part is None:
        raise ValueError("it has no arms' part")
    if part.get('reference_sha256') != reference:
        raise ValueError('the arms were zeroed again since it was applied: calibrate the wrist cameras again')
    return _radians(part, "arms'")


def head_offsets(document, reference, camera):
    """{joint: rad} the neck is drawn turned by: a saved correction's head part, for the head reference and the head
    camera alignment in use (`camera`: its saved file, or None). ValueError says why none applies."""
    part = _checked(document).get('head')
    if part is None:
        raise ValueError('it has no head part')
    if reference is None or part.get('reference_sha256') != reference_digest(reference):
        raise ValueError('the head has another reference since it was applied: align the head camera again')
    if camera is None:
        raise ValueError('the head camera has no saved alignment')
    if part.get('camera_sha256') != reference_digest(camera):
        raise ValueError('the head camera was aligned again since it was applied: apply the correction again')
    return _radians(part, 'head')


def correction(chain, offsets):
    """The fixed turn (4x4, in the chain's root frame) that drawing the chain's first moving joint through its offset
    gives everything hanging from it: a pose or a placement worked out on the measured angles, moved onto the drawn
    ones. Only the first joint (the head's pan) may have an offset: a later one's turn depends on the joints before."""
    if set(offsets) - set(chain.joints[:1]):
        raise ValueError(f"Only {chain.joints[0]}'s offset is a fixed turn of what hangs from it")
    zero = {name: 0. for name in chain.joints}
    return chain.pose(zero, offsets) @ invert(chain.pose(zero))
