"""A LeRobot policy trained on Minerva's exported demonstrations, exported for the robot, and loaded there.

Export (where LeRobot is installed, e.g. coolserver's `.venv-lerobot`):

    python -m robot_platform.data.lerobot_policy export <pretrained_model dir> <out dir> \\
        --dataset-info <dataset>/meta/info.json [--fps 15] [--device cuda:0]

writes `<out>/policy.pt`: LeRobot's input normalization, the network and its output unnormalization traced into one
TorchScript module (inputs: the state (1, J) and one (1, 3, H, W) 0..1 RGB image per camera; output: the whole
(1, chunk, J) chunk, row k meant for k / fps after the observation). `minerva_policy.json` beside it names the
joints, fps, cameras and image shape, and `sample.pt` holds one input and its output for checking another machine.
The export refuses a module whose chunk does not start with LeRobot's own `select_action` or that differs from
LeRobot by more than 1 mrad.

Load (`ExportedPolicy`, evaluation.policies `lerobot:<dir>`): plain torch, no LeRobot, so the Jetson keeps its own
Python environment. On the Jetson (torch 2.8, Orin): 37 ms a chunk once warm; the first two calls take seconds
while TorchScript optimizes, so loading warms the module up before the policy reports ready.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

KIND = 'minerva_torchscript_policy'
EXPORT_TOLERANCE_RAD = 1e-3  # GPU kernels of the traced graph differ from LeRobot's by ~1e-4 rad.


def validate_meta(meta):
    """An exported policy's description, checked: joints, fps, cameras and image shape."""
    if not isinstance(meta, dict) or meta.get('kind') != KIND or meta.get('schema') != 1:
        raise ValueError('Not an exported Minerva policy (minerva_policy.json)')
    joints, roles, shape = meta.get('joints'), meta.get('camera_roles'), meta.get('image_shape_chw')
    fps = meta.get('fps')
    if not isinstance(joints, list) or not joints or not all(isinstance(j, str) and j for j in joints):
        raise ValueError('An exported policy names the joints it was trained on')
    if not isinstance(roles, list) or not all(isinstance(r, str) and r for r in roles) or len(set(roles)) != len(roles):
        raise ValueError('An exported policy names its cameras once each')
    if (not isinstance(shape, list) or len(shape) != 3 or shape[0] != 3
            or not all(type(v) is int and v > 0 for v in shape)):
        raise ValueError('An exported policy takes RGB images of one size (3, H, W)')
    if type(fps) not in (int, float) or not math.isfinite(fps) or fps <= 0:
        raise ValueError('An exported policy says its chunk rate (fps)')
    return meta


def torchscript_runner(path, device=None):
    """`run(state, images) -> chunk` for a TorchScript export: state (J,), images {role: (H, W, 3) uint8}."""
    import numpy as np
    import torch
    device = torch.device(device or ('cuda' if torch.cuda.is_available() else 'cpu'))
    module = torch.jit.load(str(path), map_location=device).eval()

    def run(state, images):
        with torch.inference_mode():
            q = torch.as_tensor(np.asarray(state, dtype=np.float32), device=device)[None]
            frames = [torch.as_tensor(np.ascontiguousarray(image), device=device).permute(2, 0, 1)[None].float()
                      .div_(255.) for image in images]
            return module(q, frames)[0].float().cpu().numpy()
    run.device = str(device)
    return run


class ExportedPolicy:
    """An exported policy as the evaluation harness and the teleop loop's policy worker call it."""

    def __init__(self, path, *, device=None, warmup=3, runner=None):
        import numpy as np
        runner = runner or torchscript_runner
        self.path = Path(path)
        self.meta = validate_meta(json.loads((self.path / 'minerva_policy.json').read_text(encoding='utf-8')))
        self.joints, self.camera_roles = tuple(self.meta['joints']), tuple(self.meta['camera_roles'])
        self.action_dt = 1. / float(self.meta['fps'])
        self.image_shape = tuple(self.meta['image_shape_chw'][1:]) + (3,)
        self.name = f"lerobot:{self.meta.get('policy_type', 'policy')}:{self.path.name}"
        self._run = runner(self.path / 'policy.pt', device)
        # TorchScript optimizes on its first calls (seconds on the Jetson): do them now, not in the first rollout.
        blank = np.zeros(self.image_shape, dtype=np.uint8)
        for _ in range(warmup):
            self._check(self._run(np.zeros(len(self.joints)), [blank] * len(self.camera_roles)))

    def _check(self, chunk):
        import numpy as np
        chunk = np.asarray(chunk, dtype=np.float64)
        if chunk.ndim != 2 or chunk.shape[1] != len(self.joints) or not np.all(np.isfinite(chunk)):
            raise ValueError(f'The exported policy answered {chunk.shape}, not a finite (rows, {len(self.joints)}) chunk')
        return chunk

    def reset(self, context):
        if tuple(context.joint_names) != self.joints:
            raise ValueError(f'The policy was trained on the joints {list(self.joints)}, '
                             f'not this receiver\'s {list(context.joint_names)}')

    def __call__(self, observation):
        import numpy as np
        images = observation.get('images') or {}
        frames = []
        for role in self.camera_roles:
            image = images.get(role)
            if image is None:
                raise ValueError(f'The policy needs the {role} camera')
            image = np.asarray(image)
            if image.shape != self.image_shape or image.dtype != np.uint8:
                raise ValueError(f'The {role} camera gave {image.shape} {image.dtype}, the policy takes '
                                 f'{self.image_shape} uint8 as the recording stores them')
            frames.append(image)
        q = np.asarray(observation['q'], dtype=np.float64)
        if q.shape != (len(self.joints),) or not np.all(np.isfinite(q)):
            raise ValueError('The policy needs every joint\'s measured position')
        return self._check(self._run(q, frames))

    def describe(self):
        return {'kind': 'exported LeRobot policy (TorchScript)', 'path': str(self.path),
                'device': getattr(self._run, 'device', None),
                **{key: self.meta.get(key) for key in ('policy_type', 'fps', 'joints', 'camera_roles',
                                                       'image_shape_chw', 'chunk_size', 'checkpoint_sha256',
                                                       'traced_max_abs_error')}}


def export(checkpoint, out, *, dataset_info, fps=15., device=None):
    """Trace a LeRobot ACT checkpoint into `out` (see the module docs); returns its minerva_policy.json."""
    import torch
    from lerobot.common.policies.act.modeling_act import ACTPolicy

    class Chunked(torch.nn.Module):
        """select_action's steps at the pinned LeRobot revision, without its action queue: the whole chunk."""

        def __init__(self, policy, keys):
            super().__init__()
            self.policy, self.keys = policy, list(keys)

        def forward(self, state, images):
            batch = {'observation.state': state}
            for key, image in zip(self.keys, images):
                batch[key] = image
            batch = dict(self.policy.normalize_inputs(batch))
            batch['observation.images'] = [batch[key] for key in self.keys]
            return self.policy.unnormalize_outputs({'action': self.policy.model(batch)[0]})['action']

    checkpoint, out = Path(checkpoint), Path(out)
    config = json.loads((checkpoint / 'config.json').read_text(encoding='utf-8'))
    if config['type'] != 'act':
        raise ValueError(f"Only ACT exports so far, not {config['type']}")
    device = device or ('cuda:0' if torch.cuda.is_available() else 'cpu')
    policy = ACTPolicy.from_pretrained(str(checkpoint)).to(device).eval()
    features = policy.config.image_features
    keys = list(features)
    roles = [key.split('observation.images.', 1)[1] for key in keys]
    channels, height, width = tuple(features[keys[0]].shape)
    joints = json.loads(Path(dataset_info).read_text(encoding='utf-8'))['features']['observation.state']['names']
    torch.manual_seed(0)
    state = torch.randn(1, len(joints), device=device) * .3
    images = [torch.rand(1, channels, height, width, device=device) for _ in keys]
    module = Chunked(policy, keys).eval()
    with torch.inference_mode():
        eager = module(state, images)
        policy.reset()
        first = policy.select_action({'observation.state': state, **dict(zip(keys, images))})
    if not torch.allclose(first, eager[:, 0], atol=1e-5):
        raise ValueError("The chunk does not start with LeRobot's select_action")
    with torch.no_grad():
        traced = torch.jit.freeze(torch.jit.trace(module, (state, images), check_trace=False).eval())
        error = float((traced(state, images) - eager).abs().max())
    if error > EXPORT_TOLERANCE_RAD:
        raise ValueError(f'The traced module differs from LeRobot by {error:.2e} rad')
    out.mkdir(parents=True, exist_ok=True)
    traced.save(str(out / 'policy.pt'))
    torch.save({'state': state.cpu(), 'images': [image.cpu() for image in images], 'chunk': eager.cpu()},
               out / 'sample.pt')
    weights = checkpoint / 'model.safetensors'
    meta = validate_meta({'kind': KIND, 'schema': 1, 'policy_type': 'act', 'fps': float(fps), 'joints': joints,
                          'camera_roles': roles, 'image_shape_chw': [channels, height, width],
                          'image_scale': '0..1 float RGB, as LeRobot decodes the dataset videos',
                          'chunk_size': int(eager.shape[1]), 'checkpoint': str(checkpoint),
                          'checkpoint_sha256': hashlib.sha256(weights.read_bytes()).hexdigest()
                          if weights.is_file() else None,
                          'traced_max_abs_error': error, 'torch': torch.__version__})
    (out / 'minerva_policy.json').write_text(json.dumps(meta, indent=1) + '\n', encoding='utf-8')
    return meta


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m robot_platform.data.lerobot_policy', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    exporting = commands.add_parser('export', help='Trace a LeRobot ACT checkpoint for the robot')
    exporting.add_argument('checkpoint', type=Path, help='The pretrained_model directory a training run saved')
    exporting.add_argument('out', type=Path)
    exporting.add_argument('--dataset-info', type=Path, required=True, help="The dataset's meta/info.json")
    exporting.add_argument('--fps', type=float, default=15.)
    exporting.add_argument('--device')
    args = parser.parse_args(argv)
    meta = export(args.checkpoint, args.out, dataset_info=args.dataset_info, fps=args.fps, device=args.device)
    print(json.dumps({key: meta[key] for key in ('policy_type', 'fps', 'camera_roles', 'chunk_size',
                                                  'traced_max_abs_error')}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
