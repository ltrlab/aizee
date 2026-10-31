"""Train a 3D Gaussian Splatting scene from a head scene capture (docs/development/HEADSET_3D_PLAN.md, phase 4).

Runs on the operator station soliloquy, in its Ubuntu (WSL2) venv with CUDA PyTorch and gsplat (setup_wsl.sh), not in
the platform: the robot never trains. The capture is a folder:

    scene.json   {"schema": 1, "kind": "head_scene", "color_intrinsics": {fx, fy, cx, cy, width, height},
                  "depth_intrinsics": {...}, "to_color": {"rotation", "translation"}, "depth_scale": metres a unit,
                  "frames": [{"color": "color/000000.jpg", "depth": "depth/000000.u16.z",
                              "pose": [x, y, z, qx, qy, qz, qw]}, ...]}
    color/*.jpg  the head camera's colour pictures as published
    depth/*.u16.z its depth, zlib-compressed little-endian uint16 at depth_intrinsics' size

Each pose is the colour camera's optical frame (x right, y down, z ahead) in Minerva's robot frame, from the head's
measured pose (neck.head_pose): the trained scene is in the robot frame, so the headset places it as it places the
model, by the zero mapping M.

Gaussians start from the depth, back-projected into the robot frame and coloured from the pictures; then the
colour (L1 and SSIM) and the depth (where the sensor read one) are fitted with gsplat's rasterizer and its default
densification. Every eighth frame is held out and scored (PSNR). The result is a standard 3DGS PLY (positions,
spherical harmonics, opacity, scales, rotations), as three.js splat renderers read it.

    python train_head_scene.py <capture> <out.ply> [--steps 7000] [--sh 3]
"""
import argparse
import io
import json
import math
from pathlib import Path
import time
import zlib

import numpy as np
from PIL import Image

HOLD_OUT = 8  # Every eighth frame scored, never trained on.
VOXEL_M = .01  # The starting Gaussians: one per centimetre cube of depth.
DEPTH_RANGE_M = (.15, 3.)
DEPTH_WEIGHT = .2  # Of the depth's L1 (metres) in the loss.
SSIM_WEIGHT = .2


def pose_matrix(pose):
    x, y, z, qx, qy, qz, qw = pose
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                 [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                 [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    m[:3, 3] = x, y, z
    return m


def intrinsics(k):
    return np.array([[k['fx'], 0, k['cx']], [0, k['fy'], k['cy']], [0, 0, 1]], dtype=np.float64)


def load(folder):
    """The capture: colour pictures (uint8, H x W x 3), poses (camera to robot, 4 x 4), the colour camera's K, and
    each picture's depth registered into it (metres, 0 for none)."""
    folder = Path(folder)
    scene = json.loads((folder / 'scene.json').read_text())
    if scene.get('schema') != 1 or scene.get('kind') != 'head_scene':
        raise ValueError(f'{folder} is not a head scene capture')
    ck, dk = scene['color_intrinsics'], scene['depth_intrinsics']
    color_k, depth_k = intrinsics(ck), intrinsics(dk)
    r, t = np.asarray(scene['to_color']['rotation'], float), np.asarray(scene['to_color']['translation'], float)
    scale = float(scene['depth_scale'])
    images, poses, depths = [], [], []
    for frame in scene['frames']:
        image = np.asarray(Image.open(io.BytesIO((folder / frame['color']).read_bytes())).convert('RGB'))
        if image.shape[:2] != (ck['height'], ck['width']):
            raise ValueError(f"{frame['color']} is {image.shape[1]}x{image.shape[0]}, not the intrinsics' size")
        raw = np.frombuffer(zlib.decompress((folder / frame['depth']).read_bytes()), '<u2')
        depth = raw.reshape(dk['height'], dk['width']).astype(np.float64) * scale
        images.append(image)
        poses.append(pose_matrix(frame['pose']))
        depths.append(register(depth, depth_k, color_k, r, t, ck['width'], ck['height']))
    return np.stack(images), np.stack(poses), color_k, np.stack(depths).astype(np.float32)


def register(depth, depth_k, color_k, r, t, width, height):
    """The depth seen from the colour camera: each reading moved into its frame and projected, the nearest kept."""
    v, u = np.nonzero((depth > DEPTH_RANGE_M[0]) & (depth < DEPTH_RANGE_M[1]))
    z = depth[v, u]
    points = np.stack([(u - depth_k[0, 2]) / depth_k[0, 0] * z, (v - depth_k[1, 2]) / depth_k[1, 1] * z, z], 1)
    points = points @ r.T + t
    keep = points[:, 2] > 0
    points = points[keep]
    cu = np.round(points[:, 0] / points[:, 2] * color_k[0, 0] + color_k[0, 2]).astype(int)
    cv = np.round(points[:, 1] / points[:, 2] * color_k[1, 1] + color_k[1, 2]).astype(int)
    inside = (cu >= 0) & (cu < width) & (cv >= 0) & (cv < height)
    out = np.full((height, width), np.inf)
    np.minimum.at(out, (cv[inside], cu[inside]), points[inside, 2])
    out[np.isinf(out)] = 0
    return out


def starting_points(images, poses, color_k, depths, voxel=VOXEL_M):
    """Gaussians' starting positions and colours: the registered depth of the training frames, back-projected into
    the robot frame, one per voxel."""
    points, colours = [], []
    for image, pose, depth in zip(images, poses, depths):
        v, u = np.nonzero(depth[::2, ::2])
        v, u = v * 2, u * 2
        z = depth[v, u]
        camera = np.stack([(u - color_k[0, 2]) / color_k[0, 0] * z, (v - color_k[1, 2]) / color_k[1, 1] * z, z], 1)
        points.append(camera @ pose[:3, :3].T + pose[:3, 3])
        colours.append(image[v, u] / 255.)
    points, colours = np.concatenate(points), np.concatenate(colours)
    cells = np.floor(points / voxel).astype(np.int64)
    _keys, first, inverse = np.unique(cells, axis=0, return_index=True, return_inverse=True)
    count = np.bincount(inverse.reshape(-1))
    merged = np.stack([np.bincount(inverse.reshape(-1), points[:, k]) for k in range(3)], 1) / count[:, None]
    tint = np.stack([np.bincount(inverse.reshape(-1), colours[:, k]) for k in range(3)], 1) / count[:, None]
    return merged.astype(np.float32), tint.astype(np.float32)


def ssim(a, b, window=11, sigma=1.5):
    """Structural similarity of two images (B, 3, H, W), Gaussian-windowed."""
    import torch
    import torch.nn.functional as F
    coords = torch.arange(window, dtype=a.dtype, device=a.device) - window // 2
    g = torch.exp(-coords ** 2 / (2 * sigma ** 2))
    g = g / g.sum()
    kernel = torch.outer(g, g).expand(3, 1, window, window).contiguous()
    blur = lambda x: F.conv2d(x, kernel, padding=window // 2, groups=3)  # noqa: E731
    mu_a, mu_b = blur(a), blur(b)
    var_a, var_b, cov = blur(a * a) - mu_a ** 2, blur(b * b) - mu_b ** 2, blur(a * b) - mu_a * mu_b
    c1, c2 = .01 ** 2, .03 ** 2
    return (((2 * mu_a * mu_b + c1) * (2 * cov + c2)) / ((mu_a ** 2 + mu_b ** 2 + c1) * (var_a + var_b + c2))).mean()


def write_ply(path, params, sh_degree):
    """The standard 3DGS PLY: x y z, normals (zero), f_dc, f_rest (channel-major), opacity (logit), scales (log),
    rotation (w x y z)."""
    import torch.nn.functional as F
    from plyfile import PlyData, PlyElement
    means = params['means'].detach().cpu().numpy()
    count = len(means)
    sh0 = params['sh0'].detach().cpu().numpy().reshape(count, 3)
    rest = params['shN'].detach().cpu().numpy()  # (N, K - 1, 3)
    rest = rest.transpose(0, 2, 1).reshape(count, -1)
    columns = [('x', means[:, 0]), ('y', means[:, 1]), ('z', means[:, 2])]
    columns += [(name, np.zeros(count, np.float32)) for name in ('nx', 'ny', 'nz')]
    columns += [(f'f_dc_{i}', sh0[:, i]) for i in range(3)]
    columns += [(f'f_rest_{i}', rest[:, i]) for i in range(rest.shape[1])]
    columns += [('opacity', params['opacities'].detach().cpu().numpy())]
    scales = params['scales'].detach().cpu().numpy()
    quats = F.normalize(params['quats'].detach(), dim=-1).cpu().numpy()
    columns += [(f'scale_{i}', scales[:, i]) for i in range(3)] + [(f'rot_{i}', quats[:, i]) for i in range(4)]
    data = np.empty(count, dtype=[(name, 'f4') for name, _ in columns])
    for name, value in columns:
        data[name] = value
    PlyData([PlyElement.describe(data, 'vertex')]).write(str(path))


def train(capture, out, steps=7000, sh_degree=3, device='cuda'):
    import torch
    import torch.nn.functional as F
    from gsplat import rasterization
    from gsplat.strategy import DefaultStrategy
    torch.manual_seed(0)
    started = time.time()
    images, poses, color_k, depths = load(capture)
    count, height, width = images.shape[:3]
    held = [i for i in range(count) if i % HOLD_OUT == HOLD_OUT - 1] if count >= HOLD_OUT else []
    trained = [i for i in range(count) if i not in held]
    points, tint = starting_points(images[trained], poses[trained], color_k, depths[trained])
    centre = poses[:, :3, 3].mean(0)
    scene_scale = float(np.percentile(np.linalg.norm(points - centre, axis=1), 90)) * 1.1
    print(f'{count} frames ({len(held)} held out), {width}x{height}; {len(points)} starting Gaussians; '
          f'scene radius {scene_scale:.2f} m', flush=True)

    sh_count = (sh_degree + 1) ** 2
    C0 = 0.28209479177387814
    params = torch.nn.ParameterDict({
        'means': torch.nn.Parameter(torch.tensor(points, device=device)),
        'scales': torch.nn.Parameter(torch.full((len(points), 3), math.log(VOXEL_M), device=device)),
        'quats': torch.nn.Parameter(torch.tensor([[1., 0, 0, 0]], device=device).repeat(len(points), 1)),
        'opacities': torch.nn.Parameter(torch.logit(torch.full((len(points),), .1, device=device))),
        'sh0': torch.nn.Parameter(((torch.tensor(tint, device=device) - .5) / C0)[:, None, :]),
        'shN': torch.nn.Parameter(torch.zeros((len(points), sh_count - 1, 3), device=device)),
    })
    rates = {'means': 1.6e-4 * scene_scale, 'scales': 5e-3, 'quats': 1e-3, 'opacities': 5e-2, 'sh0': 2.5e-3,
             'shN': 2.5e-3 / 20}
    optimizers = {name: torch.optim.Adam([{'params': params[name], 'lr': rate, 'name': name}], eps=1e-15)
                  for name, rate in rates.items()}
    decay = torch.optim.lr_scheduler.ExponentialLR(optimizers['means'], gamma=.01 ** (1 / steps))
    strategy = DefaultStrategy(verbose=False, refine_stop_iter=int(steps * .7), reset_every=max(3000, steps // 2))
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state(scene_scale=scene_scale)

    images_t = torch.tensor(images, dtype=torch.uint8)
    depths_t = torch.tensor(depths)
    viewmats = torch.tensor(np.linalg.inv(poses), dtype=torch.float32)
    K = torch.tensor(color_k, dtype=torch.float32, device=device)[None]

    def render(index, degree):
        colors = torch.cat([params['sh0'], params['shN']], 1)
        out, alphas, info = rasterization(
            params['means'], params['quats'], torch.exp(params['scales']), torch.sigmoid(params['opacities']),
            colors, viewmats[index][None].to(device), K, width, height, sh_degree=degree, render_mode='RGB+ED',
            packed=False)  # As the densification's step_post_backward(packed=False) reads it.
        return out[0, ..., :3], out[0, ..., 3], info

    for step in range(steps):
        index = trained[np.random.randint(len(trained))]
        truth = images_t[index].to(device).float() / 255.
        depth_truth = depths_t[index].to(device)
        rgb, depth, info = render(index, min(sh_degree, step // 1000))
        strategy.step_pre_backward(params, optimizers, state, step, info)
        loss = (1 - SSIM_WEIGHT) * F.l1_loss(rgb, truth) + SSIM_WEIGHT * (
            1 - ssim(rgb.permute(2, 0, 1)[None], truth.permute(2, 0, 1)[None]))
        seen = depth_truth > 0
        if seen.any():
            loss = loss + DEPTH_WEIGHT * (depth[seen] - depth_truth[seen]).abs().mean()
        loss.backward()
        for optimizer in optimizers.values():
            optimizer.step()
            optimizer.zero_grad(set_to_none=True)
        decay.step()
        strategy.step_post_backward(params, optimizers, state, step, info, packed=False)
        if step % 1000 == 0 or step == steps - 1:
            print(f'step {step}: loss {loss.item():.4f}, {len(params["means"])} Gaussians, '
                  f'{time.time() - started:.0f} s', flush=True)

    with torch.no_grad():
        scores = []
        for index in held:
            rgb, _depth, _info = render(index, sh_degree)
            truth = images_t[index].to(device).float() / 255.
            scores.append(float(-10 * torch.log10(F.mse_loss(rgb.clamp(0, 1), truth))))
    write_ply(out, params, sh_degree)
    report = {'frames': count, 'held_out': len(held), 'gaussians': len(params['means']), 'steps': steps,
              'psnr_held_out': float(np.mean(scores)) if scores else None, 'seconds': round(time.time() - started),
              'scene_radius_m': scene_scale}
    Path(out).with_suffix('.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(report), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('capture')
    parser.add_argument('out')
    parser.add_argument('--steps', type=int, default=7000)
    parser.add_argument('--sh', type=int, default=3)
    args = parser.parse_args()
    train(args.capture, args.out, steps=args.steps, sh_degree=args.sh)
