import math

import numpy as np
import pytest

pytest.importorskip("fcl")
trimesh = pytest.importorskip("trimesh")

from robot_platform.control.collision_guard import JointBoxBound, check_joint_box, motion_bounds
from robot_platform.models import ModelProfile
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.runtime_model import RuntimeCollisionModel, build_runtime_model


def runtime_from(tmp_path, urdf):
    path = tmp_path / "robot.urdf"
    path.write_text(urdf)
    profile = ModelProfile.from_urdf(path)
    artifact = tmp_path / "collision.json"
    build_runtime_model(profile, artifact)
    return RuntimeCollisionModel(profile, artifact)


def bound_of(runtime):
    return JointBoxBound(runtime.scene, runtime.keepout_motion_corners)


def test_one_turn_moves_a_part_its_distance_from_the_axis_wherever_its_mesh_frame_sits(tmp_path):
    # The part sits 1 m out along x, but its mesh is written 5 m out and placed back by its collision origin, as
    # CAD exports leave meshes at the assembly origin. Only where the part is counts, not where its frame is.
    trimesh.creation.box(extents=[.2, .2, .2]).apply_translation([5, 0, 0]).export(tmp_path / "part.stl")
    runtime = runtime_from(tmp_path, '''<robot name="r"><link name="root"/>
      <link name="arm"><collision><origin xyz="-4 0 0"/><geometry><mesh filename="part.stl"/></geometry>
      </collision></link>
      <joint name="turn" type="continuous"><parent link="root"/><child link="arm"/><axis xyz="0 0 1"/></joint>
      </robot>''')
    bound = bound_of(runtime)
    local = runtime.scene._shapes[0].local  # The hull (padded a little), where the link frame has it.
    hull = np.asarray(runtime.keepout_motion_corners[0]) @ local[:3, :3].T + local[:3, 3]
    far = np.hypot(hull[:, 0], hull[:, 1]).max()
    assert far == pytest.approx(math.hypot(1.1, .1), abs=.01)
    assert bound.travel({"turn": .3}, {"turn": .05})[0] == pytest.approx(far * .05)
    # The reach motion_bounds charges: the mesh's radius about its own frame plus the origin, about 9 m.
    reach = motion_bounds(runtime.scene, {"turn": .25}, {"turn": .35}) / .1
    assert reach[0] > 8 * far


def test_a_turn_about_the_part_s_own_axis_moves_it_only_its_radius(tmp_path):
    # A forearm twist: the hand sits on the twist axis, half a metre beyond the joint.
    runtime = runtime_from(tmp_path, '''<robot name="r"><link name="root"/><link name="forearm"/>
      <link name="hand"><collision><origin xyz=".5 0 0"/><geometry><box size=".1 .04 .04"/></geometry>
      </collision></link>
      <joint name="twist" type="revolute"><parent link="root"/><child link="forearm"/><axis xyz="1 0 0"/>
        <limit lower="-3" upper="3"/></joint>
      <joint name="wrist" type="fixed"><parent link="forearm"/><child link="hand"/></joint></robot>''')
    travel = bound_of(runtime).travel({"twist": 0.}, {"twist": .1})[0]
    radius = max(np.hypot(y, z) for _, y, z in runtime.keepout_motion_corners[0])
    assert travel == pytest.approx(radius * .1)
    assert travel < .01 < motion_bounds(runtime.scene, {"twist": -.1}, {"twist": .1})[0] / 2


TWO_ARMS = '''<robot name="r"><link name="root"/>
  {arms}
</robot>'''
ARM = '''<link name="{side}_shoulder"/><link name="{side}_upper"/>
  <link name="{side}_hand"><collision><origin xyz=".3 0 0" rpy=".3 .2 .1"/><geometry><box size=".1 .06 .04"/>
  </geometry></collision></link>
  <link name="{side}_body"><collision><origin xyz=".1 0 0"/><geometry><box size=".2 .05 .05"/></geometry>
  </collision></link>
  <joint name="{side}_j1" type="revolute"><parent link="root"/><child link="{side}_shoulder"/>
    <origin xyz="0 {y} 1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="{side}_j2" type="revolute"><parent link="{side}_shoulder"/><child link="{side}_upper"/>
    <origin xyz=".05 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3"/></joint>
  <joint name="{side}_body_mount" type="fixed"><parent link="{side}_upper"/><child link="{side}_body"/></joint>
  <joint name="{side}_slide" type="prismatic"><parent link="{side}_upper"/><child link="{side}_hand"/>
    <origin xyz=".2 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper=".2"/></joint>'''


def two_arms(tmp_path, gap=.6):
    arms = ARM.format(side="left", y=gap / 2) + ARM.format(side="right", y=-gap / 2)
    return runtime_from(tmp_path, TWO_ARMS.format(arms=arms))


def test_bound_covers_every_sampled_pose_and_shares_the_scene_s_kinematics(tmp_path):
    runtime = two_arms(tmp_path)
    scene, bound = runtime.scene, bound_of(runtime)
    corners = [np.asarray(c) for c in runtime.keepout_motion_corners]
    rng = np.random.default_rng(7)
    worst = 0.
    for _ in range(30):
        center = {name: float(rng.uniform(-1.5, 1.5)) for name in scene.independent_joints}
        center.update(left_slide=float(rng.uniform(.05, .15)), right_slide=float(rng.uniform(.05, .15)))
        half = {name: float(rng.uniform(0, .3)) for name in scene.independent_joints}
        half.update(left_slide=.04, right_slide=.04)
        frames = bound.frames(center)
        start = runtime.transforms(center)
        assert np.allclose(bound.shape_transforms(frames), start)
        travel = bound.travel(center, half, frames)
        for _ in range(20):
            pose = {name: center[name] + float(rng.choice([-1, 1, rng.uniform(-1, 1)])) * half[name]
                    for name in center}
            moved = runtime.transforms(pose)
            for s, points in enumerate(corners):
                a = points @ start[s][:3, :3].T + start[s][:3, 3]
                b = points @ moved[s][:3, :3].T + moved[s][:3, 3]
                worst = max(worst, np.linalg.norm(b - a, axis=1).max() / max(travel[s], 1e-12))
    assert .5 < worst <= 1 + 1e-9


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_box_check_clears_what_cannot_reach_and_names_what_could(tmp_path):
    runtime = two_arms(tmp_path, gap=.6)
    backend = NativeCollisionBackend(runtime, geometry_mode="keepout")
    bound = bound_of(runtime)
    # The hands point at each other along y, 0.6 m apart at the shoulders.
    center = dict(left_j1=-math.pi / 2, left_j2=0., left_slide=.1, right_j1=math.pi / 2, right_j2=0., right_slide=.1)
    gap = backend.proximity(center, threshold=1., scope="inter_branch")["minimum_distance_m"]
    small = {name: .01 for name in center}
    clear = check_joint_box(backend, bound, center, small, scope="inter_branch")
    assert clear["clear"] and clear["queries"] == 1 and clear["reason"] == "clear_joint_box"
    assert 0 < clear["clearance_lower_bound_m"] < gap
    # Wide enough that the bound could bring the hands together: blocked, with the pair that could touch.
    wide = dict(small, left_j1=.5, right_j1=.5)
    blocked = check_joint_box(backend, bound, center, wide, scope="inter_branch")
    assert not blocked["clear"] and blocked["reason"] == "insufficient_swept_clearance"
    assert blocked["shapes"] and blocked["clearance_lower_bound_m"] <= .005
    late = check_joint_box(backend, bound, center, small, scope="inter_branch", deadline=0., clock=lambda: 1.)
    assert not late["clear"] and late["reason"] == "collision_deadline"
