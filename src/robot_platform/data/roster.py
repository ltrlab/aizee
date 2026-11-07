"""The axes a Minerva episode records (task capture format 9), with no imports: the control loop's process reads it.

Both arms and grippers on CAN, in receiver motor radians, then the neck's two Feetech servos, in raw servo radians:
head_pan drives the CAD joint neck_z, head_tilt drives neck_y. The order within an episode is its `minerva_joints`.
"""

ARM_AXES = tuple(f'{side}_arm_j{j}' for side in ('left', 'right') for j in range(1, 7)) + (
    'left_gripper',
    'right_gripper',
)
HEAD_AXES = ('head_pan', 'head_tilt')
HEAD_JOINTS = ('neck_z', 'neck_y')  # The CAD joints HEAD_AXES drive, in the same order.
AXES = ARM_AXES + HEAD_AXES
# The head camera's optical pose in the robot frame: [x, y, z, qx, qy, qz, qw] (neck.head_pose.pose7).
HEAD_POSE_WIDTH = 7
FORMAT_VERSION = 9
