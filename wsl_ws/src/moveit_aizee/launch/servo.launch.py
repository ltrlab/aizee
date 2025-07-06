from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,         
)

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory  
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_spawn_controllers_launch,
    generate_move_group_launch,
)

def generate_launch_description():

    # Allow CLI override, e.g. ros2_control_hardware_type:=mock_components
    hw_arg = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="feetech",
        description="Hardware plugin name or 'mock_components'",
    )
    hw_type = LaunchConfiguration("ros2_control_hardware_type")

    # Build the parameter set we need (URDF, SRDF, kinematics, etc.)
    moveit_config = (
        MoveItConfigsBuilder("aizee_model", package_name="moveit_aizee")
        .robot_description(mappings={"ros2_control_hardware_type": hw_type})
        .robot_description_semantic()
        .robot_description_kinematics()
        .trajectory_execution()
        .to_moveit_configs()
    )

    # ---- Core launch files supplied by moveit_configs_utils ----
    rsp_ld = generate_rsp_launch(moveit_config)
    static_tf_ld = generate_static_virtual_joint_tfs_launch(moveit_config)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    controllers_ld = generate_spawn_controllers_launch(moveit_config)

    pkg_share = get_package_share_directory("moveit_aizee")
    left_yaml = str(moveit_config.package_path / "config/servo_left.yaml")
    right_yaml = str(moveit_config.package_path / "config/servo_right.yaml")

    left_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_arm",
        #namespace="left_arm",
        parameters=[
            left_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    right_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_arm",
        #namespace="right_arm",
        parameters=[
            right_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    rviz_cfg = str(moveit_config.package_path / "config/moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="log",
    )



    dummy_joint_names = [
        "bl_wheel_joint", "br_wheel_joint", "fl_wheel_joint", "fr_wheel_joint",
        "gantry_base_joint", "gantry_swivel_joint",
        "head_swivel_joint", "head_tilt_joint", "head_pitch_joint"
    ]

    dummy_js_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="dummy_joint_state_publisher",
        parameters=[{
            "rate": 30.0,
            "source_list": dummy_joint_names,
        }],
        output="log",
    )  
       
    delayed_rviz = TimerAction(
	    period=5.0,                 # seconds; adjust if Jetson is slower
	    actions=[rviz],
	)

    move_group_ld = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        hw_arg,
        ros2_control_node,
        dummy_js_pub,
        rsp_ld,
        static_tf_ld,
        controllers_ld,
        move_group_ld,
        delayed_rviz,
    ])

