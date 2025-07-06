#!/usr/bin/env python3
"""
Bring up both Aizee arms with MoveIt Servo.
  • Feetech ros2_control driver
  • joint_state_broadcaster  + 2 trajectory controllers
  • 2 Servo nodes (left / right)
  • optional Joy-to-Servo game-pad tele-op
"""
import os, yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory as gpsd
from moveit_configs_utils import MoveItConfigsBuilder

# ---------- helper ----------------------------------------------------------
def _yaml(pkg, rel):
    with open(os.path.join(gpsd(pkg), rel), "r") as f:
        return yaml.safe_load(f)
# ---------------------------------------------------------------------------

def generate_launch_description():

    # 1) full MoveIt config -----------------------------------------------
    mc = (MoveItConfigsBuilder("aizee_model", package_name="moveit_aizee")
          .robot_description()                    # <builds URDF+Xacro, SRDF,…
          .robot_description_semantic()
          .robot_description_kinematics()
          .to_moveit_configs())

    # 2) Servo params for each arm ----------------------------------------
    left_servo_yaml  = _yaml("moveit_aizee", "config/servo_left.yaml")
    right_servo_yaml = _yaml("moveit_aizee", "config/servo_right.yaml")

    left_servo_params  = {"moveit_servo": left_servo_yaml}
    right_servo_params = {"moveit_servo": right_servo_yaml}

    # 3) ros2_control + controllers ---------------------------------------
    ros2_ctrl_yaml = os.path.join(gpsd("moveit_aizee"),
                                  "config", "ros2_controllers.yaml")

    ros2_control = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[mc.robot_description, ros2_ctrl_yaml],
        output="screen",
    )

    js_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    left_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_arm_controller",  "-c", "/controller_manager"],
    )
    right_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    # 4) Two standalone Servo nodes ---------------------------------------
    left_servo_node = Node(
        package="moveit_servo", executable="servo_node_main",
        namespace="left_arm",
        parameters=[left_servo_params,
                    mc.robot_description,
                    mc.robot_description_semantic,
                    mc.robot_description_kinematics],
        output="screen",
    )
    right_servo_node = Node(
        package="moveit_servo", executable="servo_node_main",
        namespace="right_arm",
        parameters=[right_servo_params,
                    mc.robot_description,
                    mc.robot_description_semantic,
                    mc.robot_description_kinematics],
        output="screen",
    )

    # 5) Optional game-pad container (delete if Unity drives Servo) --------
    joy_container = ComposableNodeContainer(
        name="joy_to_servo_container",
        namespace="joy_space",
        package="rclcpp_components", executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(                         # state publisher
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[mc.robot_description]),
            ComposableNode(                         # static tf world→base
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                parameters=[{"child_frame_id": "base_link",
                             "frame_id": "world"}]),
            ComposableNode(                         # raw joystick
                package="joy", plugin="joy::Joy", name="joy_node"),
            # convert Joy → TwistStamped / JointJog
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="joy_to_servo"),
        ],
        output="screen",
    )

    # 6) RViz (loads MoveIt panels & Servo markers) ------------------------
    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", os.path.join(gpsd("moveit_aizee"),
                                      "config", "moveit.rviz")],
        parameters=[mc.robot_description,
                    mc.robot_description_semantic],
    )
    
    # 7) Dummy Joint Publisher (for testing)
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

    return LaunchDescription([
        ros2_control, js_broadcaster, left_ctrl, right_ctrl,
        left_servo_node, right_servo_node, dummy_js_pub,
        joy_container, rviz
    ])
