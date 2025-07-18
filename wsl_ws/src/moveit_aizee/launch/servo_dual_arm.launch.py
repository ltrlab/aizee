#!/usr/bin/env python3
"""
dual_arm_servo.launch.py
Bring up both Aizee arms with MoveIt Servo.

• left_arm  (ttyUSB0)  – ros2_control_node  +  JT-controller
• right_arm (ttyUSB1)  – ros2_control_node  +  JT-controller
• joint_correction      – merges & offsets the two joint_state streams
• two independent Servo nodes
• optional Xbox-controller tele-op
• RViz with MoveIt panels
"""
import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory as gpsd
from moveit_configs_utils import MoveItConfigsBuilder

PKG_SHARE = gpsd("moveit_aizee")
CFG = lambda *p: os.path.join(PKG_SHARE, "config", *p)

# ---------------------------------------------------------------------------

def _read_yaml(rel_path):
    with open(CFG(rel_path), "r") as f:
        return yaml.safe_load(f)

# ---------------------------------------------------------------------------

def generate_launch_description():

    # ─────────────────────── common MoveIt description ────────────────────
    mc = (MoveItConfigsBuilder("aizee_model", package_name="moveit_aizee")
          .robot_description()
          .robot_description_semantic()
          .robot_description_kinematics()
          .to_moveit_configs())

    # ─────────────────────── Servo parameters (per arm) ───────────────────
    left_servo_params  = {"moveit_servo": _read_yaml("servo_left.yaml")}
    right_servo_params = {"moveit_servo": _read_yaml("servo_right.yaml")}

    # ────────────────────── ros2_control: two managers ────────────────────
        # ──────────────────── ros2_control (single) ─────────────────────
    ctrl_mgr = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[mc.robot_description,
                    CFG("ros2_controllers.yaml")],     # ← new file above
        output="screen")

    spawner_js   = Node(package="controller_manager", executable="spawner",
                        arguments=["joint_state_broadcaster",
                                "--controller-manager", "/controller_manager"])
    spawner_left = Node(package="controller_manager", executable="spawner",
                        arguments=["left_arm_controller",
                                "--controller-manager", "/controller_manager"])
    spawner_right= Node(package="controller_manager", executable="spawner",
                        arguments=["right_arm_controller",
                                "--controller-manager", "/controller_manager"])

    # ─────────────────────── joint-state merger / offset ──────────────────
    joint_merger = Node(
        package="aizee_jetson_core",
        executable="joint_correction",     # name you give the new script
        output="screen")

    # ─────────────────────────── Servo nodes ──────────────────────────────
    left_servo = Node(package="moveit_servo", executable="servo_node_main",
                      namespace="left_arm",
                      parameters=[left_servo_params,
                                  mc.robot_description,
                                  mc.robot_description_semantic,
                                  mc.robot_description_kinematics],
                      output="screen")

    right_servo = Node(package="moveit_servo", executable="servo_node_main",
                       namespace="right_arm",
                       parameters=[right_servo_params,
                                   mc.robot_description,
                                   mc.robot_description_semantic,
                                   mc.robot_description_kinematics],
                       output="screen")

    # ────────────────────────── MoveGroup nodes ───────────────────────────
    move_group_overrides = {
        "move_group": {
            "planning_pipelines": "ompl,chomp",
            "default_planning_pipeline": "ompl",
        }
    }

    global_mg = Node(package="moveit_ros_move_group", executable="move_group",
                     output="screen",
                     parameters=[mc.to_dict(), move_group_overrides])

    # ──────────────────── optional Joy-to-Servo tele-op ───────────────────
    joy_container = ComposableNodeContainer(
        name="joy_to_servo_container",
        namespace="joy_space",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(package="robot_state_publisher",
                           plugin="robot_state_publisher::RobotStatePublisher",
                           parameters=[mc.robot_description]),
            ComposableNode(package="tf2_ros",
                           plugin="tf2_ros::StaticTransformBroadcasterNode",
                           parameters=[{"child_frame_id": "base_link",
                                        "frame_id": "world"}]),
            ComposableNode(package="joy",
                           plugin="joy::Joy",
                           name="joy_node"),
            ComposableNode(package="moveit_servo",
                           plugin="moveit_servo::JoyToServoPub",
                           name="joy_to_servo"),
        ],
        output="screen",
    )

    joy_bridge = Node(package="aizee_teleop",
                      executable="joy_dual_arm_node",
                      name="joy_dual_arm_node",
                      output="screen")

    # ───────────────────────────── RViz ───────────────────────────────────
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", CFG("moveit.rviz")],
                parameters=[mc.robot_description,
                            mc.robot_description_semantic,
                            mc.robot_description_kinematics],
                output="log")

    # (Optional) end-effector interactive marker --------------------------
    ee_marker = Node(package="aizee_jetson_core",
                     executable="arm_interactive_marker",
                     name="ee_interactive_marker",
                     output="screen",
                     parameters=[{"base_frame": "gantry_base_link",
                                  "ee_frame":   "right_wrist_tool_link"}])

    # ───────────────────── assemble launch description ────────────────────
    return LaunchDescription([
        # controllers
        ctrl_mgr,
        spawner_js, #
        spawner_left, spawner_right,
        # merger
        #joint_merger,
        # servo
        left_servo, right_servo,
        # planning
        global_mg,
        # tele-op / viz
        joy_container, joy_bridge,
        rviz, ee_marker,
    ])
