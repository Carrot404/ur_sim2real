'''
Description: load  with controllers
version: 1.0.0
Author: Songjie Xiao
Email: songjiexiao@zju.edu.cn
Date: 2023-09-17 16:42:12
LastEditTime: 2023-11-22 20:18:05
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):

    # example of using launch_configurations
    # type: string
    # ur_type = context.launch_configurations["ur_type"]
    # ur_type = LaunchConfiguration("ur_type").perform(context)
    # not string type, some struct type
    # ur_type = LaunchConfiguration("ur_type")

    ur_type = context.launch_configurations["ur_type"]
    tf_prefix = context.launch_configurations["tf_prefix"]
    robot_ip = context.launch_configurations["robot_ip"]
    use_fake_hardware = context.launch_configurations["use_fake_hardware"]
    initial_joint_controller = context.launch_configurations["initial_joint_controller"]
    activate_joint_controller = context.launch_configurations["activate_joint_controller"]
    launch_rviz = context.launch_configurations["launch_rviz"]

    ur_subs = dict(
        name="ur3_probe",
        ur_type=ur_type,
        tf_prefix=tf_prefix,
        robot_ip=robot_ip,
        use_fake_hardware=use_fake_hardware,
    )
    robot_description = ParameterBuilder("ur_probe_description").xacro_parameter(
        parameter_name="robot_description",
        file_path="urdf/ur_probe.urdf.xacro",
        mappings=ur_subs
    ).to_dict()

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("ur_probe_bringup"), "config",
         "ur_probe_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_probe_description"), "rviz", "view_robot.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=UnlessCondition(use_fake_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": "false"},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags,
        )

    controller_spawner_names = [
        "joint_state_broadcaster",
        "joint_to_cartesian_controller",
        # "io_and_status_controller",
        # "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "admittance_controller",
        # "joint_trajectory_controller",
    ]
    # controller_spawner_inactive_names = ["joint_trajectory_controller"]

    controller_spawners = [controller_spawner(
        name) for name in controller_spawner_names] 
    # + [
    #     controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    # ]

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller),
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        ur_control_node,
        dashboard_client_node,
        # controller_stopper_node,
        # initial_joint_controller_spawner_started,
        # initial_joint_controller_spawner_stopped,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="ursim.",
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
