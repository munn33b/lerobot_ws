import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('lerobot_description'),
            'config',
            'so101_controllers.yaml',
        ]
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': False}],
        arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120", "--switch-timeout", "100"],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller', "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120", "--switch-timeout", "100",
            '--param-file',
            robot_controllers,
            ],
    )

    gripper_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller', "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120", "--switch-timeout", "100",
            '--param-file',
            robot_controllers,
            ],
    )

    return launch.LaunchDescription([
        joint_state_broadcaster_spawner,
        gripper_trajectory_controller_spawner,
        joint_trajectory_controller_spawner
    ])
