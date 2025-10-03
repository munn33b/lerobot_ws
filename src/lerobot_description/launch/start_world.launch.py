import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():

    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    package_name_gazebo = 'lerobot_description'
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    default_robot_name = 'SO101'
    pkg_name = FindPackageShare(package="lerobot_description").find("lerobot_description")

    robot_name = LaunchConfiguration('robot_name')

    default_ros_gz_bridge_config_file_path = os.path.join(pkg_name, ros_gz_bridge_config_file_path)


    bras_robot_description_path = FindPackageShare("lerobot_description").find("lerobot_description")

    # Define the models directory inside the package
    models_path = os.path.join(bras_robot_description_path, "models")
    workspace_src_path = os.path.abspath(os.path.join(bras_robot_description_path, "../../../lerobot_description/share"))


    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{models_path}:{workspace_src_path}"
    print (os.environ["GZ_SIM_RESOURCE_PATH"])


    world_file = os.path.join(
        get_package_share_directory('lerobot_description'),
        'worlds',
        'tables_and_cubes_overhead_cam.sdf'
    )



    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="camera_node",
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )]),
        launch_arguments={
                'gz_args': f' -r -v 4 {world_file} '
        }.items()
    )
    gz_ros2_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )


  # Launch the arm controller after launching the joint state broadcaster




    return launch.LaunchDescription([
        ign_gazebo,
        bridge,
        gz_ros2_clock_bridge,
        start_gazebo_ros_bridge_cmd
    ])
