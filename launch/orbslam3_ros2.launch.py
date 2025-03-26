from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def launch_rviz2(context):
    visualize = context.launch_configurations.get('visualize', 'false')
    if visualize.lower() == 'true':
        # Get the package share directory and construct the path to the RViz config file
        package_share_directory = get_package_share_directory('orbslam3_ros2')
        rviz_config_path = os.path.join(package_share_directory, 'config', 'orbslam3_ros2.rviz')

        return [ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )]
    return []

def generate_launch_description():
    
    slam_pkg_path = get_package_share_directory("orbslam3_ros2")

    vocab_file = os.path.join(slam_pkg_path, "config", "ORBvoc.txt")
    # settings_file = os.path.join(slam_pkg_path, "config", "camera_and_slam_settings.yaml")
    settings_file = os.path.join(slam_pkg_path, "config", "TUM_RGB-D_Dataset.yaml")

    print(f"Path of vocab file: {vocab_file}")
    print(f"Path of settings file: {settings_file}")

    # Declare an argument to enable or disable bag recording
    record_bag_arg = DeclareLaunchArgument(
        "record_bag", default_value="false", description="Enable or disable ros2 bag recording")
    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='false', description='Launch RViz2 with saved configuration')
    start_octomap = DeclareLaunchArgument(
        'start_octomap', default_value='false', description='Start Octomap server')
    camera_type_arg = DeclareLaunchArgument(
        'camera_type', default_value='mono', description='Camera type: mono, rgbd, stereo')


    # SLAM Node
    slam_node = Node(
        package='orbslam3_ros2',
        executable=LaunchConfiguration('camera_type'),  # Get the executable based on camera type
        output='screen',
        parameters=[
            {"vocab_path": vocab_file},
            {"config_path": settings_file},
        ]
    )

    # ros2 bag record command (only starts if enabled)
    bag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],  # '-a' records all topics
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output='screen'
    )

    # Octomap Server Node (only launch if start_octomap is true)
    octomap_server_node = ExecuteProcess(
        cmd=['ros2', 'run', 'octomap_server', 'octomap_server_node', '--ros-args', '--remap', 'cloud_in:=/slam/pointcloud'],
        condition=IfCondition(LaunchConfiguration("start_octomap")),
        # condition=lambda context: context.launch_configurations['start_octomap'] == 'true'
    )

    return LaunchDescription([
        record_bag_arg,
        start_octomap,
        visualize_arg,
        camera_type_arg,
        slam_node,
        bag_record_process,
        OpaqueFunction(function=launch_rviz2),
        octomap_server_node
    ])
