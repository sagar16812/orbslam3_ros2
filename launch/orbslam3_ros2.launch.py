from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    
    slam_pkg_path = get_package_share_directory("orbslam3_ros2")

    vocab_file = os.path.join(slam_pkg_path, "config", "ORBvoc.txt")
    settings_file = os.path.join(slam_pkg_path, "config", "camera_and_slam_settings.yaml")

    print(f"Path of vocab file: {vocab_file}")
    print(f"Path of settings file: {settings_file}")

    # Declare an argument to enable or disable bag recording
    record_bag_arg = DeclareLaunchArgument(
        "record_bag", default_value="false", description="Enable or disable ros2 bag recording"
    )

    # SLAM Node
    slam_node = Node(
        package='orbslam3_ros2',
        executable='orb_slam3',
        output='screen',
        parameters=[
            {"vocab_path": vocab_file},
            {"config_path": settings_file}
        ]
    )

    # ros2 bag record command (only starts if enabled)
    bag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],  # '-a' records all topics
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output='screen'
    )

    return LaunchDescription([
        record_bag_arg,
        slam_node,
        bag_record_process
    ])
