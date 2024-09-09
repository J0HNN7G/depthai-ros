import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Path to the original launch file
    camera_launch_file = os.path.join(
        get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')

    # Include the original launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    # GroupAction to apply the namespace globally
    camera_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('robomaster_20'),  # Add your desired namespace here
            camera_launch  # Include the original launch file with the namespace
        ]
    )

    return LaunchDescription([
        camera_with_namespace
    ])
