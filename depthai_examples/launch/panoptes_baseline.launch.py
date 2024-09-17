import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    depthai_examples_path = get_package_share_directory('depthai_examples')
    
    urdf_launch_dir = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch"
    )

    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    camera_model = LaunchConfiguration('camera_model', default='OAK-D').perform(context)
    parent_frame = LaunchConfiguration('parent_frame', default='oak-d-base-frame').perform(context)
    camera_param_uri = LaunchConfiguration('camera_param_uri', default='package://depthai_examples/params/camera').perform(context)
    sync_nn = LaunchConfiguration('sync_nn', default='true').perform(context)
    subpixel = LaunchConfiguration('subpixel', default='true').perform(context)
    nnName = LaunchConfiguration('nnName', default='x').perform(context)
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default=os.path.join(depthai_examples_path, 'resources')).perform(context)
    fullFrameTracking = LaunchConfiguration('fullFrameTracking', default='false').perform(context)
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    name = LaunchConfiguration("name").perform(context)
    colorResolution = LaunchConfiguration("colorResolution").perform(context)
    monoResolution = LaunchConfiguration("monoResolution").perform(context)

    color_sens_name = 'rgb'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, "urdf_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "tf_prefix": name,
                "camera_model": camera_model,
                "base_frame": name,
                "parent_frame": parent_frame,
                "cam_pos_x": cam_pos_x,
                "cam_pos_y": cam_pos_y,
                "cam_pos_z": cam_pos_z,
                "cam_roll": cam_roll,
                "cam_pitch": cam_pitch,
                "cam_yaw": cam_yaw,
                "use_composition": 'false'
            }.items(),
        ),
        Node(
            package='depthai_examples',
            executable='tracker_yolov4_spatial_node',
            output='screen',
            name = 'tracker_yolov4_spatial_node',
            namespace = namespace,
            parameters=[{
                'namespace' : namespace,
                'tf_prefix': name,
                'camera_param_uri': camera_param_uri,
                'sync_nn': bool(sync_nn),
                'nnName': nnName,
                'resourceBaseFolder': resourceBaseFolder,
                'fullFrameTracking': bool(fullFrameTracking),
                'colorResolution': colorResolution,
                'monoResolution': monoResolution,
            }],
        ),
        Node(
            package='image_proc',
            executable="rectify_node",
            name="rectify_color_node",
            namespace=namespace,
            remappings=[
                ('image', f'{name}/{color_sens_name}/image_raw'),
                ('camera_info', f'{name}/{color_sens_name}/camera_info'),
                ('image_rect', f'{name}/{color_sens_name}/image_rect')
            ]
        )
    ]


def generate_launch_description():
    depthai_examples_path = get_package_share_directory('depthai_examples')

    declared_arguments = [
        DeclareLaunchArgument('name', default_value='oak'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE'),
        DeclareLaunchArgument('parent_frame', default_value='oak-d-base-frame'),
        DeclareLaunchArgument('cam_pos_x', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_param_uri', default_value='package://depthai_examples/params/camera'),
        DeclareLaunchArgument('sync_nn', default_value='true'),
        DeclareLaunchArgument('subpixel', default_value='true'),
        DeclareLaunchArgument('nnName', default_value='x'),
        DeclareLaunchArgument('resourceBaseFolder', default_value=os.path.join(depthai_examples_path, 'resources')),
        DeclareLaunchArgument('fullFrameTracking', default_value='false'),
        DeclareLaunchArgument('colorResolution', default_value='1080p'),
        DeclareLaunchArgument('monoResolution', default_value='400p')
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )