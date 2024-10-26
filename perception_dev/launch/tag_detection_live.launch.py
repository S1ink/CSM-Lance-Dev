import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def make_camera_driver(subdir, is_ip = False):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('camera_utils'), 'launch', 'camera_driver.launch.py')),
        launch_arguments = {
            'use_ip_driver': str(is_ip),
            'config_subdir': subdir
        }.items()
    )

def generate_launch_description():

    cameras = [
        ('arducam1', False)
    ]
    cam_nodes = [
        make_camera_driver(x[0], x[1])
        for x in cameras
    ]
    cam_nodes.append(Node(
        name = 'cardinal_perception_tag_detection',
        package = 'cardinal_perception',
        executable = 'tag_detection_node',
        output = 'screen',
        parameters = [
            os.path.join(get_package_share_directory('perception_dev'), 'config', 'tag_detection_live.yaml')
            # TODO setup topics based on the array defined here
        ],
        remappings = [ ('tags_detections', '/cardinal_perception/tags_detections') ]
    ))

    return LaunchDescription(cam_nodes)
