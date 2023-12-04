#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    DeclareLaunchArgument,
)

import os
import subprocess

lslidar_filter_params_file = LaunchConfiguration("lslidar_filter_params_file")

def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_c32.yaml')
    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    ros_version = p.communicate()[0]
    print(ros_version)
    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    node_namespace='c32',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    output='screen',
                                    parameters=[driver_dir],
                                    )
    elif ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n' or ros_version == b'iron\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    namespace='c32',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[driver_dir],
                                    )
    else:
        print("Please configure the ros environment")
        exit()

    lslidar_crop_box_container = ComposableNodeContainer(
                    name='lslidar_crop_box',
                    namespace='c32',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package="pcl_ros",
                            plugin="pcl_ros::CropBox",
                            name="pcl_box_lslidar_filter",
                            remappings=[
                                ("input", "/lslidar_point_cloud"),
                                ("output", "/lslidar_point_cloud/filtered"),
                            ],
                            parameters=[lslidar_filter_params_file],
                        ),
                    ],
                )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lslidar_filter_params_file",
            default_value=os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "lslidar_filter.yaml",
            ),
            description="Full path to the ROS2 parameters file to use Lslidar filter",
        ),
        driver_node,
        lslidar_crop_box_container,
    ])