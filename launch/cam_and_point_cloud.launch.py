import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera_node',
            output='screen',
            remappings=[('depth',             '/camera/depth/image'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('image',             '/camera/rgb/image')]
        ),

        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/camera/depth/image'),
                                ('camera_info', '/camera/depth/camera_info')
                                # ('image', '/camera/depth/converted_image')
                                ]
                ),
            ],
            output='screen',
        ),
    ])
