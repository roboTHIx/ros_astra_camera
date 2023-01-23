

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription #, DeclareLaunchArgument#, ExecuteProcess, TimerAction
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def create_cam(cam_name, cam_serial, base_link, x = "0.0", y = "0.0", z = "0.0", roll = "0.0", pitch = "0.0", yaw = "0.0"):
  
  cam_link = cam_name + '_link'
  cam = IncludeLaunchDescription(
     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('astra_camera'), 'launch/cam_and_point_cloud.launch.py')),
     launch_arguments={
      'container_ns': cam_name,
      'camera_name':  cam_name,
      'camera_serial' : cam_serial,
      }.items(),
   )
  #tf to cam_link
  static_to_cam_link = Node(package='tf2_ros',
                            namespace=cam_name,
                            executable='static_transform_publisher',
                            name='static_transform_cam_link',
                            arguments = [x, y, z, roll, pitch, yaw, base_link, cam_link]  # '0 0 0 0 0 0 map odom'
                           )

  return LaunchDescription([cam, static_to_cam_link])



def generate_launch_description():

  static_cam_holder_base_r = Node(package='tf2_ros',
                             namespace='cam_holder_base_right',
                             executable='static_transform_publisher',
                             name='static_transform_cam_holder_base_right',
                             arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "cam_holder_base_right"]  # '0 0 0 0 0 0 map odom'
                            )

  cam_karl = create_cam("karl", "'18111230311'", "cam_holder_base_right", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0")

  cam_hilde = create_cam("hilde", "ADA4B1303B2", "cam_holder_base_right", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0")


  return LaunchDescription([cam_karl, cam_hilde, static_cam_holder_base_r])

