import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import DeclareLaunchArgument#, ExecuteProcess, TimerAction
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  # launch param
  container_ns = LaunchConfiguration('container_ns')
  camera_name = LaunchConfiguration('camera_name')
  camera_serial = LaunchConfiguration('camera_serial')
  
  container_ns_arg = DeclareLaunchArgument(
        'container_ns',
        default_value=''
    )

  camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='astra_mini'
    )

  camera_serial_arg = DeclareLaunchArgument(
        'camera_serial',
        default_value=''
    )

  cam_node = Node(package='astra_camera',
                  executable='astra_camera_node',
                  name='astra_camera_node',
                  namespace=camera_name,
                  output='screen',
              
                  parameters=[{
                    "serial" : camera_serial,
                    "auto_exposure" : True,
                    "auto_white_balance" : True,
                    "depth_frame_id" : (camera_name, TextSubstitution(text='_depth')),
                    "color_frame_id" : (camera_name, TextSubstitution(text='_color')),
                  }],
                  remappings=[('depth',             'depth/image'),
                              ('depth_camera_info', 'depth/camera_info'),
                              ('image',             'rgb/image'),
                              ('ir_image',          'ir/image'),
                              ]
                 )

  # launch plugin through rclcpp_components container
  pc_node = ComposableNodeContainer(name='container',
                                    package='rclcpp_components',
                                    namespace=container_ns,
                                    executable='component_container',
                                    composable_node_descriptions=[
                                        # Driver itself
                                        ComposableNode(
                                            package='depth_image_proc',
                                            plugin='depth_image_proc::PointCloudXyzNode',
                                            name='point_cloud_xyz_node',
                                            namespace=camera_name,
                                            remappings=[('image_rect',  'depth/image'),
                                                        ('camera_info', 'depth/camera_info'),
                                                        ('points',      'depth/points')
                                                        # ('image', '/camera/depth/converted_image')
                                                        ]
                                        ),
                                    ],
                                    output='screen',
  )

  #todo tf
  # static_base_link = Node(package='tf2_ros',
  #                          executable='static_transform_publisher',
  #                          name='static_transform_base_footprint_base_link',
  #                          arguments = ["0", "0", "0.2055", "0", "0", "0", "base_footprint", "base_link"]  # '0 0 0 0 0 0 map odom'
  #                         #  arguments=''
  #                         )

  #tf -> camera_name_link -> camera_depth, camera_color
  static_cam_link_to_depth = Node(package='tf2_ros',
                                  namespace=camera_name,
                                  executable='static_transform_publisher',
                                  name='static_transform_cam_link_depth',
                                  arguments = ["0", "0", "0", "0", "0", "0", (camera_name, TextSubstitution(text='_link')), (camera_name, TextSubstitution(text='_depth'))]  # '0 0 0 0 0 0 map odom'
                                 )
  static_cam_link_to_color = Node(package='tf2_ros',
                                  namespace=camera_name,
                                  executable='static_transform_publisher',
                                  name='static_transform_cam_link_color',
                                  arguments = ["0", "0", "0", "0", "0", "0", (camera_name, TextSubstitution(text='_link')), (camera_name, TextSubstitution(text='_color'))]  # '0 0 0 0 0 0 map odom'
                                 )

  return LaunchDescription([
    #args
    container_ns_arg,
    camera_name_arg,
    camera_serial_arg,
    #nodes
    cam_node,
    pc_node,
    static_cam_link_to_depth,
    static_cam_link_to_color
    ])

