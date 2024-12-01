import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
  baud_rate = LaunchConfiguration('baud_rate', default='1000000')
  interface = LaunchConfiguration('interface', default='position')
  yaml_file = LaunchConfiguration(
    'yaml_file',
    default=os.path.join(
      get_package_share_directory('moveit_open_manipulator_x'),
      'config',
      'hardware.yaml'
    )
  )

  ld = LaunchDescription()

  ld.add_action(
    Node(
      package="moveit_open_manipulator_x",
      executable="omx_control_node",
      parameters=[{
        "usb_port": usb_port,
        "baud_rate": baud_rate,
        "interface": interface,
        "yaml_file": yaml_file,
      }],
    )
  )

  return ld
