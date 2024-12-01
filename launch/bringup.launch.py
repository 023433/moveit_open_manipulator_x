import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="moveit_open_manipulator_x").to_moveit_configs()

  # Parameters
  usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
  baud_rate = LaunchConfiguration('baud_rate', default='1000000')
  param_dir = LaunchConfiguration(
    'param_dir',
    default=os.path.join(
      get_package_share_directory('open_manipulator_x_controller'),
      'param',
      'open_manipulator_x_controller_params.yaml'
    )
  )

  robot_name = "open_manipulator_x"
  package_name = robot_name + "_description"

  urdf_file = os.path.join(
    get_package_share_directory(package_name),
    'urdf',
    robot_name + ".urdf.xacro"
  )

  print('urdf_file_name : {}'.format(urdf_file))

  with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()

  ld = LaunchDescription()

  ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
  ld.add_action(
    DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
  )
  ld.add_action(
    DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
  )

  ld.add_action(
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      respawn=True,
      parameters=[
        {
          "robot_description": robot_desc,
        },
      ],
      output='screen'
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "joint_state_broadcaster",
          "--controller-manager-timeout",
          "300",
          "--controller-manager",
          "/controller_manager",
      ]
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="spawner",
      arguments=["arm_controller", "-c", "/controller_manager"],
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="spawner",
      arguments=["gripper_controller", "-c", "/controller_manager"],
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
        moveit_config.robot_description,
        str(moveit_config.package_path / "config/ros2_controllers.yaml"),
      ],
    )
  )

  should_publish = LaunchConfiguration("publish_monitored_planning_scene")

  move_group_configuration = {
    "publish_robot_description_semantic": True,
    "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
    "publish_planning_scene": should_publish,
    "publish_geometry_updates": should_publish,
    "publish_state_updates": should_publish,
    "publish_transforms_updates": should_publish,
    "monitor_dynamics": False,
  }

  move_group_params = [
    moveit_config.to_dict(),
    move_group_configuration,
  ]

  add_debuggable_node(
    ld,
    package="moveit_ros_move_group",
    executable="move_group",
    commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
    output="screen",
    parameters=move_group_params,
    extra_debug_args=["--debug"],
    # Set the display variable, in case OpenGL code is used internally
    additional_env={"DISPLAY": os.environ["DISPLAY"]},
  )
  
  return ld


