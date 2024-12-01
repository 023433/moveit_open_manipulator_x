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
    Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="static_transform_publisher",
      output="log",
    )
  )

  ld.add_action(
    Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      name="robot_state_publisher",
      output="both",
      parameters=[moveit_config.robot_description],
    )
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
      executable="ros2_control_node",
      parameters=[
        moveit_config.robot_description,
        str(moveit_config.package_path / "config/ros2_controllers.yaml"),
      ],
      remappings=[
        ("~/robot_description", "/robot_description"),
      ],
      output="both",
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

  moveit_config = (
    MoveItConfigsBuilder("open_manipulator_x", package_name="moveit_open_manipulator_x")
    .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_scene_monitor(
      publish_robot_description=True, publish_robot_description_semantic=True
    )
    .planning_pipelines(
      pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
    )
    .to_moveit_configs()
  )

  ld.add_action(
    Node(
      package="moveit_ros_move_group",
      executable="move_group",
      output="screen",
      parameters=[moveit_config.to_dict()]
    )
  )

  return ld


