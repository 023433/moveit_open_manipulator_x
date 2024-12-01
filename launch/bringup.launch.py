from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="moveit_open_manipulator_x").to_moveit_configs()
  
  # Parameters
  usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
  baud_rate = LaunchConfiguration('baud_rate', default='1000000')
  yaml_file = LaunchConfiguration(
    'yaml_file', 
    default=PathJoinSubstitution(
      [FindPackageShare("moveit_open_manipulator_x"), "config", "hardware.yaml"]
    )
  )
  interface = LaunchConfiguration('interface', default='position')

  robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution(
      [FindPackageShare("moveit_open_manipulator_x"), "config", "open_manipulator_x.urdf.xacro"]
    ),
    " ",
    "usb_port:=",
    usb_port,
    " ",
    "baud_rate:=",
    baud_rate,
    " ",
    "yaml_file:=",
    yaml_file,
    " ",
    "interface:=",
    interface,
  ])

  robot_description = {"robot_description": robot_description_content}


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
      parameters=[robot_description],
    )
  )

  ld.add_action(
    Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
        robot_description,
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


