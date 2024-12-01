
#include "moveit_open_manipulator_x/omx_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

namespace moveit_open_manipulator_x
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}


rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal){
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(rclcpp::get_logger("server"), "GoalResponse handle_goal");
  RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "Received a new goal request.");

  // Print UUID
  std::ostringstream uuid_stream;
  for (const auto & byte : uuid) {
    uuid_stream << std::hex << static_cast<int>(byte);
  }
  RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "Goal UUID: %s", uuid_stream.str().c_str());

  // Print trajectory joints
  RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "Trajectory joint names:");
  for (const auto & joint_name : goal->trajectory.joint_names) {
    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "  %s", joint_name.c_str());
  }

  // Print trajectory points
  RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "Trajectory points:");
  for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
    const auto & point = goal->trajectory.points[i];
    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "  Point %zu:", i);
    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "    Positions:");
    
    for (const auto & position : point.positions) {
      RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "      %f", position);
    }

    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "    Velocities:");
    for (const auto & velocity : point.velocities) {
      RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "      %f", velocity);
    }

    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "    Accelerations:");
    for (const auto & acceleration : point.accelerations) {
      RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "      %f", acceleration);
    }

    RCLCPP_INFO(rclcpp::get_logger("handle_goal"), "    Time from start: %f seconds", rclcpp::Duration(point.time_from_start).seconds());
  }


  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle){
  (void)goal_handle;
  RCLCPP_INFO(rclcpp::get_logger("server"), "GoalResponse handle_cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle){
  (void)goal_handle;
  RCLCPP_INFO(rclcpp::get_logger("server"), "GoalResponse handle_accepted");
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };
  
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    handle_goal,
    handle_cancel,
    handle_accepted
  );

  std::cout << "이이이이이이이이aaa" << std::endl;
  std::cout << "이이이이이이이이aaa  " << get_node()->get_name() <<std::endl;
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
  }
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  double traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  size_t ind = cur_time.seconds() * (traj_len / total_time);
  ind = std::min(static_cast<double>(ind), traj_len - 2);
  double delta = cur_time.seconds() - ind * (total_time / traj_len);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (new_msg_)
  {
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  if (trajectory_msg_ != nullptr)
  {
    interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
    for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    {
      joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
    }
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace moveit_open_manipulator_x

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moveit_open_manipulator_x::RobotController, controller_interface::ControllerInterface)
