#include "moveit_open_manipulator_x/omx_hardware.hpp"
#include <string>
#include <vector>

namespace moveit_open_manipulator_x
{

auto logger_ = rclcpp::get_logger("handle_goal");

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }


  RCLCPP_INFO(logger_, "HardwareInfo name: %s", info.name.c_str());
  RCLCPP_INFO(logger_, "HardwareInfo type: %s", info.type.c_str());

  RCLCPP_INFO(logger_, "HardwareInfo usb_port: %s", info_.hardware_parameters["usb_port"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo baud_rate: %s", info_.hardware_parameters["baud_rate"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo yaml_file: %s", info_.hardware_parameters["yaml_file"].c_str());
  RCLCPP_INFO(logger_, "HardwareInfo interface: %s", info_.hardware_parameters["interface"].c_str());
  
  std::string port_name = info_.hardware_parameters["usb_port"];
  uint32_t baud_rate = stoi(info_.hardware_parameters["baud_rate"]);

  initWorkbench(port_name, baud_rate);


  for (const auto & joint_name : info.joints) {
    RCLCPP_INFO(logger_, "joint_name %s / %s", joint_name.name.c_str(), joint_name.type.c_str());
  }

  joint_position_.assign(5, 0);
  joint_velocities_.assign(5, 0);
  joint_position_command_.assign(5, 0);
  joint_velocities_command_.assign(5, 0);

  ft_states_.assign(5, 0);
  ft_command_.assign(5, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber

  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

bool RobotSystem::initWorkbench(const std::string port_name, const uint32_t baud_rate){
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  return result;
}




} // namespace moveit_open_manipulator_x


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moveit_open_manipulator_x::RobotSystem, hardware_interface::SystemInterface)
