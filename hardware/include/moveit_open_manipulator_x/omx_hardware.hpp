#ifndef ROS2_CONTROL__OMX_HARDWARE_HPP_
#define ROS2_CONTROL__OMX_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

using hardware_interface::return_type;

namespace moveit_open_manipulator_x
{

typedef struct _ItemValue{
  std::string item_name;
  int32_t value;
} ItemValue;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  DynamixelWorkbench *dxl_wb_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}
  };

private:
  // ROS Parameters
  std::string port_name_;
  int64_t baud_rate_;
  std::string yaml_file_;
  std::string interface_;

  std::map<std::string, uint32_t> dynamixel_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;


  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool initDynamixels(const std::string yaml_file);


};


} // namespace moveit_open_manipulator_x


#endif  // ROS2_CONTROL__OMX_HARDWARE_HPP_
