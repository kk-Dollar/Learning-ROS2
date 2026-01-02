#ifndef MOBILE_BASE_HW_INTERFACE
#define MOBILE_BASE_HW_INTERFACE

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/node.hpp>
#include "noxbot_hardware/esp_transport.hpp"
namespace mobile_base_hardware
{
class MobileBaseHwInterface : public hardware_interface::SystemInterface
{
public:
  // lifecycle node override
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  //system interface override
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  //interface declaration
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  //state
  double left_wheel_position_=0.0;
  double right_wheel_position_=0.0;
  double left_wheel_velocity_=0.0;
  double right_wheel_velocity_=0.0;
  
  //command
  double left_wheel_velocity_cmd_=0.0;
  double right_wheel_velocity_cmd_=0.0;
  
  //joint names
  std::string left_joint_name_;
  std::string right_joint_name_;

  //esp_transport variable
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<EspTransport> esp_transport_;

}; //class

}  // namespace mobile_base_hardware
#endif