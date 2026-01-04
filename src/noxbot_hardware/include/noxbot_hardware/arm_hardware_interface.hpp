#ifndef ARM_HARDWARE_INTERFACE
#define ARM_HARDWARE_INTERFACE
#include "hardware_interface/system_interface.hpp"

namespace arm_hardware
{
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // cycle node override
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // system_interface override
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // interface declaration
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
   double arm_joint1_position_=0.0;
   double arm_joint2_position_=0.0;

   double arm_joint1_pos_cmd_=0.0;
   double arm_joint2_pos_cmd_=0.0;

   std::string arm_joint1_name_;
   std::string arm_joint2_name_;
};
}  // namespace arm_hardware

#endif