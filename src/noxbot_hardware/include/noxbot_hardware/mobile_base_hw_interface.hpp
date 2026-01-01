#ifndef MOBILE_BASE_HW_INTERFACE
#define MOBILE_BASE_HW_INTERFACE

#include "hardware_interface/system_interface.hpp"
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

private:
}; //class

}  // namespace mobile_base_hardware
#endif