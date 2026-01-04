#include "noxbot_hardware/mobile_base_hw_interface.hpp"
#include "noxbot_hardware/esp_transport.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mobile_base_hardware
{
hardware_interface::CallbackReturn MobileBaseHwInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  node_ = std::make_shared<rclcpp::Node>("mobile_base_hw_interface");
  info_ = info;
  left_joint_name_ = info.joints[0].name;
  right_joint_name_ = info.joints[1].name;

  left_wheel_position_ = 0.0;
  right_wheel_position_ = 0.0;
  left_wheel_velocity_ = 0.0;
  right_wheel_velocity_ = 0.0;

  left_wheel_velocity_cmd_ = 0.0;
  right_wheel_velocity_cmd_ = 0.0;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileBaseHwInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(left_joint_name_, hardware_interface::HW_IF_POSITION, &left_wheel_position_);
  state_interfaces.emplace_back(left_joint_name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_);
  state_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_POSITION, &right_wheel_position_);
  state_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_);
  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> MobileBaseHwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(left_joint_name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_cmd_);
  command_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_cmd_);
  return command_interfaces;
}

hardware_interface::CallbackReturn MobileBaseHwInterface::on_configure(const rclcpp_lifecycle::State&)
{
  // CREATE ESP TRANSPORT (BORROW THE NODE)
  esp_transport_ = std::make_unique<EspTransport>(node_.get());
  esp_transport_->init();
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn MobileBaseHwInterface::on_activate(const rclcpp_lifecycle::State&)
{
  left_wheel_velocity_cmd_ = 0.0;
  right_wheel_velocity_cmd_ = 0.0;
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn MobileBaseHwInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  // Send zero command before stopping
  if (esp_transport_)
  {
    esp_transport_->sendWheelCommands(0.0, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHwInterface::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
    //for demo without encoder
    left_wheel_velocity_ = left_wheel_velocity_cmd_;
    right_wheel_velocity_ = right_wheel_velocity_cmd_;

    left_wheel_position_ += left_wheel_velocity_ * period.seconds();
    right_wheel_position_ += right_wheel_velocity_ * period.seconds();

  // read from ESP
//   if (esp_transport_ && esp_transport_->isAlive())
//   {
//     left_wheel_velocity_ = esp_transport_->getLeftVelocity();
//     right_wheel_velocity_ = esp_transport_->getRightVelocity();

//     left_wheel_position_ = esp_transport_->getLeftPosition();
//     right_wheel_position_ = esp_transport_->getRightPosition();
//   }
//   else
//   {
//     // SAFETY: stop reporting motion
//     left_wheel_velocity_ = 0.0;
//     right_wheel_velocity_ = 0.0;
//   }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MobileBaseHwInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  // send left_wheel_velocity and right_wheel_velocity to esp
  if (esp_transport_)
  {
    if (!esp_transport_->isAlive())
    {
      // Warn but still forward commands so the ESP can move even before feedback arrives
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "ESP feedback stale; forwarding commands anyway");
    }

    esp_transport_->sendWheelCommands(left_wheel_velocity_cmd_, right_wheel_velocity_cmd_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mobile_base_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHwInterface, hardware_interface::SystemInterface)