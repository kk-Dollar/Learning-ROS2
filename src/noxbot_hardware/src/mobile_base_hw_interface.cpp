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

  // angular_scale amplifies turn (differential) so wheel difference exceeds motor deadzone
  angular_scale_ = 3.0;
  const auto it = info.hardware_parameters.find("angular_scale");
  if (it != info.hardware_parameters.end())
  {
    angular_scale_ = std::stod(it->second);
  }
  RCLCPP_INFO(node_->get_logger(), "mobile_base angular_scale = %.2f", angular_scale_);

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
  // Read from ESP when connected
  if (esp_transport_ && esp_transport_->isAlive())
  {
    left_wheel_velocity_ = esp_transport_->getLeftVelocity();
    right_wheel_velocity_ = esp_transport_->getRightVelocity();
    left_wheel_position_ = esp_transport_->getLeftPosition();
    right_wheel_position_ = esp_transport_->getRightPosition();
  }
  else
  {
    // Fallback when no encoder feedback: use commanded velocity and integrate position
    // so diff_drive_controller odometry (and RViz) still update when driving
    left_wheel_velocity_ = left_wheel_velocity_cmd_;
    right_wheel_velocity_ = right_wheel_velocity_cmd_;
    left_wheel_position_ += left_wheel_velocity_ * period.seconds();
    right_wheel_position_ += right_wheel_velocity_ * period.seconds();
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MobileBaseHwInterface::write(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  // if (!esp_transport_) return hardware_interface::return_type::OK;

  // if (!esp_transport_->isAlive())
  // {
  //   // HARD SAFETY STOP
  //   esp_transport_->sendWheelCommands(0.0, 0.0);
  //   return hardware_interface::return_type::OK;
  // }

  // esp_transport_->sendWheelCommands(
  //   left_wheel_velocity_cmd_,
  //   right_wheel_velocity_cmd_);

  // return hardware_interface::return_type::OK;
    RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "WRITE(): left=%.3f right=%.3f",
    left_wheel_velocity_cmd_,
    right_wheel_velocity_cmd_
  );

  if (esp_transport_)
  {
    // Amplify differential (turn) so wheel difference exceeds motor deadzone
    const double mid = (left_wheel_velocity_cmd_ + right_wheel_velocity_cmd_) * 0.5;
    const double left_out = mid + angular_scale_ * (left_wheel_velocity_cmd_ - mid);
    const double right_out = mid + angular_scale_ * (right_wheel_velocity_cmd_ - mid);
    esp_transport_->sendWheelCommands(left_out, right_out);
  }
  return hardware_interface::return_type::OK;
}


}  // namespace mobile_base_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHwInterface, hardware_interface::SystemInterface)