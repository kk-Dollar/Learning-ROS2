#include "noxbot_hardware/arm_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm_hardware
{
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
    if(SystemInterface::on_init(info)!=CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    info_=info;
    arm_joint1_name_=info.joints[0].name;
    arm_joint2_name_=info.joints[1].name;
    arm_joint1_position_=0.0;
    arm_joint2_position_=0.0;
    arm_joint1_pos_cmd_=0.0;
    arm_joint2_pos_cmd_=0.0;

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
   std::vector<hardware_interface::StateInterface> state_interface;
   state_interface.emplace_back(arm_joint1_name_,hardware_interface::HW_IF_POSITION,&arm_joint1_position_);
   state_interface.emplace_back(arm_joint2_name_,hardware_interface::HW_IF_POSITION,&arm_joint2_position_);
   return state_interface;
}
std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interface;
   command_interface.emplace_back(arm_joint1_name_,hardware_interface::HW_IF_POSITION,&arm_joint1_pos_cmd_);
   command_interface.emplace_back(arm_joint2_name_,hardware_interface::HW_IF_POSITION,&arm_joint2_pos_cmd_);
   return command_interface;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    arm_joint1_position_=arm_joint1_pos_cmd_;
    arm_joint2_position_=arm_joint2_pos_cmd_;
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    return hardware_interface::return_type::OK;
}



}  // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)