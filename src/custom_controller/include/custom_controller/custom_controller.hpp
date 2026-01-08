
#ifndef CUSTOM_CONTROLLER_HPP
#define CUSTOM_CONTROLLER_HPP

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

using FloatArray =  example_interfaces::msg::Float64MultiArray;
namespace arm_controller
{
    class ArmController: public controller_interface::ControllerInterface
    {
     public:   
        ArmController();

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;

        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
     protected:
      std::vector<std::string>joint_names_;
      std::string interface_name_;
      double coefficient;

      std::vector<double>app_command_;
      rclcpp::Subscription<FloatArray>::SharedPtr command_subscriber_;

    };
}

#endif