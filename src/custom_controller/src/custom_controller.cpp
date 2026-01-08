#include "custom_controller/custom_controller.hpp"

namespace arm_controller
{
    ArmController::ArmController() : controller_interface::ControllerInterface()
    {
    }

    controller_interface::CallbackReturn ArmController::on_init()
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        interface_name_ = auto_declare<std::string>("interface_name", "position");
        coefficient = auto_declare<double>("coefficient", 0.8);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto callback = [this](const FloatArray::SharedPtr msg) -> void
        {
            if (msg->data.size() == joint_names_.size())
            {
                app_command_.clear();
                for (auto cmd : msg->data)
                {
                    app_command_.push_back(cmd);
                }
            }
        };
        command_subscriber_ = get_node()->create_subscription<FloatArray>("/joints_command", 10, callback);
        return CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/" + interface_name_);
        }
        return config;
    }
    controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/" + interface_name_);
        }
        return config;
    }

    controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State &)
    {
        app_command_.clear();
        for (int i = 0; i < (int)joint_names_.size(); i++)
        {
            app_command_.push_back(state_interfaces_[i].get_value());
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ArmController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for(int i=0;i<(int)joint_names_.size();i++)
        {
            double state= state_interfaces_[i].get_value();
            double cmd=app_command_[i];
            double new_cmd=cmd*coefficient+state*(1-coefficient);
            (void)command_interfaces_[i].set_value(new_cmd);

        }
        return controller_interface::return_type::OK;
    }

} // namespace arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController,controller_interface::ControllerInterface)