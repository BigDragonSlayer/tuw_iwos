#ifndef TUW_IWOS_STEERING_CONTROLLER_HPP_
#define TUW_IWOS_STEERING_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"  // for the torque service requests

namespace tuw_iwos_steering_controller {
    class SteeringController : public controller_interface::ControllerInterface
    {
        public:
            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            // controller parameters
            std::string left_gpio = "left_gpio"; // TODO think about naming it the correct name by default
            std::string left_joint = "left_joint"; // TODO think about naming it the correct name by default
            double cmd_expiration = 0.5;  // TODO think about what default value would be good

            // subscription
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr double_subscriber;
            double cmd;
            rclcpp::Time last_cmd;

            // service clients
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_client;

            // interfaces
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_position;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_position;

    };
}

#endif // TUW_IWOS_STEERING_CONTROLLER_H_
