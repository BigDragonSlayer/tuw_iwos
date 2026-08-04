#include "tuw_iwos_steering_controller/tuw_iwos_steering_controller.hpp"

namespace tuw_iwos_steering_controller {

    using namespace controller_interface;
    using namespace rclcpp_lifecycle;

    CallbackReturn SteeringController::on_init() {
        this->left_gpio = auto_declare<std::string>("left_gpio", this->left_gpio);
        //this->right_gpio = auto_declare<std::string>("right_gpio", this->right_gpio);         // TODO add once the left hinge is working
        // TODO the joints themselves as parameters
        this->left_joint = auto_declare<std::string>("left_joint", this->left_joint);

        this->cmd_expiration = auto_declare<double>("cmd_expiration", this->expiration);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SteeringController::on_configure(const State &previous_state) {
        // TODO initialize variables

        // TODO subscribers
        auto double_callback = [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
            this->cmd = msg->data;
            this->last_cmd = get_node()->get_clock()->now();
        };
        this->double_subscriber = get_node()->create_subscription<std_msgs::msg::Float64>("left_position", 10, double_callback);

        // TODO publishers

        // TODO service client for torque
        this->torque_client = get_node()->create_client<std_srvs::srv::SetBool>("dynamixel_hardware_interface/set_dxl_torque");

        return CallbackReturn::SUCCESS
    }

    InterfaceConfiguration SteeringController::command_interface_configuration() {
        // return a command interface configuration, that includes all the command interfaces the controller claims
        std::vector<std::string> conf_names;

        conf_names.push_back(this->left_joint + "/position"); // TODO replace with HW_IF_POSITION
        // TODO include the other command interfaces

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration SteeringController::state_interface_configuration() {
        // TODO same as for the command_interface_configuration, but for the state interfaces to be read
        std::vector<std::string> conf_names;

        conf_names.push_back(this->left_joint + "/position"); // TODO replace with HW_IF_POSITION

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    CallbackReturn SteeringController::on_activate(const State &previous_state) {
        // TODO store command interfaces into their respective variables
        for(auto& interface : command_interfaces_) {
            if(interface.get_prefix_name() == this->left_joint) {
                if(interface.get_interface_name() == "position") {
                    this->cmd_position.push_back(std::ref(interface));
                }
            }
        }

        // TODO store state interfaces into their respective variables
        for(auto& interface : state_interfaces_) {
            if(interface.get_prefix_name() == this->left_joint) {
                if(interface.get_interface_name() == "position") {
                    this->state_position.push_back(std::ref(interface));
                }
            }
        }

        // TODO enable torque
        // TODO put that in some method
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        while(!torque_client->wait_for_service(1s)) {
            if(!rclcpp::ok()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service to enable torque.");
                return CallbackReturn::ERROR;
            }
        }

        auto result = torque_client->async_send_request(request);
        if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            if(result.get()->success) {
                RCLCPP_INFO(get_node()->get_logger(), result.get()->message);
            } else {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to request to enable torque");
                return CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to call service for enabling torque.");
            return CallbackReturn::ERROR;
        }

        // TODO set all command interfaces to an initial default value
        this->cmd_position[0].get().set_value(0.0);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SteeringController::on_deactivate(const State &previous_state) {
        // TODO set all command interfaces to a default value
        // TODO disable torque

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SteeringController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        // TODO the real deal

        return CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_iwos_steering_controller::SteeringController, controller_interface::ControllerInterface)
