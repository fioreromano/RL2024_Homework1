#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/float64_multi_array.hpp"


class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));
        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Arm Controller Node has started.");
        publishPositionCommand();
    }

private:
    // Callback per il topic joint_states
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received joint states:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }
    void publishPositionCommand() {
        std_msgs::msg::Float64MultiArray command_msg;
        command_msg.data = {1.0, 0.5, 0.4, 0.5};
        
        // Publisher per il topic /position_controller/command
        position_command_publisher_->publish(command_msg);
        RCLCPP_INFO(this->get_logger(), "Published position command: [%f, %f, %f, %f]", 
                    command_msg.data[0], command_msg.data[1], command_msg.data[2], command_msg.data[3]);
    }

    // Subscriber per il topic joint_states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    // Publisher per il topic /position_controller/command
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
