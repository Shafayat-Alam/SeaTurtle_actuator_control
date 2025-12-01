#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class TorqueToggleNode : public rclcpp::Node
{
public:
    TorqueToggleNode()
    : Node("torque_toggle_node")
    {
        torque1_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_current", 10);

        torque2_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_current", 10);

        RCLCPP_INFO(this->get_logger(), "Torque Toggle Node started.");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TorqueToggleNode::timerCallback, this)
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr torque1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr torque2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_ = false;

    void timerCallback()
    {
        std_msgs::msg::Int32 msg;
        msg.data = toggle_ ? 50 : -50;
        toggle_ = !toggle_;

        RCLCPP_INFO(this->get_logger(), "Publishing Torque: %d", msg.data);

        torque1_pub_->publish(msg);
        torque2_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueToggleNode>());
    rclcpp::shutdown();
    return 0;
}
