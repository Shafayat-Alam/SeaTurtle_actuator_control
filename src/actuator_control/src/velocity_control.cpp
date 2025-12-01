#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class VelocityControlNode : public rclcpp::Node
{
public:
    VelocityControlNode()
    : Node("velocity_control_node")
    {
        vel1_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_velocity", 10);

        vel2_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_velocity", 10);

        RCLCPP_INFO(this->get_logger(), "Velocity Control Node started.");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&VelocityControlNode::timerCallback, this)
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vel1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vel2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_ = false;

    void timerCallback()
    {
        std_msgs::msg::Int32 msg;

        // WX430-T200 → velocity is ±(0–2047)
        msg.data = toggle_ ? 200 : -200;
        toggle_ = !toggle_;

        RCLCPP_INFO(this->get_logger(), "Publishing Velocity: %d", msg.data);

        vel1_pub_->publish(msg);
        vel2_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityControlNode>());
    rclcpp::shutdown();
    return 0;
}
