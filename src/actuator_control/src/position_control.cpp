#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class PositionControlNode : public rclcpp::Node
{
public:
    PositionControlNode()
    : Node("position_control_node")
    {
        pos1_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_position", 10);

        pos2_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_position", 10);

        RCLCPP_INFO(this->get_logger(), "Position Control Node started.");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PositionControlNode::timerCallback, this)
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pos1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pos2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_ = false;

    void timerCallback()
    {
        std_msgs::msg::Int32 msg;
        msg.data = toggle_ ? 1024 : 0;
        toggle_ = !toggle_;

        RCLCPP_INFO(this->get_logger(), "Publishing Position: %d", msg.data);

        pos1_pub_->publish(msg);
        pos2_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionControlNode>());
    rclcpp::shutdown();
    return 0;
}
