#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <unordered_map>

class DynamixelDriver : public rclcpp::Node
{
public:
    DynamixelDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("dynamixel_driver", options)
    {
        // ----------------------------------------------------
        // READ PARAMETER EARLY (THIS FIXES EVERYTHING)
        // ----------------------------------------------------
        this->declare_parameter<int>("operating_mode", 3);
        operating_mode_ = this->get_parameter("operating_mode").as_int();

        RCLCPP_INFO(this->get_logger(), 
            "Selected Operating Mode = %d (0=current, 1=velocity, 3=position)", 
            operating_mode_);

        // ----------------------------------------------------
        // SETUP BASIC CONFIG
        // ----------------------------------------------------
        PORT_NAME = "/dev/ttyUSB0";
        BAUDRATE = 57600;

        ADDR_OPERATING_MODE   = 11;
        ADDR_TORQUE_ENABLE    = 64;
        ADDR_GOAL_CURRENT     = 102;
        ADDR_GOAL_VELOCITY    = 104;
        ADDR_GOAL_POSITION    = 116;
        ADDR_PRESENT_POSITION = 132;

        ADDR_PROFILE_VELOCITY = 112;
        ADDR_CURRENT_LIMIT    = 38;
        ADDR_PWM_LIMIT        = 36;

        DXL_IDS = {1, 2};

        portHandler = dynamixel::PortHandler::getPortHandler(PORT_NAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

        portHandler->openPort();
        portHandler->setBaudRate(BAUDRATE);

        // ----------------------------------------------------
        // APPLY MODE TO SERVOS BEFORE ANY SUBSCRIBERS START
        // ----------------------------------------------------
        for (int id : DXL_IDS)
        {
            setMode(id, operating_mode_);
        }

        // ----------------------------------------------------
        // SUBSCRIBERS
        // ----------------------------------------------------
        position_subs[1] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_position", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write4(1, ADDR_GOAL_POSITION, msg->data); });

        position_subs[2] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_position", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write4(2, ADDR_GOAL_POSITION, msg->data); });

        velocity_subs[1] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_velocity", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write4(1, ADDR_GOAL_VELOCITY, msg->data); });

        velocity_subs[2] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_velocity", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write4(2, ADDR_GOAL_VELOCITY, msg->data); });

        torque_subs[1] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo1/goal_current", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write2(1, ADDR_GOAL_CURRENT, msg->data); });

        torque_subs[2] = create_subscription<std_msgs::msg::Int32>(
            "/dynamixel/servo2/goal_current", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg){ write2(2, ADDR_GOAL_CURRENT, msg->data); });

        // ----------------------------------------------------
        // STATE FEEDBACK
        // ----------------------------------------------------
        state_pub_1 = create_publisher<std_msgs::msg::Int32>("servo1/state_position", 10);
        state_pub_2 = create_publisher<std_msgs::msg::Int32>("servo2/state_position", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DynamixelDriver::publishState, this));
    }

private:

    // ----------------------------------------------------
    // SET MODE
    // ----------------------------------------------------
    void setMode(int id, int mode)
    {
        write1(id, ADDR_TORQUE_ENABLE, 0);
        write1(id, ADDR_OPERATING_MODE, mode);

        if (mode == 1)  // velocity
            write4(id, ADDR_PROFILE_VELOCITY, 0);

        if (mode == 0)  // current
        {
            write2(id, ADDR_PWM_LIMIT, 885);
            write2(id, ADDR_CURRENT_LIMIT, 1193);
        }

        write1(id, ADDR_TORQUE_ENABLE, 1);

        RCLCPP_INFO(this->get_logger(), "[ID %d] Mode SET to %d", id, mode);
    }

    // WRITE HELPERS
    void write1(int id, int addr, uint8_t value){ uint8_t e; packetHandler->write1ByteTxRx(portHandler, id, addr, value, &e); }
    void write2(int id, int addr, uint16_t value){ uint8_t e; packetHandler->write2ByteTxRx(portHandler, id, addr, value, &e); }
    void write4(int id, int addr, int32_t value){ uint8_t e; packetHandler->write4ByteTxRx(portHandler, id, addr, value, &e); }

    void publishState()
    {
        uint32_t pos;

        packetHandler->read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION, &pos);
        std_msgs::msg::Int32 p1; p1.data = pos;
        state_pub_1->publish(p1);

        packetHandler->read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION, &pos);
        std_msgs::msg::Int32 p2; p2.data = pos;
        state_pub_2->publish(p2);
    }

    // VARIABLES
    int operating_mode_;
    const char *PORT_NAME;
    int BAUDRATE;

    int ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE;
    int ADDR_GOAL_CURRENT, ADDR_GOAL_VELOCITY, ADDR_GOAL_POSITION;
    int ADDR_PRESENT_POSITION, ADDR_PROFILE_VELOCITY;
    int ADDR_CURRENT_LIMIT, ADDR_PWM_LIMIT;

    std::vector<int> DXL_IDS;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> position_subs;
    std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> velocity_subs;
    std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> torque_subs;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_1;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_2;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelDriver>());
    rclcpp::shutdown();
    return 0;
}
