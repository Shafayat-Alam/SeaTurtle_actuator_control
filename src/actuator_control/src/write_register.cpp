#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

class WriteRegisterNode : public rclcpp::Node
{
public:
    WriteRegisterNode(int id, int addr, int value)
    : Node("write_register_node")
    {
        auto port = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        auto packet = dynamixel::PacketHandler::getPacketHandler(2.0);

        port->openPort();
        port->setBaudRate(57600);

        uint8_t dxl_error;

        if (value <= 255)
        {
            packet->write1ByteTxRx(port, id, addr, value, &dxl_error);
            RCLCPP_INFO(get_logger(), "WROTE 1 byte: ID %d Register[%d] = %d",
                        id, addr, value);
        }
        else
        {
            packet->write4ByteTxRx(port, id, addr, value, &dxl_error);
            RCLCPP_INFO(get_logger(), "WROTE 4 bytes: ID %d Register[%d] = %d",
                        id, addr, value);
        }

        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 4) {
        printf("Usage: write_register <id> <address> <value>\n");
        return 1;
    }

    int id    = std::stoi(argv[1]);
    int addr  = std::stoi(argv[2]);
    int value = std::stoi(argv[3]);

    rclcpp::spin(std::make_shared<WriteRegisterNode>(id, addr, value));
    return 0;
}
