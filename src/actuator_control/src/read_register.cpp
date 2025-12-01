#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

class ReadRegisterNode : public rclcpp::Node
{
public:
    ReadRegisterNode(int id, int addr)
    : Node("read_register_node")
    {
        auto port = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        auto packet = dynamixel::PacketHandler::getPacketHandler(2.0);

        port->openPort();
        port->setBaudRate(57600);

        uint8_t data8 = 0;
        uint32_t data32 = 0;
        uint8_t dxl_error;

        // Decide read size based on address
        if (addr <= 200)  // simple rule: small registers are 1 byte
        {
            packet->read1ByteTxRx(port, id, addr, &data8, &dxl_error);
            RCLCPP_INFO(get_logger(), "Register[%d] (1 byte) for ID %d = %d",
                        addr, id, data8);
        }
        else
        {
            packet->read4ByteTxRx(port, id, addr, &data32, &dxl_error);
            RCLCPP_INFO(get_logger(), "Register[%d] (4 bytes) for ID %d = %d",
                        addr, id, data32);
        }

        rclcpp::shutdown();
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        printf("Usage: read_register <id> <address>\n");
        return 1;
    }

    int id   = std::stoi(argv[1]);
    int addr = std::stoi(argv[2]);

    rclcpp::spin(std::make_shared<ReadRegisterNode>(id, addr));
    return 0;
}
