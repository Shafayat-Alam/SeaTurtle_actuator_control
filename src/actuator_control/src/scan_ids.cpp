#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scan_ids");

    const char* PORT_NAME = "/dev/ttyUSB0";
    int BAUDRATE = 57600;

    auto portHandler = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open port.");
        return 1;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set baudrate.");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "🔍 Scanning IDs 1 → 10");

    for (int id = 1; id <= 10; id++) {
        uint16_t modelNumber = 0;
        int dxl_comm_result = packetHandler->ping(portHandler, id, &modelNumber);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(node->get_logger(),
                        "✔ Found servo! ID: %d | Model: %d", id, modelNumber);
        }
    }

    rclcpp::shutdown();
    return 0;
}
