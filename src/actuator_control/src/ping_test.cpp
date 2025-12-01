#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>

int main()
{
    const char* PORT_NAME = "/dev/ttyUSB0";
    const int BAUDRATE = 57600;

    auto portHandler = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler->openPort()) {
        std::cout << "❌ Failed to open port\n";
        return 0;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cout << "❌ Failed to set baudrate\n";
        return 0;
    }

    std::cout << "\n🔍 Scanning IDs 1 → 10\n";

    for (int id = 1; id <= 10; id++) {
        uint16_t model = 0;
        uint8_t dxl_error = 0;

        int result = packetHandler->ping(portHandler, id, &model, &dxl_error);

        if (result == COMM_SUCCESS) {
            std::cout << "✅ Found servo! ID: " << id
                      << " | Model: " << model << "\n";
        }
    }

    portHandler->closePort();
    return 0;
}
