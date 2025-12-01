cmake_minimum_required(VERSION 3.8)
project(actuator_control)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(driver src/driver.cpp)
ament_target_dependencies(driver rclcpp std_msgs)
target_link_libraries(driver dxl_x64_cpp)

add_executable(position_control src/position_control.cpp)
ament_target_dependencies(position_control rclcpp std_msgs)

add_executable(velocity_control src/velocity_control.cpp)
ament_target_dependencies(velocity_control rclcpp std_msgs)

add_executable(torque_control src/torque_control.cpp)
ament_target_dependencies(torque_control rclcpp std_msgs)

add_executable(ping_test src/ping_test.cpp)
target_link_libraries(ping_test dxl_x64_cpp)
ament_target_dependencies(ping_test rclcpp)

add_executable(scan_ids src/scan_ids.cpp)
ament_target_dependencies(scan_ids rclcpp)
target_link_libraries(scan_ids dxl_x64_cpp)

add_executable(read_register src/read_register.cpp)
ament_target_dependencies(read_register rclcpp)
target_link_libraries(read_register dxl_x64_cpp)

add_executable(write_register src/write_register.cpp)
ament_target_dependencies(write_register rclcpp)
target_link_libraries(write_register dxl_x64_cpp)


install(TARGETS
    write_register
    read_register
    scan_ids
    ping_test 
    driver
    position_control
    velocity_control
    torque_control
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
