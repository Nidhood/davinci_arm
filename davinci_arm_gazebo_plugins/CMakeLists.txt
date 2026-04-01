cmake_minimum_required(VERSION 3.8)

# Basic configuration for a ROS 2 package:
project(davinci_arm_gazebo_plugins
    VERSION 1.0.0
    DESCRIPTION "Davinci Arm 6DOF Robot Gazebo Spawner"
    LANGUAGES CXX
    )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall 
        -Wextra 
        -Wpedantic
        -Werror=return-type
        )
endif()

# Bring in the package dependencies:
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(davinci_arm_msgs REQUIRED)

# Add libraries:
add_library(davinci_arm_hardware_interface SHARED
    src/davinci_hardware_interface.cpp
)
target_include_directories(davinci_arm_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(davinci_arm_hardware_interface
    hardware_interface pluginlib rclcpp rclcpp_lifecycle gz_ros2_control std_msgs
    gz-sim8 gz-transport13 gz-msgs10 davinci_arm_msgs
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf/davinci_arm_hardware_interface_plugin.xml")
    pluginlib_export_plugin_description_file(gz_ros2_control urdf/davinci_arm_hardware_interface_plugin.xml)
endif()

# Install libraries:
install(TARGETS
    davinci_arm_hardware_interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install directories:
install(DIRECTORY config urdf
    DESTINATION share/${PROJECT_NAME}/
)

# Install plugin only if it exists:
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf/davinci_arm_hardware_interface_plugin.xml")
    install(FILES urdf/davinci_arm_hardware_interface_plugin.xml DESTINATION share/${PROJECT_NAME})
endif()


# Call ament_package macro:
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    hardware_interface pluginlib rclcpp rclcpp_lifecycle gz_ros2_control
)
ament_package()
