cmake_minimum_required(VERSION 3.8)

project(davinci_arm_gazebo_plugins
    VERSION 1.0.0
    DESCRIPTION "Davinci Arm Gazebo plugins and MoveIt adapter"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Library:
add_library(davinci_arm_moveit_adapter SHARED
    src/davinci_moveit_adapter.cpp
)

target_include_directories(davinci_arm_moveit_adapter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(davinci_arm_moveit_adapter
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    gz_ros2_control
    gz-sim8
    gz-transport13
    gz-msgs10
)

pluginlib_export_plugin_description_file(
    gz_ros2_control
    urdf/davinci_arm_moveit_adapter_plugin.xml
)

# Install:
install(
    TARGETS davinci_arm_moveit_adapter
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

# Install directories:
install(DIRECTORY config launch urdf
    DESTINATION share/${PROJECT_NAME}/
)

# Export:
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    gz_ros2_control
    gz-sim8
    gz-transport13
    gz-msgs10
)

ament_package()