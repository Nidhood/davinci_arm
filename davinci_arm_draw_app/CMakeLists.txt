cmake_minimum_required(VERSION 3.8)

# Basic configuration for a ROS 2 package:
project(davinci_arm_draw_app
    VERSION 1.0.0
    DESCRIPTION "Davinci Arm 6DOF Robot Gazebo and ROS2 bridge"
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
find_package(moveit_ros_planning_interface REQUIRED)

# MoveIt2 configuration node executable:
add_executable(davinci_arm_draw_app src/davinci_arm_draw_app.cpp)
target_include_directories(davinci_arm_draw_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)
ament_target_dependencies(davinci_arm_draw_app rclcpp moveit_ros_planning_interface)

# Install executables:
install(TARGETS davinci_arm_draw_app
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories:
install(DIRECTORY config launch 
    DESTINATION share/${PROJECT_NAME}/
)

# Call ament_package macro:
ament_package()
