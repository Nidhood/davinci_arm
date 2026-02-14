cmake_minimum_required(VERSION 3.8)

# Basic configuration for a ROS 2 package:
project(davinci_arm_description
    VERSION 1.0.0
    DESCRIPTION "Davinci Arm 6DOF Robot Description"
    LANGUAGES CXX
    )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
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
find_package(geometry_msgs REQUIRED)

# Install directories:
install(DIRECTORY launch config models rviz
    DESTINATION share/${PROJECT_NAME}/
)

# Call ament_package macro:
ament_package()
