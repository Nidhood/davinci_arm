cmake_minimum_required(VERSION 3.8)

project(davinci_arm_characterization 
    VERSION 1.0.0 
    DESCRIPTION "Davinci Arm characterization of physical plant"
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
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Werror=return-type
    )
endif()

# Bring in the package dependencies:
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add libraries:
add_library(joint_tf_visualizer_lib
    src/joint_tf_visualizer.cpp
)

target_include_directories(joint_tf_visualizer_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joint_tf_visualizer_lib
    geometry_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    urdf
    visualization_msgs
)

# Add executables:
add_executable(joint_tf_visualizer_node
    src/joint_tf_visualizer_node.cpp
)

target_link_libraries(joint_tf_visualizer_node
    joint_tf_visualizer_lib
)

ament_target_dependencies(joint_tf_visualizer_node
    rclcpp
)

# Install targets:
install(
    TARGETS
    joint_tf_visualizer_lib
    joint_tf_visualizer_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories:
install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    geometry_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    urdf
    visualization_msgs
)

# Package setup
ament_package()
