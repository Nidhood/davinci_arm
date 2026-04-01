# Changelog

All notable changes to the Davinci Arm project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Complete project structure for multi-package ROS 2 workspace
- Core prop_arm meta-package with organized dependencies
- Comprehensive URDF model system with visual and collision meshes
- Advanced Gazebo simulation environment with custom world

### New important commands:

- colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
- QT_QPA_PLATFORM=xcb ros2 launch moveit_setup_assistant setup_assistant.launch.py
