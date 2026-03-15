#pragma once

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <map>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <gz/transport/Node.hh>

#include "gz_ros2_control/gz_system_interface.hpp"

class DavinciHardwareInterface : public gz_ros2_control::GazeboSimSystemInterface {
private:

    // Gazebo transport:
    sim::EntityComponentManager *ecm_{nullptr};
    std::map<std::string, sim::Entity> enable_joints_;
    std::unique_ptr<gz::transport::Node> gz_node_;
    gz::transport::Node::Publisher actuators_pub_;

    // ROS2 & Gazebo communication:
    rclcpp::Node::SharedPtr nh_;
    std::string actuators_topic_{"/davinci_arm/command/angles"};
    std::string joint_name_{"arm_link_joint"};

    // ROS2 communication:

    // Real motors characterization parameters loader:
    void loadParameters(const hardware_interface::HardwareInfo &hardware_info);
public:
    DavinciHardwareInterface() = default;
    ~DavinciHardwareInterface() = default;

    bool initSim(
        rclcpp::Node::SharedPtr &model_nh,
        std::map<std::string, sim::Entity> &joints,
        const hardware_interface::HardwareInfo &hardware_info,
        sim::EntityComponentManager &ecm,
        unsigned int update_rate
    ) override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

};