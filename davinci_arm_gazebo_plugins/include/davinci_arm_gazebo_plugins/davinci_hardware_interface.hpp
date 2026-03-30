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
#include "davinci_arm_msgs/msg/joints_states.hpp"


namespace davinci_arm_hardware_interface{

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
        rclcpp::Publisher<davinci_arm_msgs::msg::JointsStates> joints_states_pub_;

        // Real motors characterization parameters loader:
        void loadParameters(const hardware_interface::HardwareInfo &hardware_info);

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DavinciHardwareInterface)

        bool initSim(
            rclcpp::Node::SharedPtr &model_nh,
            std::map<std::string, sim::Entity> &joints,
            const hardware_interface::HardwareInfo &hardware_info,
            sim::EntityComponentManager &ecm,
            unsigned int update_rate
        ) override;


        // Hardware interface life cycle configurations:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams & params) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State & previous_state) override;

        // Hardware interface information exchange (Gazebo Harmonic <-> ROS2):
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };

} // namespace davinci_arm_hardware_interface
