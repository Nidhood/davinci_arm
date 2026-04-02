#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "gz_ros2_control/gz_system_interface.hpp"

namespace davinci_arm_gazebo_plugins
{

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//------------------------------------------------------------------------------
// Forward declaration of the private implementation
//------------------------------------------------------------------------------

class DavinciMoveItAdapterPrivate;

//------------------------------------------------------------------------------
// MoveIt / ros2_control adapter for the Davinci Arm in Gazebo
//------------------------------------------------------------------------------

class DavinciMoveItAdapter : public gz_ros2_control::GazeboSimSystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DavinciMoveItAdapter)

    DavinciMoveItAdapter();
    ~DavinciMoveItAdapter() override;

    //--------------------------------------------------------------------------
    // Gazebo <-> ros2_control initialization
    //--------------------------------------------------------------------------

    bool initSim(
        rclcpp::Node::SharedPtr & model_nh,
        std::map<std::string, gz::sim::Entity> & joints,
        const hardware_interface::HardwareInfo & hardware_info,
        gz::sim::EntityComponentManager & ecm,
        unsigned int update_rate) override;

    //--------------------------------------------------------------------------
    // Hardware interface lifecycle
    //--------------------------------------------------------------------------

    CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;

    //--------------------------------------------------------------------------
    // ros2_control interfaces
    //--------------------------------------------------------------------------

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    //--------------------------------------------------------------------------
    // Hardware telemetry / command exchange
    //--------------------------------------------------------------------------

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    std::unique_ptr<DavinciMoveItAdapterPrivate> data_ptr_;
};

}  // namespace davinci_arm_gazebo_plugins