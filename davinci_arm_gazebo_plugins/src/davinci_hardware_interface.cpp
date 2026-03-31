#include "davinci_arm_gazebo_plugins/davinci_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <memory>


namespace davinci_arm_hardware_interface {

//------------------------------------------------------------------------------
// Gazebo life cycle configuration
//------------------------------------------------------------------------------


// Initial configuration to gazebo.
bool DavinciHardwareInterface::initSim(rclcpp::Node::SharedPtr &model_nh,
                                       std::map<std::string, sim::Entity> &joints,
                                       const hardware_interface::HardwareInfo &/*hardware_info*/,
                                       sim::EntityComponentManager &ecm,
                                       unsigned int /*update_rate*/) {
    nh_ = model_nh;
    ecm_ = &ecm;
    enable_joints_ = joints;

    // Personified parameters loader:
    // loadParameters(hardware_info);

    gz_node_ = std::make_unique<gz::transport::Node>();
    // actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

    return true;
}

//------------------------------------------------------------------------------
// Hardware interface life cycle configuration
//------------------------------------------------------------------------------

// Initialization of all members variables and process parameters from the params argument.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) {

    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }


    return hardware_interface::CallbackReturn::SUCCESS;
}

// Setup of the communication to the hardware and set everything up so that the hardware can be activated.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {



    return hardware_interface::CallbackReturn::SUCCESS;
}

// Opposite of on_configure method.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {



    return hardware_interface::CallbackReturn::SUCCESS;
}

// Hardware "power" is enabled.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Activating... please wait...");


    RCLCPP_INFO(get_logger(), "Successfully activated! :D");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Hardware "power" is disabled.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Deactivating... please wait...");


    RCLCPP_INFO(get_logger(), "Successfully deactivated! :D");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Hardware is shutdown.
hardware_interface::CallbackReturn DavinciHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & previous_state) {


    return on_cleanup(previous_state);
}

//------------------------------------------------------------------------------
// Hardware interface telemetry
//------------------------------------------------------------------------------

hardware_interface::return_type DavinciHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type davinci_arm_hardware_interface::DavinciHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    return hardware_interface::return_type::OK;
}

//------------------------------------------------------------------------------
// Utils
//------------------------------------------------------------------------------

// void loadParameters(const hardware_interface::HardwareInfo &hardware_info) {
//     auto getParam = [&](const std::string &param_name, auto &variable, auto default_val) {
//         using T = std::decay_t<decltype(variable)>;

//         auto it = hardware_info.hardware_parameters.find(param_name);
//         if (it == hardware_info.hardware_parameters.end())
//         {
//             if constexpr (std::is_same_v<T, std::string>)
//                 variable = default_val;
//             else
//                 variable = static_cast<T>(default_val);
//             return;
//         }

//         const std::string &val_str = it->second;

//         if constexpr (std::is_same_v<T, std::string>)
//         {
//             variable = val_str;
//         }
//         else if constexpr (std::is_same_v<T, int>)
//         {
//             variable = static_cast<int>(std::stoi(val_str));
//         }
//         else if constexpr (std::is_integral_v<T>)
//         {
//             variable = static_cast<T>(std::stoll(val_str));
//         }
//         else if constexpr (std::is_floating_point_v<T>)
//         {
//             const double v = std::stod(val_str);
//             const double def = static_cast<double>(default_val);
//             variable = static_cast<T>(std::max(def, v));
//         }
//         else
//         {
//             static_assert(std::is_arithmetic_v<T>,
//                           "Unsupported parameter type in getParam");
//         }
//     };
// }

}

PLUGINLIB_EXPORT_CLASS(davinci_arm_hardware_interface::DavinciHardwareInterface,
                       gz_ros2_control::GazeboSimSystemInterface)