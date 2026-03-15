#include "davinci_arm_gazebo_plugins/davinci_hardware_interface.hpp"
#include <memory>

// Initial configuration to gazebo sim:
bool DavinciHardwareInterface::initSim(rclcpp::Node::SharedPtr &model_nh,
                                       std::map<std::string, sim::Entity> &joints,
                                       const hardware_interface::HardwareInfo &hardware_info,
                                       sim::EntityComponentManager &ecm,
                                       unsigned int update_rate) {
    nh_ = model_nh;
    ecm_ = &ecm;
    enable_joints_ = joints;

    // Personified parameters loader:
    loadParameters(hardware_info);

    gz_node_ = std::make_unique<gz::transport::Node>();
    // actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

    return true;
}

void loadParameters(const hardware_interface::HardwareInfo &hardware_info) {
    auto getParam = [&](const std::string &param_name, auto &variable, auto default_val) {
        using T = std::decay_t<decltype(variable)>;

        auto it = hardware_info.hardware_parameters.find(param_name);
        if (it == hardware_info.hardware_parameters.end())
        {
            if constexpr (std::is_same_v<T, std::string>)
                variable = default_val;
            else
                variable = static_cast<T>(default_val);
            return;
        }

        const std::string &val_str = it->second;

        if constexpr (std::is_same_v<T, std::string>)
        {
            variable = val_str;
        }
        else if constexpr (std::is_same_v<T, int>)
        {
            variable = static_cast<int>(std::stoi(val_str));
        }
        else if constexpr (std::is_integral_v<T>)
        {
            variable = static_cast<T>(std::stoll(val_str));
        }
        else if constexpr (std::is_floating_point_v<T>)
        {
            const double v = std::stod(val_str);
            const double def = static_cast<double>(default_val);
            variable = static_cast<T>(std::max(def, v));
        }
        else
        {
            static_assert(std::is_arithmetic_v<T>,
                          "Unsupported parameter type in getParam");
        }
    };
}