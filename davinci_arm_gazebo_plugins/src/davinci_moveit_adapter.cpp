#include "davinci_arm_gazebo_plugins/davinci_moveit_adapter.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace sim = gz::sim;

namespace davinci_arm_gazebo_plugins
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kCommandEpsilon = 1e-12;

//------------------------------------------------------------------------------
// Utils
//------------------------------------------------------------------------------

std::string strip_joint_suffix(std::string joint_name)
{
    const std::string suffix_link_joint = "_link_joint";
    const std::string suffix_joint = "_joint";

    if (
        joint_name.size() >= suffix_link_joint.size() &&
        joint_name.compare(
            joint_name.size() - suffix_link_joint.size(),
            suffix_link_joint.size(),
            suffix_link_joint) == 0)
    {
        joint_name.erase(joint_name.size() - suffix_link_joint.size());
        return joint_name;
    }

    if (
        joint_name.size() >= suffix_joint.size() &&
        joint_name.compare(
            joint_name.size() - suffix_joint.size(),
            suffix_joint.size(),
            suffix_joint) == 0)
    {
        joint_name.erase(joint_name.size() - suffix_joint.size());
        return joint_name;
    }

    return joint_name;
}

std::string default_gz_topic_from_joint_name(const std::string & joint_name)
{
    return "/davinci_arm/" + strip_joint_suffix(joint_name) + "_cmd_pos";
}

double parse_initial_value(
    const hardware_interface::InterfaceInfo & interface_info,
    const rclcpp::Logger & logger,
    const std::string & joint_name)
{
    if (interface_info.initial_value.empty()) {
        return kNaN;
    }

    try {
        return hardware_interface::stod(interface_info.initial_value);
    } catch (const std::exception &) {
        RCLCPP_ERROR(
            logger,
            "Failed parsing initial_value='%s' for joint '%s' interface '%s'.",
            interface_info.initial_value.c_str(),
            joint_name.c_str(),
            interface_info.name.c_str());
        throw;
    }
}

std::string resolve_command_topic(
    const hardware_interface::ComponentInfo & joint_info,
    const rclcpp::Logger & logger)
{
    for (const auto & command_interface : joint_info.command_interfaces) {
        if (command_interface.name != hardware_interface::HW_IF_POSITION) {
            continue;
        }

        const auto it_gz_topic = command_interface.parameters.find("gz_topic");
        if (it_gz_topic != command_interface.parameters.end() && !it_gz_topic->second.empty()) {
            return it_gz_topic->second;
        }

        const auto it_command_topic = command_interface.parameters.find("command_topic");
        if (
            it_command_topic != command_interface.parameters.end() &&
            !it_command_topic->second.empty())
        {
            return it_command_topic->second;
        }
    }

    const std::string inferred_topic = default_gz_topic_from_joint_name(joint_info.name);
    RCLCPP_WARN(
        logger,
        "Joint '%s' has no explicit 'gz_topic' parameter. Using inferred topic '%s'.",
        joint_info.name.c_str(),
        inferred_topic.c_str());

    return inferred_topic;
}

double compute_scalar_effort(
    const sdf::JointType joint_type,
    const sdf::JointAxis & joint_axis,
    const gz::msgs::Wrench & wrench_msg)
{
    const auto & axis = joint_axis.Xyz();

    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

    if (joint_type == sdf::JointType::PRISMATIC) {
        wx = wrench_msg.force().x();
        wy = wrench_msg.force().y();
        wz = wrench_msg.force().z();
    } else {
        wx = wrench_msg.torque().x();
        wy = wrench_msg.torque().y();
        wz = wrench_msg.torque().z();
    }

    return axis[0] * wx + axis[1] * wy + axis[2] * wz;
}

}  // namespace

//------------------------------------------------------------------------------
// Private implementation
//------------------------------------------------------------------------------

struct JointData
{
    // Joint identification
    std::string name;
    sim::Entity sim_joint{sim::kNullEntity};

    // Gazebo joint metadata
    sdf::JointType joint_type{sdf::JointType::REVOLUTE};
    sdf::JointAxis joint_axis{};

    // ros2_control state interfaces
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};

    // ros2_control command interfaces
    double position_cmd{0.0};

    // Interface availability flags
    bool has_position_state{false};
    bool has_velocity_state{false};
    bool has_effort_state{false};
    bool has_position_command{false};

    // Gazebo transport command topic
    std::string gz_command_topic;
    gz::transport::Node::Publisher gz_position_pub;
    bool has_position_publisher{false};

    // Internal command tracking
    double last_sent_position_cmd{kNaN};
    bool command_initialized{false};
};

class DavinciMoveItAdapterPrivate
{
public:
    DavinciMoveItAdapterPrivate() = default;
    ~DavinciMoveItAdapterPrivate() = default;

    sim::EntityComponentManager * ecm{nullptr};
    std::unique_ptr<gz::transport::Node> gz_node;
    unsigned int update_rate{0u};
    bool is_active{false};

    std::vector<JointData> joints;
};

//------------------------------------------------------------------------------
// Constructor / destructor
//------------------------------------------------------------------------------

DavinciMoveItAdapter::DavinciMoveItAdapter()
    : data_ptr_(std::make_unique<DavinciMoveItAdapterPrivate>())
{
}

DavinciMoveItAdapter::~DavinciMoveItAdapter() = default;

//------------------------------------------------------------------------------
// Gazebo <-> ros2_control initialization
//------------------------------------------------------------------------------

bool DavinciMoveItAdapter::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    unsigned int update_rate)
{
    this->nh_ = model_nh;
    data_ptr_->ecm = &ecm;
    data_ptr_->update_rate = update_rate;
    data_ptr_->gz_node = std::make_unique<gz::transport::Node>();

    if (hardware_info.joints.empty()) {
        RCLCPP_ERROR(this->nh_->get_logger(), "No joints were found in hardware_info.");
        return false;
    }

    // Reserve memory first so that all interface pointers remain stable.
    data_ptr_->joints.clear();
    data_ptr_->joints.reserve(hardware_info.joints.size());

    for (const auto & joint_info : hardware_info.joints) {
        const auto it_joint = joints.find(joint_info.name);
        if (it_joint == joints.end()) {
            RCLCPP_WARN(
                this->nh_->get_logger(),
                "Skipping joint '%s' because it does not exist in the Gazebo model.",
                joint_info.name.c_str());
            continue;
        }

        // Create the joint directly inside the final storage.
        data_ptr_->joints.emplace_back();
        auto & joint = data_ptr_->joints.back();

        joint.name = joint_info.name;
        joint.sim_joint = it_joint->second;

        // Read static joint metadata if available.
        if (const auto * joint_type_comp =
                    ecm.Component<sim::components::JointType>(joint.sim_joint))
        {
            joint.joint_type = joint_type_comp->Data();
        }

        if (const auto * joint_axis_comp =
                    ecm.Component<sim::components::JointAxis>(joint.sim_joint))
        {
            joint.joint_axis = joint_axis_comp->Data();
        }

        // Ensure read components exist.
        if (!ecm.EntityHasComponentType(
                    joint.sim_joint,
                    sim::components::JointPosition().TypeId()))
        {
            ecm.CreateComponent(joint.sim_joint, sim::components::JointPosition());
        }

        if (!ecm.EntityHasComponentType(
                    joint.sim_joint,
                    sim::components::JointVelocity().TypeId()))
        {
            ecm.CreateComponent(joint.sim_joint, sim::components::JointVelocity());
        }

        if (!ecm.EntityHasComponentType(
                    joint.sim_joint,
                    sim::components::JointTransmittedWrench().TypeId()))
        {
            ecm.CreateComponent(joint.sim_joint, sim::components::JointTransmittedWrench());
        }

        RCLCPP_INFO(this->nh_->get_logger(), "Loading joint '%s'", joint.name.c_str());

        //--------------------------------------------------------------------------
        // Parse and store state interfaces
        //--------------------------------------------------------------------------

        double initial_position = kNaN;
        double initial_velocity = kNaN;
        double initial_effort = kNaN;

        for (const auto & state_interface : joint_info.state_interfaces) {
            if (state_interface.name == hardware_interface::HW_IF_POSITION) {
                initial_position = parse_initial_value(
                                       state_interface, this->nh_->get_logger(), joint.name);
                if (std::isfinite(initial_position)) {
                    joint.position = initial_position;
                }
                joint.has_position_state = true;
                continue;
            }

            if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
                initial_velocity = parse_initial_value(
                                       state_interface, this->nh_->get_logger(), joint.name);
                if (std::isfinite(initial_velocity)) {
                    joint.velocity = initial_velocity;
                }
                joint.has_velocity_state = true;
                continue;
            }

            if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
                initial_effort = parse_initial_value(
                                     state_interface, this->nh_->get_logger(), joint.name);
                if (std::isfinite(initial_effort)) {
                    joint.effort = initial_effort;
                }
                joint.has_effort_state = true;
                continue;
            }

            RCLCPP_WARN(
                this->nh_->get_logger(),
                "Unsupported state interface '%s' for joint '%s'.",
                state_interface.name.c_str(),
                joint.name.c_str());
        }

        //--------------------------------------------------------------------------
        // Parse and store command interfaces
        //--------------------------------------------------------------------------

        for (const auto & command_interface : joint_info.command_interfaces) {
            if (command_interface.name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_WARN(
                    this->nh_->get_logger(),
                    "Unsupported command interface '%s' for joint '%s'. "
                    "This adapter only supports position commands.",
                    command_interface.name.c_str(),
                    joint.name.c_str());
                continue;
            }

            joint.gz_command_topic = resolve_command_topic(joint_info, this->nh_->get_logger());
            joint.gz_position_pub =
                data_ptr_->gz_node->Advertise<gz::msgs::Double>(joint.gz_command_topic);
            joint.has_position_publisher = true;
            joint.has_position_command = true;

            joint.position_cmd = std::isfinite(initial_position) ? initial_position : joint.position;

            RCLCPP_INFO(
                this->nh_->get_logger(),
                "Joint '%s' command topic: %s",
                joint.name.c_str(),
                joint.gz_command_topic.c_str());
        }

        //--------------------------------------------------------------------------
        // Apply initial simulated state when provided
        //--------------------------------------------------------------------------

        if (std::isfinite(initial_position)) {
            ecm.CreateComponent(
                joint.sim_joint,
                sim::components::JointPositionReset({initial_position}));
        }

        if (std::isfinite(initial_velocity)) {
            ecm.CreateComponent(
                joint.sim_joint,
                sim::components::JointVelocityReset({initial_velocity}));
        }

        if (std::isfinite(initial_effort)) {
            joint.effort = initial_effort;
        }
    }

    if (data_ptr_->joints.empty()) {
        RCLCPP_ERROR(this->nh_->get_logger(), "No valid joints were configured in the adapter.");
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
// Hardware interface lifecycle
//------------------------------------------------------------------------------

CallbackReturn DavinciMoveItAdapter::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn DavinciMoveItAdapter::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(this->nh_->get_logger(), "DavinciMoveItAdapter configured successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DavinciMoveItAdapter::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    data_ptr_->is_active = true;

    // Initialize commands with current positions to avoid startup jumps.
    for (auto & joint : data_ptr_->joints) {
        if (!joint.has_position_command) {
            continue;
        }

        if (!std::isfinite(joint.position_cmd)) {
            joint.position_cmd = joint.position;
        }

        if (!joint.has_position_publisher) {
            continue;
        }

        gz::msgs::Double msg;
        msg.set_data(joint.position_cmd);
        joint.gz_position_pub.Publish(msg);

        joint.last_sent_position_cmd = joint.position_cmd;
        joint.command_initialized = true;
    }

    RCLCPP_INFO(this->nh_->get_logger(), "DavinciMoveItAdapter activated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DavinciMoveItAdapter::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    data_ptr_->is_active = false;
    RCLCPP_INFO(this->nh_->get_logger(), "DavinciMoveItAdapter deactivated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DavinciMoveItAdapter::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    data_ptr_->is_active = false;
    RCLCPP_INFO(this->nh_->get_logger(), "DavinciMoveItAdapter cleaned up.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DavinciMoveItAdapter::on_shutdown(
    const rclcpp_lifecycle::State & previous_state)
{
    return on_cleanup(previous_state);
}

//------------------------------------------------------------------------------
// ros2_control interfaces
//------------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
DavinciMoveItAdapter::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(data_ptr_->joints.size() * 3);

    for (auto & joint : data_ptr_->joints) {
        if (joint.has_position_state) {
            state_interfaces.emplace_back(
                joint.name,
                hardware_interface::HW_IF_POSITION,
                &joint.position);
        }

        if (joint.has_velocity_state) {
            state_interfaces.emplace_back(
                joint.name,
                hardware_interface::HW_IF_VELOCITY,
                &joint.velocity);
        }

        if (joint.has_effort_state) {
            state_interfaces.emplace_back(
                joint.name,
                hardware_interface::HW_IF_EFFORT,
                &joint.effort);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DavinciMoveItAdapter::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(data_ptr_->joints.size());

    for (auto & joint : data_ptr_->joints) {
        if (joint.has_position_command) {
            command_interfaces.emplace_back(
                joint.name,
                hardware_interface::HW_IF_POSITION,
                &joint.position_cmd);
        }
    }

    return command_interfaces;
}

//------------------------------------------------------------------------------
// Hardware telemetry
//------------------------------------------------------------------------------

hardware_interface::return_type DavinciMoveItAdapter::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    for (auto & joint : data_ptr_->joints) {
        if (joint.sim_joint == sim::kNullEntity) {
            continue;
        }

        if (joint.has_position_state) {
            if (const auto * joint_position =
                        data_ptr_->ecm->Component<sim::components::JointPosition>(joint.sim_joint))
            {
                if (!joint_position->Data().empty()) {
                    joint.position = joint_position->Data()[0];
                }
            }
        }

        if (joint.has_velocity_state) {
            if (const auto * joint_velocity =
                        data_ptr_->ecm->Component<sim::components::JointVelocity>(joint.sim_joint))
            {
                if (!joint_velocity->Data().empty()) {
                    joint.velocity = joint_velocity->Data()[0];
                }
            }
        }

        if (joint.has_effort_state) {
            if (const auto * joint_wrench =
                        data_ptr_->ecm->Component<sim::components::JointTransmittedWrench>(joint.sim_joint))
            {
                joint.effort = compute_scalar_effort(
                                   joint.joint_type,
                                   joint.joint_axis,
                                   joint_wrench->Data());
            }
        }
    }

    return hardware_interface::return_type::OK;
}

//------------------------------------------------------------------------------
// Hardware commands
//------------------------------------------------------------------------------

hardware_interface::return_type DavinciMoveItAdapter::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    if (!data_ptr_->is_active) {
        return hardware_interface::return_type::OK;
    }

    for (auto & joint : data_ptr_->joints) {
        if (!joint.has_position_command || !joint.has_position_publisher) {
            continue;
        }

        const double desired_position = std::isfinite(joint.position_cmd) ?
                                        joint.position_cmd : joint.position;

        const bool should_publish =
            !joint.command_initialized ||
            !std::isfinite(joint.last_sent_position_cmd) ||
            std::abs(desired_position - joint.last_sent_position_cmd) > kCommandEpsilon;

        if (!should_publish) {
            continue;
        }

        gz::msgs::Double msg;
        msg.set_data(desired_position);
        joint.gz_position_pub.Publish(msg);

        joint.last_sent_position_cmd = desired_position;
        joint.command_initialized = true;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace davinci_arm_gazebo_plugins

PLUGINLIB_EXPORT_CLASS(
    davinci_arm_gazebo_plugins::DavinciMoveItAdapter,
    gz_ros2_control::GazeboSimSystemInterface)