#include "davinci_arm_gui/infra/ros/topic_registry.hpp"

#include <stdexcept>
#include <utility>

namespace davinci_arm::infra::ros {

using davinci_arm::models::CommandType;
using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;

namespace {

std::string getStringEither(
    rclcpp::Node& node,
    const std::string& primary,
    const std::string& legacy,
    const std::string& fallback)
{
    if (!node.has_parameter(primary)) {
        node.declare_parameter<std::string>(primary, fallback);
    }
    if (!node.has_parameter(legacy)) {
        node.declare_parameter<std::string>(legacy, fallback);
    }

    const auto primary_value = node.get_parameter(primary).as_string();
    const auto legacy_value = node.get_parameter(legacy).as_string();

    if (!primary_value.empty() && primary_value != fallback) {
        return primary_value;
    }
    if (!legacy_value.empty() && legacy_value != fallback) {
        return legacy_value;
    }
    return fallback;
}

std::vector<std::string> getStringArrayEither(
    rclcpp::Node& node,
    const std::string& primary,
    const std::string& legacy,
    const std::vector<std::string>& fallback)
{
    if (!node.has_parameter(primary)) {
        node.declare_parameter<std::vector<std::string>>(primary, fallback);
    }
    if (!node.has_parameter(legacy)) {
        node.declare_parameter<std::vector<std::string>>(legacy, fallback);
    }

    const auto primary_value = node.get_parameter(primary).as_string_array();
    const auto legacy_value = node.get_parameter(legacy).as_string_array();

    if (!primary_value.empty() && primary_value != fallback) {
        return primary_value;
    }
    if (!legacy_value.empty() && legacy_value != fallback) {
        return legacy_value;
    }
    return fallback;
}

}  // namespace

TopicRegistry::TopicRegistry(rclcpp::Node& node)
{
    if (!node.has_parameter("topic_ns.real")) {
        node.declare_parameter<std::string>("topic_ns.real", "/davinci_arm/real");
    }
    if (!node.has_parameter("topic_ns.sim")) {
        node.declare_parameter<std::string>("topic_ns.sim", "/davinci_arm");
    }

    real_ns_ = normalizeNs_(node.get_parameter("topic_ns.real").as_string());
    sim_ns_ = normalizeNs_(node.get_parameter("topic_ns.sim").as_string());

    joint_names_ = getStringArrayEither(
                       node,
                       "joints.names",
                       "joint_names",
    {
        "shoulder_link_joint",
        "bicep_link_joint",
        "arm_link_joint",
        "wrist_link_joint",
        "end_effector_link_joint",
    });

    angle_ = getStringEither(node, "topics.signals.angle", "topic.signals.angle", "angle_rad");
    motor_speed_ = getStringEither(node, "topics.signals.motor_speed", "topic.signals.motor_speed", "motor_vel_rad");
    pwm_feedback_ = getStringEither(node, "topics.signals.pwm_feedback", "topic.signals.pwm_feedback", "esc/pwm_us_fb");
    angle_ref_ = getStringEither(node, "topics.signals.angle_ref", "topic.signals.angle_ref", "ref_angle_rad");

    ref_angle_ = getStringEither(node, "topics.commands.ref_angle", "topic.commands.ref_angle", "ref_angle_rad");
    pwm_cmd_ = getStringEither(node, "topics.commands.pwm_cmd", "topic.commands.pwm_cmd", "esc/pwm_us");
    auto_mode_ = getStringEither(node, "topics.commands.auto_mode", "topic.commands.auto_mode", "esc/auto_mode");

    real_joint_states_topic_ = normalizeTopic_(getStringEither(
                                   node,
                                   "topics.telemetry.real_joint_states",
                                   "topics.ros.real_joint_states",
                                   "/davinci_arm/real/joint_states"));

    sim_joint_states_topic_ = normalizeTopic_(getStringEither(
                                  node,
                                  "topics.telemetry.sim_joint_states",
                                  "topics.ros.sim_joint_states",
                                  "/joint_states"));

    controller_state_topic_ = normalizeTopic_(getStringEither(
                                  node,
                                  "topics.telemetry.controller_state",
                                  "topics.ros.controller_state",
                                  "/davinci_arm_controller/controller_state"));

    display_planned_path_topic_ = normalizeTopic_(getStringEither(
                                      node,
                                      "topics.telemetry.display_planned_path",
                                      "topics.ros.display_planned_path",
                                      "/display_planned_path"));

    planning_scene_topic_ = normalizeTopic_(getStringEither(
            node,
            "topics.telemetry.planning_scene",
            "topics.ros.planning_scene",
            "/planning_scene"));

    trajectory_execution_event_topic_ = normalizeTopic_(getStringEither(
                                            node,
                                            "topics.telemetry.trajectory_execution_event",
                                            "topics.ros.trajectory_execution_event",
                                            "/trajectory_execution_event"));

    for (const auto& joint_name : joint_names_) {
        const auto leaf = defaultCommandLeafFromJoint_(joint_name);

        const auto sim_param = "topics.commands.per_joint.sim." + joint_name;
        const auto real_param = "topics.commands.per_joint.real." + joint_name;

        if (!node.has_parameter(sim_param)) {
            node.declare_parameter<std::string>(sim_param, join_(sim_ns_, leaf));
        }
        if (!node.has_parameter(real_param)) {
            node.declare_parameter<std::string>(real_param, join_(real_ns_, leaf));
        }

        sim_joint_command_topics_.emplace(
            joint_name,
            normalizeTopic_(node.get_parameter(sim_param).as_string()));
        real_joint_command_topics_.emplace(
            joint_name,
            normalizeTopic_(node.get_parameter(real_param).as_string()));
    }
}

std::string TopicRegistry::normalizeNs_(std::string ns)
{
    if (ns.empty()) {
        return ns;
    }
    if (ns.front() != '/') {
        ns.insert(ns.begin(), '/');
    }
    while (ns.size() > 1 && ns.back() == '/') {
        ns.pop_back();
    }
    return ns;
}

std::string TopicRegistry::normalizeTopic_(std::string topic)
{
    if (topic.empty()) {
        return topic;
    }
    if (topic.front() != '/') {
        topic.insert(topic.begin(), '/');
    }
    return topic;
}

std::string TopicRegistry::join_(const std::string& ns, const std::string& name)
{
    if (name.empty()) {
        return ns;
    }
    if (name.front() == '/') {
        return normalizeTopic_(name);
    }
    if (ns.empty()) {
        return "/" + name;
    }
    return ns + "/" + name;
}

std::string TopicRegistry::stripJointSuffix_(std::string joint_name)
{
    static constexpr const char* kLinkJointSuffix = "_link_joint";
    static constexpr const char* kJointSuffix = "_joint";

    if (joint_name.size() >= std::char_traits<char>::length(kLinkJointSuffix) &&
            joint_name.compare(
                joint_name.size() - std::char_traits<char>::length(kLinkJointSuffix),
                std::char_traits<char>::length(kLinkJointSuffix),
                kLinkJointSuffix) == 0)
    {
        joint_name.erase(
            joint_name.size() - std::char_traits<char>::length(kLinkJointSuffix));
        return joint_name;
    }

    if (joint_name.size() >= std::char_traits<char>::length(kJointSuffix) &&
            joint_name.compare(
                joint_name.size() - std::char_traits<char>::length(kJointSuffix),
                std::char_traits<char>::length(kJointSuffix),
                kJointSuffix) == 0)
    {
        joint_name.erase(
            joint_name.size() - std::char_traits<char>::length(kJointSuffix));
    }

    return joint_name;
}

std::string TopicRegistry::defaultCommandLeafFromJoint_(const std::string& joint_name)
{
    return stripJointSuffix_(joint_name) + "_cmd_pos";
}

std::string TopicRegistry::topic(Domain domain, TelemetrySignalType signal) const
{
    const auto& ns = (domain == Domain::Real) ? real_ns_ : sim_ns_;

    switch (signal) {
    case TelemetrySignalType::Angle:
        return join_(ns, angle_);
    case TelemetrySignalType::MotorSpeed:
        return join_(ns, motor_speed_);
    case TelemetrySignalType::PwmFeedback:
        return join_(ns, pwm_feedback_);
    case TelemetrySignalType::AngleRef:
        return join_(ns, angle_ref_);
    default:
        break;
    }

    throw std::runtime_error("TopicRegistry::topic(): unsupported TelemetrySignalType");
}

std::string TopicRegistry::topic(Domain domain, CommandType command) const
{
    const auto& ns = (domain == Domain::Real) ? real_ns_ : sim_ns_;

    switch (command) {
    case CommandType::RefAngle:
        return join_(ns, ref_angle_);
    case CommandType::PwmCmd:
        return join_(ns, pwm_cmd_);
    case CommandType::AutoMode:
        return join_(ns, auto_mode_);
    default:
        break;
    }

    throw std::runtime_error("TopicRegistry::topic(): unsupported CommandType");
}

std::string TopicRegistry::jointPositionCommandTopic(
    Domain domain,
    const std::string& joint_name) const
{
    const auto& map = (domain == Domain::Real) ? real_joint_command_topics_ : sim_joint_command_topics_;
    const auto it = map.find(joint_name);
    if (it == map.end()) {
        throw std::runtime_error(
            "TopicRegistry::jointPositionCommandTopic(): unknown joint '" + joint_name + "'");
    }
    return it->second;
}

const std::vector<std::string>& TopicRegistry::jointNames() const noexcept
{
    return joint_names_;
}

const std::string& TopicRegistry::realNamespace() const noexcept
{
    return real_ns_;
}

const std::string& TopicRegistry::simNamespace() const noexcept
{
    return sim_ns_;
}

const std::string& TopicRegistry::realJointStatesTopic() const noexcept
{
    return real_joint_states_topic_;
}

const std::string& TopicRegistry::simJointStatesTopic() const noexcept
{
    return sim_joint_states_topic_;
}

const std::string& TopicRegistry::controllerStateTopic() const noexcept
{
    return controller_state_topic_;
}

const std::string& TopicRegistry::displayPlannedPathTopic() const noexcept
{
    return display_planned_path_topic_;
}

const std::string& TopicRegistry::planningSceneTopic() const noexcept
{
    return planning_scene_topic_;
}

const std::string& TopicRegistry::trajectoryExecutionEventTopic() const noexcept
{
    return trajectory_execution_event_topic_;
}

}  // namespace davinci_arm::infra::ros
