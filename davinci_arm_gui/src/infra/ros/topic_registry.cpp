#include "davinci_arm_gui/infra/ros/topic_registry.hpp"

#include <stdexcept>
#include <utility>

namespace davinci_arm::infra::ros {

using davinci_arm::models::CommandType;
using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;

namespace {

std::string getStringEither(rclcpp::Node& node,
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
        node.declare_parameter<std::string>("topic_ns.real", "/arm");
    }
    if (!node.has_parameter("topic_ns.sim")) {
        node.declare_parameter<std::string>("topic_ns.sim", "/arm_sim");
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
                                   "topics.ros.real_joint_states",
                                   "topic.ros.real_joint_states",
                                   "/real/joint_states"));

    sim_joint_states_topic_ = normalizeTopic_(getStringEither(
                                  node,
                                  "topics.ros.sim_joint_states",
                                  "topic.ros.sim_joint_states",
                                  "/joint_states"));

    controller_state_topic_ = normalizeTopic_(getStringEither(
                                  node,
                                  "topics.ros.controller_state",
                                  "topic.ros.controller_state",
                                  "/davinci_arm_controller/controller_state"));

    display_planned_path_topic_ = normalizeTopic_(getStringEither(
                                      node,
                                      "topics.ros.display_planned_path",
                                      "topic.ros.display_planned_path",
                                      "/display_planned_path"));

    planning_scene_topic_ = normalizeTopic_(getStringEither(
            node,
            "topics.ros.planning_scene",
            "topic.ros.planning_scene",
            "/planning_scene"));

    trajectory_execution_event_topic_ = normalizeTopic_(getStringEither(
                                            node,
                                            "topics.ros.trajectory_execution_event",
                                            "topic.ros.trajectory_execution_event",
                                            "/trajectory_execution_event"));

    real_joint_trajectory_cmd_topic_ = normalizeTopic_(getStringEither(
                                           node,
                                           "topics.ros.real_joint_trajectory_command",
                                           "topic.ros.real_joint_trajectory_command",
                                           "/real/davinci_arm_controller/joint_trajectory"));

    sim_joint_trajectory_cmd_topic_ = normalizeTopic_(getStringEither(
                                          node,
                                          "topics.ros.sim_joint_trajectory_command",
                                          "topic.ros.sim_joint_trajectory_command",
                                          "/davinci_arm_controller/joint_trajectory"));
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

const std::string& TopicRegistry::realJointTrajectoryCommandTopic() const noexcept
{
    return real_joint_trajectory_cmd_topic_;
}

const std::string& TopicRegistry::simJointTrajectoryCommandTopic() const noexcept
{
    return sim_joint_trajectory_cmd_topic_;
}

}  // namespace davinci_arm::infra::ros
