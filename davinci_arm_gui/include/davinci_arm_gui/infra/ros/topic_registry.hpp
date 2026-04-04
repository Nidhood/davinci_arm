#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "davinci_arm_gui/core/models/command_type.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

namespace davinci_arm::infra::ros {

class TopicRegistry final {
public:
    explicit TopicRegistry(rclcpp::Node& node);

    [[nodiscard]] std::string topic(
        davinci_arm::models::Domain domain,
        davinci_arm::models::TelemetrySignalType signal) const;

    [[nodiscard]] std::string topic(
        davinci_arm::models::Domain domain,
        davinci_arm::models::CommandType command) const;

    [[nodiscard]] std::string jointPositionCommandTopic(
        davinci_arm::models::Domain domain,
        const std::string& joint_name) const;

    [[nodiscard]] const std::vector<std::string>& jointNames() const noexcept;

    [[nodiscard]] const std::string& realNamespace() const noexcept;
    [[nodiscard]] const std::string& simNamespace() const noexcept;

    [[nodiscard]] const std::string& realJointStatesTopic() const noexcept;
    [[nodiscard]] const std::string& simJointStatesTopic() const noexcept;
    [[nodiscard]] const std::string& controllerStateTopic() const noexcept;
    [[nodiscard]] const std::string& displayPlannedPathTopic() const noexcept;
    [[nodiscard]] const std::string& planningSceneTopic() const noexcept;
    [[nodiscard]] const std::string& trajectoryExecutionEventTopic() const noexcept;

private:
    static std::string join_(const std::string& ns, const std::string& name);
    static std::string normalizeNs_(std::string ns);
    static std::string normalizeTopic_(std::string topic);
    static std::string stripJointSuffix_(std::string joint_name);
    static std::string defaultCommandLeafFromJoint_(const std::string& joint_name);

    std::string real_ns_;
    std::string sim_ns_;

    std::vector<std::string> joint_names_;

    std::string angle_;
    std::string motor_speed_;
    std::string pwm_feedback_;
    std::string angle_ref_;

    std::string ref_angle_;
    std::string pwm_cmd_;
    std::string auto_mode_;

    std::string real_joint_states_topic_;
    std::string sim_joint_states_topic_;
    std::string controller_state_topic_;
    std::string display_planned_path_topic_;
    std::string planning_scene_topic_;
    std::string trajectory_execution_event_topic_;

    std::unordered_map<std::string, std::string> sim_joint_command_topics_;
    std::unordered_map<std::string, std::string> real_joint_command_topics_;
};

}  // namespace davinci_arm::infra::ros
