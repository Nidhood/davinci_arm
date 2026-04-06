#pragma once

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/infra/ros/topic_registry.hpp"

#include <QObject>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace davinci_arm::infra::ros {

class DavinciArmRosBridge final : public QObject {
    Q_OBJECT

public:
    explicit DavinciArmRosBridge(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<const TopicRegistry> topics,
        QObject* parent = nullptr);

    ~DavinciArmRosBridge() override = default;

    void sendRefAngle(double rad);
    void sendPwm(std::uint16_t us);
    void sendAutoMode(bool enabled);
    void sendStop();

    void sendAngleReference(davinci_arm::models::Domain domain, double rad);
    void sendPwmCommand(davinci_arm::models::Domain domain, std::uint16_t us);
    void sendAutoModeCommand(davinci_arm::models::Domain domain, bool enabled);

    void sendJointTrajectory(
        davinci_arm::models::Domain domain,
        const std::vector<double>& positions_rad,
        const std::vector<double>& velocities_rad_s = {},
        const std::vector<double>& accelerations_rad_s2 = {},
        double time_from_start_s = 0.0);

    void sendHoldPosition(davinci_arm::models::Domain domain);

    [[nodiscard]] bool isConnected() const noexcept;
    [[nodiscard]] bool isLive(davinci_arm::models::Domain domain) const noexcept;

signals:
    void telemetryUpdated(davinci_arm::models::TelemetrySample sample);
    void connectionChanged(bool connected);
    void streamLiveChanged(davinci_arm::models::Domain domain, bool live);

private:
    struct JointSnapshot {
        std::unordered_map<std::string, double> position_by_joint;
        std::unordered_map<std::string, double> velocity_by_joint;
        std::chrono::steady_clock::time_point last_rx{};
        bool valid{false};
    };

    using TimePoint = std::chrono::steady_clock::time_point;

    void setupPublishers_();
    void setupSubscribers_();
    void setupConnectionTimer_();

    void onJointState_(
        davinci_arm::models::Domain domain,
        const sensor_msgs::msg::JointState& msg);
    void onControllerState_(
        const control_msgs::msg::JointTrajectoryControllerState& msg);
    void onTrajectoryExecutionEvent_(const std_msgs::msg::String& msg);

    void emitJointTelemetry_(
        davinci_arm::models::Domain domain,
        const std::string& joint_name,
        double position_rad,
        const std::optional<double>& velocity_rad_s,
        double time_sec);

    void emitReferenceTelemetry_(const std::string& joint_name, double ref_position_rad, double time_sec);

    [[nodiscard]] bool shouldEmitJointTelemetry_(
        davinci_arm::models::Domain domain,
        const std::string& joint_name,
        TimePoint now);

    void touchRx_(davinci_arm::models::Domain domain);
    void updateConnection_();

    std::vector<double> currentPositionsOrZeros_(davinci_arm::models::Domain domain) const;
    [[nodiscard]] bool isEstopLatched_() const;

    template<typename MsgT>
    void subscribe_(
        const std::string& topic,
        const rclcpp::QoS& qos,
        std::function<void(const MsgT&)> callback);

    template<typename MsgT, typename PubT>
    void publish_(const PubT& pub, const MsgT& msg);

private:
    static constexpr std::chrono::milliseconds kConnectionTimeout{2000};
    static constexpr std::chrono::milliseconds kUiEmitPeriod{25};
    static constexpr std::chrono::milliseconds kEstopHoldoff{500};

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<const TopicRegistry> topics_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_angle_real_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_pwm_real_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_auto_real_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_angle_sim_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_pwm_sim_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_auto_sim_;

    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    pub_joint_pos_real_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    pub_joint_pos_sim_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr conn_timer_;

    mutable std::mutex state_mtx_;
    JointSnapshot real_state_;
    JointSnapshot sim_state_;
    std::unordered_map<std::string, double> ref_position_by_joint_;
    TimePoint estop_latched_until_{};

    mutable std::mutex emit_mtx_;
    std::unordered_map<std::string, TimePoint> last_emit_real_;
    std::unordered_map<std::string, TimePoint> last_emit_sim_;

    bool connected_{false};
    bool real_live_{false};
    bool sim_live_{false};
};

template<typename MsgT>
void DavinciArmRosBridge::subscribe_(
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::function<void(const MsgT&)> callback)
{
    auto sub = node_->create_subscription<MsgT>(
                   topic,
                   qos,
    [cb = std::move(callback)](const typename MsgT::SharedPtr msg) {
        cb(*msg);
    });

    subscriptions_.push_back(sub);
}

template<typename MsgT, typename PubT>
void DavinciArmRosBridge::publish_(const PubT& pub, const MsgT& msg)
{
    if (pub) {
        pub->publish(msg);
    }
}

}  // namespace davinci_arm::infra::ros