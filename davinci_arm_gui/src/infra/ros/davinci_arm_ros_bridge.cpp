#include "davinci_arm_gui/infra/ros/davinci_arm_ros_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string_view>
#include <utility>

#include "davinci_arm_gui/core/models/command_type.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

namespace davinci_arm::infra::ros {

namespace {

using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;
using TelemetrySample = davinci_arm::models::TelemetrySample;

constexpr std::array<Domain, 2> kDomains {Domain::Real, Domain::Sim};

std::uint32_t toNanosecondsRemainder(double seconds)
{
    const auto whole_seconds = static_cast<std::int64_t>(std::floor(seconds));
    const auto remainder = seconds - static_cast<double>(whole_seconds);
    return static_cast<std::uint32_t>(std::llround(remainder * 1e9));
}

std::int32_t toSecondsFloor(double seconds)
{
    return static_cast<std::int32_t>(std::floor(seconds));
}

template<typename T>
void setDomainIfPresent(T& sample, Domain domain)
{
    if constexpr (requires { sample.domain = domain; }) {
        sample.domain = domain;
    }
}

template<typename T>
void setSignalIfPresent(T& sample, TelemetrySignalType signal)
{
    if constexpr (requires { sample.signal = signal; }) {
        sample.signal = signal;
    } else if constexpr (requires { sample.signal_type = signal; }) {
        sample.signal_type = signal;
    } else if constexpr (requires { sample.telemetry_signal = signal; }) {
        sample.telemetry_signal = signal;
    }
}

template<typename T>
void setValueIfPresent(T& sample, double value)
{
    if constexpr (requires { sample.value = value; }) {
        sample.value = value;
    } else if constexpr (requires { sample.y = value; }) {
        sample.y = value;
    } else if constexpr (requires { sample.scalar = value; }) {
        sample.scalar = value;
    }
}

template<typename T>
void setTimeIfPresent(T& sample, double time_sec)
{
    if constexpr (requires { sample.time_sec = time_sec; }) {
        sample.time_sec = time_sec;
    } else if constexpr (requires { sample.timestamp_sec = time_sec; }) {
        sample.timestamp_sec = time_sec;
    } else if constexpr (requires { sample.t_sec = time_sec; }) {
        sample.t_sec = time_sec;
    } else if constexpr (requires { sample.t = time_sec; }) {
        sample.t = time_sec;
    }

    if constexpr (requires { sample.stamp.sec = 0; sample.stamp.nanosec = 0U; }) {
        sample.stamp.sec = toSecondsFloor(time_sec);
        sample.stamp.nanosec = toNanosecondsRemainder(time_sec);
    }
}

template<typename T>
void setJointIfPresent(T& sample, const std::string& joint_name)
{
    if constexpr (requires { sample.joint_name = joint_name; }) {
        sample.joint_name = joint_name;
    } else if constexpr (requires { sample.label = joint_name; }) {
        sample.label = joint_name;
    } else if constexpr (requires { sample.series = joint_name; }) {
        sample.series = joint_name;
    } else if constexpr (requires { sample.channel = joint_name; }) {
        sample.channel = joint_name;
    }
}

TelemetrySample makeTelemetrySample(Domain domain,
                                    TelemetrySignalType signal,
                                    const std::string& joint_name,
                                    double value,
                                    double time_sec)
{
    TelemetrySample sample {};
    setDomainIfPresent(sample, domain);
    setSignalIfPresent(sample, signal);
    setJointIfPresent(sample, joint_name);
    setValueIfPresent(sample, value);
    setTimeIfPresent(sample, time_sec);
    return sample;
}

const char* toCString(Domain domain)
{
    return (domain == Domain::Real) ? "real" : "sim";
}

}  // namespace

DavinciArmRosBridge::DavinciArmRosBridge(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<const TopicRegistry> topics,
    QObject* parent)
    : QObject(parent),
      node_(std::move(node)),
      topics_(std::move(topics))
{
    setupPublishers_();
    setupSubscribers_();
    setupConnectionTimer_();
}

void DavinciArmRosBridge::setupPublishers_()
{
    const auto qos = rclcpp::QoS(10).reliable();

    pub_ref_angle_real_ = node_->create_publisher<std_msgs::msg::Float64>(
                              topics_->topic(Domain::Real, davinci_arm::models::CommandType::RefAngle), qos);
    pub_pwm_real_ = node_->create_publisher<std_msgs::msg::UInt16>(
                        topics_->topic(Domain::Real, davinci_arm::models::CommandType::PwmCmd), qos);
    pub_auto_real_ = node_->create_publisher<std_msgs::msg::Bool>(
                         topics_->topic(Domain::Real, davinci_arm::models::CommandType::AutoMode), qos);

    pub_ref_angle_sim_ = node_->create_publisher<std_msgs::msg::Float64>(
                             topics_->topic(Domain::Sim, davinci_arm::models::CommandType::RefAngle), qos);
    pub_pwm_sim_ = node_->create_publisher<std_msgs::msg::UInt16>(
                       topics_->topic(Domain::Sim, davinci_arm::models::CommandType::PwmCmd), qos);
    pub_auto_sim_ = node_->create_publisher<std_msgs::msg::Bool>(
                        topics_->topic(Domain::Sim, davinci_arm::models::CommandType::AutoMode), qos);

    pub_real_joint_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                                     topics_->realJointTrajectoryCommandTopic(), qos);
    pub_sim_joint_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                                    topics_->simJointTrajectoryCommandTopic(), qos);
}

void DavinciArmRosBridge::setupSubscribers_()
{
    subscribe_<sensor_msgs::msg::JointState>(
        topics_->simJointStatesTopic(),
        rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState& msg) {
        onJointState_(Domain::Sim, msg);
    });

    subscribe_<sensor_msgs::msg::JointState>(
        topics_->realJointStatesTopic(),
        rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState& msg) {
        onJointState_(Domain::Real, msg);
    });

    subscribe_<control_msgs::msg::JointTrajectoryControllerState>(
        topics_->controllerStateTopic(),
        rclcpp::QoS(20).reliable(),
    [this](const control_msgs::msg::JointTrajectoryControllerState& msg) {
        onControllerState_(msg);
    });

    subscribe_<std_msgs::msg::String>(
        topics_->trajectoryExecutionEventTopic(),
        rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::String& msg) {
        onTrajectoryExecutionEvent_(msg);
    });
}

void DavinciArmRosBridge::setupConnectionTimer_()
{
    conn_timer_ = node_->create_wall_timer(
                      std::chrono::milliseconds(200),
    [this]() {
        updateConnection_();
    });
}

void DavinciArmRosBridge::onJointState_(Domain domain, const sensor_msgs::msg::JointState& msg)
{
    const auto now = std::chrono::steady_clock::now();

    std::scoped_lock lock(state_mtx_);
    auto& snapshot = (domain == Domain::Real) ? real_state_ : sim_state_;

    snapshot.valid = true;
    snapshot.last_rx = now;

    for (std::size_t i = 0; i < msg.name.size(); ++i) {
        const auto& joint_name = msg.name[i];
        const auto it = std::find(topics_->jointNames().begin(), topics_->jointNames().end(), joint_name);
        if (it == topics_->jointNames().end()) {
            continue;
        }

        if (i < msg.position.size()) {
            snapshot.position_by_joint[joint_name] = msg.position[i];
        }
        if (i < msg.velocity.size()) {
            snapshot.velocity_by_joint[joint_name] = msg.velocity[i];
        }

        const auto pos_it = snapshot.position_by_joint.find(joint_name);
        if (pos_it != snapshot.position_by_joint.end()) {
            const auto vel_it = snapshot.velocity_by_joint.find(joint_name);
            const std::optional<double> vel =
                (vel_it != snapshot.velocity_by_joint.end())
                ? std::optional<double>(vel_it->second)
                : std::nullopt;

            emitJointTelemetry_(domain, joint_name, pos_it->second, vel);
        }
    }

    touchRx_(domain);
}

void DavinciArmRosBridge::onControllerState_(const control_msgs::msg::JointTrajectoryControllerState& msg)
{
    std::scoped_lock lock(state_mtx_);

    const auto joint_count = std::min(msg.joint_names.size(), msg.reference.positions.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
        const auto& joint_name = msg.joint_names[i];
        const auto it = std::find(topics_->jointNames().begin(), topics_->jointNames().end(), joint_name);
        if (it == topics_->jointNames().end()) {
            continue;
        }

        ref_position_by_joint_[joint_name] = msg.reference.positions[i];
        emitReferenceTelemetry_(joint_name, msg.reference.positions[i]);
    }
}

void DavinciArmRosBridge::onTrajectoryExecutionEvent_(const std_msgs::msg::String& msg)
{
    RCLCPP_INFO(node_->get_logger(), "trajectory_execution_event: %s", msg.data.c_str());
}

void DavinciArmRosBridge::emitJointTelemetry_(
    Domain domain,
    const std::string& joint_name,
    double position_rad,
    const std::optional<double>& velocity_rad_s)
{
    const double time_sec = node_->now().seconds();
    emit telemetryUpdated(makeTelemetrySample(domain, TelemetrySignalType::Angle, joint_name, position_rad, time_sec));

    if (velocity_rad_s.has_value()) {
        emit telemetryUpdated(makeTelemetrySample(domain, TelemetrySignalType::MotorSpeed, joint_name, *velocity_rad_s, time_sec));
    }

    const auto ref_it = ref_position_by_joint_.find(joint_name);
    if (ref_it != ref_position_by_joint_.end()) {
        emit telemetryUpdated(makeTelemetrySample(domain, TelemetrySignalType::AngleRef, joint_name, ref_it->second, time_sec));
    }
}

void DavinciArmRosBridge::emitReferenceTelemetry_(const std::string& joint_name, double ref_position_rad)
{
    const double time_sec = node_->now().seconds();
    emit telemetryUpdated(makeTelemetrySample(Domain::Real, TelemetrySignalType::AngleRef, joint_name, ref_position_rad, time_sec));
    emit telemetryUpdated(makeTelemetrySample(Domain::Sim, TelemetrySignalType::AngleRef, joint_name, ref_position_rad, time_sec));
}

void DavinciArmRosBridge::touchRx_(Domain domain)
{
    const bool previous_real = real_live_;
    const bool previous_sim = sim_live_;

    if (domain == Domain::Real) {
        real_live_ = true;
    } else {
        sim_live_ = true;
    }

    if (previous_real != real_live_) {
        emit streamLiveChanged(Domain::Real, real_live_);
    }
    if (previous_sim != sim_live_) {
        emit streamLiveChanged(Domain::Sim, sim_live_);
    }

    const bool new_connected = real_live_ || sim_live_;
    if (connected_ != new_connected) {
        connected_ = new_connected;
        emit connectionChanged(connected_);
    }
}

void DavinciArmRosBridge::updateConnection_()
{
    const auto now = std::chrono::steady_clock::now();

    bool next_real_live = false;
    bool next_sim_live = false;

    {
        std::scoped_lock lock(state_mtx_);
        if (real_state_.valid) {
            next_real_live = (now - real_state_.last_rx) <= kConnectionTimeout;
        }
        if (sim_state_.valid) {
            next_sim_live = (now - sim_state_.last_rx) <= kConnectionTimeout;
        }
    }

    if (real_live_ != next_real_live) {
        real_live_ = next_real_live;
        emit streamLiveChanged(Domain::Real, real_live_);
    }
    if (sim_live_ != next_sim_live) {
        sim_live_ = next_sim_live;
        emit streamLiveChanged(Domain::Sim, sim_live_);
    }

    const bool next_connected = real_live_ || sim_live_;
    if (connected_ != next_connected) {
        connected_ = next_connected;
        emit connectionChanged(connected_);
    }
}

trajectory_msgs::msg::JointTrajectory DavinciArmRosBridge::buildSinglePointTrajectory_(
    Domain domain,
    const std::vector<double>& positions_rad,
    const std::vector<double>& velocities_rad_s,
    const std::vector<double>& accelerations_rad_s2,
    double time_from_start_s) const
{
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = topics_->jointNames();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions_rad;

    if (!velocities_rad_s.empty()) {
        point.velocities = velocities_rad_s;
    }
    if (!accelerations_rad_s2.empty()) {
        point.accelerations = accelerations_rad_s2;
    }

    if (time_from_start_s < 0.0) {
        time_from_start_s = 0.0;
    }
    point.time_from_start.sec = toSecondsFloor(time_from_start_s);
    point.time_from_start.nanosec = toNanosecondsRemainder(time_from_start_s);

    msg.points.push_back(std::move(point));
    (void)domain;
    return msg;
}

std::vector<double> DavinciArmRosBridge::currentPositionsOrZeros_(Domain domain) const
{
    std::vector<double> positions(topics_->jointNames().size(), 0.0);

    std::scoped_lock lock(state_mtx_);
    const auto& snapshot = (domain == Domain::Real) ? real_state_ : sim_state_;

    for (std::size_t i = 0; i < topics_->jointNames().size(); ++i) {
        const auto& joint_name = topics_->jointNames()[i];
        const auto it = snapshot.position_by_joint.find(joint_name);
        if (it != snapshot.position_by_joint.end()) {
            positions[i] = it->second;
        }
    }

    return positions;
}

void DavinciArmRosBridge::sendJointTrajectory(
    Domain domain,
    const std::vector<double>& positions_rad,
    const std::vector<double>& velocities_rad_s,
    const std::vector<double>& accelerations_rad_s2,
    double time_from_start_s)
{
    if (positions_rad.size() != topics_->jointNames().size()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "sendJointTrajectory(%s): expected %zu joint positions but received %zu.",
            toCString(domain),
            topics_->jointNames().size(),
            positions_rad.size());
        return;
    }

    const auto msg = buildSinglePointTrajectory_(
                         domain,
                         positions_rad,
                         velocities_rad_s,
                         accelerations_rad_s2,
                         time_from_start_s);

    if (domain == Domain::Real) {
        publish_(pub_real_joint_trajectory_, msg);
    } else {
        publish_(pub_sim_joint_trajectory_, msg);
    }
}

void DavinciArmRosBridge::sendHoldPosition(Domain domain)
{
    auto positions = currentPositionsOrZeros_(domain);
    std::vector<double> velocities(positions.size(), 0.0);
    std::vector<double> accelerations(positions.size(), 0.0);
    sendJointTrajectory(domain, positions, velocities, accelerations, 0.0);
}

void DavinciArmRosBridge::sendRefAngle(double rad)
{
    for (const auto domain : kDomains) {
        sendAngleReference(domain, rad);
    }
}

void DavinciArmRosBridge::sendPwm(std::uint16_t us)
{
    for (const auto domain : kDomains) {
        sendPwmCommand(domain, us);
    }
}

void DavinciArmRosBridge::sendAutoMode(bool enabled)
{
    for (const auto domain : kDomains) {
        sendAutoModeCommand(domain, enabled);
    }
}

void DavinciArmRosBridge::sendStop()
{
    for (const auto domain : kDomains) {
        sendAutoModeCommand(domain, false);
        sendPwmCommand(domain, static_cast<std::uint16_t>(0));
        sendHoldPosition(domain);
    }
}

void DavinciArmRosBridge::sendAngleReference(Domain domain, double rad)
{
    std_msgs::msg::Float64 scalar_msg;
    scalar_msg.data = rad;

    if (domain == Domain::Real) {
        publish_(pub_ref_angle_real_, scalar_msg);
    } else {
        publish_(pub_ref_angle_sim_, scalar_msg);
    }

    auto positions = currentPositionsOrZeros_(domain);
    if (!positions.empty()) {
        positions.front() = rad;
        sendJointTrajectory(domain, positions);
    }
}

void DavinciArmRosBridge::sendPwmCommand(Domain domain, std::uint16_t us)
{
    std_msgs::msg::UInt16 msg;
    msg.data = us;

    if (domain == Domain::Real) {
        publish_(pub_pwm_real_, msg);
    } else {
        publish_(pub_pwm_sim_, msg);
    }
}

void DavinciArmRosBridge::sendAutoModeCommand(Domain domain, bool enabled)
{
    std_msgs::msg::Bool msg;
    msg.data = enabled;

    if (domain == Domain::Real) {
        publish_(pub_auto_real_, msg);
    } else {
        publish_(pub_auto_sim_, msg);
    }
}

bool DavinciArmRosBridge::isConnected() const noexcept
{
    return connected_;
}

bool DavinciArmRosBridge::isLive(Domain domain) const noexcept
{
    return (domain == Domain::Real) ? real_live_ : sim_live_;
}

}  // namespace davinci_arm::infra::ros
