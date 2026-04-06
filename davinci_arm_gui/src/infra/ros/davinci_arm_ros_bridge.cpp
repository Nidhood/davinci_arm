#include "davinci_arm_gui/infra/ros/davinci_arm_ros_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <type_traits>
#include <utility>

#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

namespace davinci_arm::infra::ros {

namespace {

using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;
using TelemetrySample = davinci_arm::models::TelemetrySample;
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

constexpr std::array<Domain, 2> kCommandDomains{Domain::Real, Domain::Sim};

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
void setJointIfPresent(T& sample, const std::string& joint_name)
{
    if constexpr (requires { sample.joint_name = joint_name; }) {
        sample.joint_name = joint_name;
    }
    if constexpr (requires { sample.label = joint_name; }) {
        sample.label = joint_name;
    }
    if constexpr (requires { sample.series = joint_name; }) {
        sample.series = joint_name;
    }
    if constexpr (requires { sample.channel = joint_name; }) {
        sample.channel = joint_name;
    }
}

template<typename T>
void setNumericFieldIfPresent(T& sample, TelemetrySignalType signal, double value)
{
    if constexpr (requires { sample.value = value; }) {
        sample.value = value;
    }
    if constexpr (requires { sample.y = value; }) {
        sample.y = value;
    }
    if constexpr (requires { sample.scalar = value; }) {
        sample.scalar = value;
    }

    if (signal == TelemetrySignalType::Angle) {
        if constexpr (requires { sample.arm_angle_rad = value; }) {
            sample.arm_angle_rad = value;
        }
    } else if (signal == TelemetrySignalType::MotorSpeed) {
        if constexpr (requires { sample.motor_speed_rad_s = value; }) {
            sample.motor_speed_rad_s = value;
        }
    } else if (signal == TelemetrySignalType::AngleRef) {
        if constexpr (requires { sample.ref_angle_rad = value; }) {
            sample.ref_angle_rad = value;
        }
    } else if (signal == TelemetrySignalType::PwmFeedback) {
        if constexpr (requires { sample.pwm_us = static_cast<std::uint16_t>(0); }) {
            sample.pwm_us = static_cast<std::uint16_t>(std::llround(std::max(0.0, value)));
        }
    }
}

template<typename T>
void setTimeIfPresent(T& sample, TimePoint steady_tp, double t_sec)
{
    if constexpr (requires { sample.time_sec = t_sec; }) {
        sample.time_sec = t_sec;
    }
    if constexpr (requires { sample.timestamp_sec = t_sec; }) {
        sample.timestamp_sec = t_sec;
    }
    if constexpr (requires { sample.t_sec = t_sec; }) {
        sample.t_sec = t_sec;
    }
    if constexpr (requires { sample.time_s = t_sec; }) {
        sample.time_s = t_sec;
    }

    if constexpr (requires { sample.t = steady_tp; }) {
        sample.t = steady_tp;
    } else if constexpr (requires { sample.t = t_sec; }) {
        sample.t = t_sec;
    }

    if constexpr (requires { sample.timestamp = steady_tp; }) {
        sample.timestamp = steady_tp;
    }
}

template<typename T>
void setRefIfPresent(T& sample, double ref_rad)
{
    if constexpr (requires { sample.ref_angle_rad = ref_rad; }) {
        sample.ref_angle_rad = ref_rad;
    }
}

template<typename T>
void setValidIfPresent(T& sample, bool valid)
{
    if constexpr (requires { sample.valid = valid; }) {
        sample.valid = valid;
    }
}

TelemetrySample makeTelemetrySample(
    Domain domain,
    TelemetrySignalType signal,
    const std::string& joint_name,
    double value,
    TimePoint steady_tp,
    double t_sec)
{
    TelemetrySample sample{};
    setDomainIfPresent(sample, domain);
    setSignalIfPresent(sample, signal);
    setJointIfPresent(sample, joint_name);
    setNumericFieldIfPresent(sample, signal, value);
    setTimeIfPresent(sample, steady_tp, t_sec);
    setValidIfPresent(sample, true);
    return sample;
}

double timeSecFromJointState(const sensor_msgs::msg::JointState& msg, const rclcpp::Node& node)
{
    const double stamp_sec =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (std::isfinite(stamp_sec) && stamp_sec > 0.0) {
        return stamp_sec;
    }

    return node.now().seconds();
}

const char* toCString(Domain domain)
{
    switch (domain) {
    case Domain::Real:
        return "real";
    case Domain::Sim:
        return "sim";
    case Domain::Ref:
        return "ref";
    }
    return "unknown";
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

    for (const auto& joint_name : topics_->jointNames()) {
        pub_joint_pos_real_.emplace(
            joint_name,
            node_->create_publisher<std_msgs::msg::Float64>(
                topics_->jointPositionCommandTopic(Domain::Real, joint_name), qos));

        pub_joint_pos_sim_.emplace(
            joint_name,
            node_->create_publisher<std_msgs::msg::Float64>(
                topics_->jointPositionCommandTopic(Domain::Sim, joint_name), qos));
    }
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

bool DavinciArmRosBridge::shouldEmitJointTelemetry_(
    Domain domain,
    const std::string& joint_name,
    TimePoint now)
{
    std::scoped_lock lock(emit_mtx_);
    auto& last_map = (domain == Domain::Real) ? last_emit_real_ : last_emit_sim_;
    const auto it = last_map.find(joint_name);
    if (it != last_map.end() && (now - it->second) < kUiEmitPeriod) {
        return false;
    }
    last_map[joint_name] = now;
    return true;
}

void DavinciArmRosBridge::onJointState_(Domain domain, const sensor_msgs::msg::JointState& msg)
{
    const auto now = SteadyClock::now();
    const double t_sec = timeSecFromJointState(msg, *node_);

    struct PendingSample {
        std::string joint_name;
        double position_rad{0.0};
        std::optional<double> velocity_rad_s;
    };

    std::vector<PendingSample> pending;
    pending.reserve(msg.name.size());

    {
        std::scoped_lock lock(state_mtx_);
        auto& snapshot = (domain == Domain::Real) ? real_state_ : sim_state_;
        snapshot.valid = true;
        snapshot.last_rx = now;

        for (std::size_t i = 0; i < msg.name.size(); ++i) {
            const auto& joint_name = msg.name[i];
            const auto it = std::find(
                                topics_->jointNames().begin(),
                                topics_->jointNames().end(),
                                joint_name);
            if (it == topics_->jointNames().end()) {
                continue;
            }

            if (i < msg.position.size()) {
                snapshot.position_by_joint[joint_name] = msg.position[i];

                if (!ref_position_by_joint_.contains(joint_name)) {
                    ref_position_by_joint_[joint_name] = msg.position[i];
                }
            }
            if (i < msg.velocity.size()) {
                snapshot.velocity_by_joint[joint_name] = msg.velocity[i];
            }

            const auto pos_it = snapshot.position_by_joint.find(joint_name);
            if (pos_it == snapshot.position_by_joint.end()) {
                continue;
            }

            PendingSample item;
            item.joint_name = joint_name;
            item.position_rad = pos_it->second;

            const auto vel_it = snapshot.velocity_by_joint.find(joint_name);
            if (vel_it != snapshot.velocity_by_joint.end()) {
                item.velocity_rad_s = vel_it->second;
            }

            pending.push_back(std::move(item));
        }
    }

    for (const auto& item : pending) {
        if (!shouldEmitJointTelemetry_(domain, item.joint_name, now)) {
            continue;
        }
        emitJointTelemetry_(domain, item.joint_name, item.position_rad, item.velocity_rad_s, t_sec);
    }

    touchRx_(domain);
}

void DavinciArmRosBridge::onControllerState_(const control_msgs::msg::JointTrajectoryControllerState& msg)
{
    const auto count = std::min(msg.joint_names.size(), msg.reference.positions.size());
    const double t_sec = node_->now().seconds();

    std::vector<std::pair<std::string, double>> refs;
    refs.reserve(count);

    {
        std::scoped_lock lock(state_mtx_);
        for (std::size_t i = 0; i < count; ++i) {
            const auto& joint_name = msg.joint_names[i];
            if (std::find(topics_->jointNames().begin(), topics_->jointNames().end(), joint_name) ==
                    topics_->jointNames().end()) {
                continue;
            }

            ref_position_by_joint_[joint_name] = msg.reference.positions[i];
            refs.emplace_back(joint_name, msg.reference.positions[i]);
        }
    }

    for (const auto& [joint_name, ref] : refs) {
        emitReferenceTelemetry_(joint_name, ref, t_sec);
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
    const std::optional<double>& velocity_rad_s,
    double time_sec)
{
    const auto steady_now = SteadyClock::now();

    std::optional<double> ref_value;
    {
        std::scoped_lock lock(state_mtx_);
        const auto ref_it = ref_position_by_joint_.find(joint_name);
        if (ref_it != ref_position_by_joint_.end()) {
            ref_value = ref_it->second;
        }
    }

    auto angle_sample = makeTelemetrySample(
                            domain,
                            TelemetrySignalType::Angle,
                            joint_name,
                            position_rad,
                            steady_now,
                            time_sec);

    if (ref_value.has_value()) {
        setRefIfPresent(angle_sample, *ref_value);
    }

    emit telemetryUpdated(angle_sample);

    if (velocity_rad_s.has_value()) {
        auto speed_sample = makeTelemetrySample(
                                domain,
                                TelemetrySignalType::MotorSpeed,
                                joint_name,
                                *velocity_rad_s,
                                steady_now,
                                time_sec);

        if (ref_value.has_value()) {
            setRefIfPresent(speed_sample, *ref_value);
        }

        emit telemetryUpdated(speed_sample);
    }

    if (ref_value.has_value()) {
        emitReferenceTelemetry_(joint_name, *ref_value, time_sec);
    }
}

void DavinciArmRosBridge::emitReferenceTelemetry_(
    const std::string& joint_name,
    double ref_position_rad,
    double time_sec)
{
    const auto steady_now = SteadyClock::now();

    auto ref_sample = makeTelemetrySample(
                          Domain::Ref,
                          TelemetrySignalType::AngleRef,
                          joint_name,
                          ref_position_rad,
                          steady_now,
                          time_sec);

    emit telemetryUpdated(ref_sample);
}

void DavinciArmRosBridge::touchRx_(Domain domain)
{
    const bool previous_real = real_live_;
    const bool previous_sim = sim_live_;

    if (domain == Domain::Real) {
        real_live_ = true;
    } else if (domain == Domain::Sim) {
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
    const auto now = SteadyClock::now();
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

bool DavinciArmRosBridge::isEstopLatched_() const
{
    std::scoped_lock lock(state_mtx_);
    return SteadyClock::now() < estop_latched_until_;
}

void DavinciArmRosBridge::sendJointTrajectory(
    Domain domain,
    const std::vector<double>& positions_rad,
    const std::vector<double>& velocities_rad_s,
    const std::vector<double>& accelerations_rad_s2,
    double time_from_start_s)
{
    (void)velocities_rad_s;
    (void)accelerations_rad_s2;
    (void)time_from_start_s;

    if (isEstopLatched_()) {
        return;
    }

    const auto& joints = topics_->jointNames();
    if (positions_rad.size() != joints.size()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "sendJointTrajectory(%s): expected %zu joint positions but received %zu.",
            toCString(domain),
            joints.size(),
            positions_rad.size());
        return;
    }

    {
        std::scoped_lock lock(state_mtx_);
        for (std::size_t i = 0; i < joints.size(); ++i) {
            ref_position_by_joint_[joints[i]] = positions_rad[i];
        }
    }

    const double t_sec = node_->now().seconds();
    for (std::size_t i = 0; i < joints.size(); ++i) {
        emitReferenceTelemetry_(joints[i], positions_rad[i], t_sec);
    }

    auto& pub_map = (domain == Domain::Real) ? pub_joint_pos_real_ : pub_joint_pos_sim_;

    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto& joint_name = joints[i];
        const auto pub_it = pub_map.find(joint_name);
        if (pub_it == pub_map.end() || !pub_it->second) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Missing joint command publisher for %s joint '%s'.",
                toCString(domain),
                joint_name.c_str());
            continue;
        }

        std_msgs::msg::Float64 msg;
        msg.data = positions_rad[i];
        publish_(pub_it->second, msg);
    }
}

void DavinciArmRosBridge::sendHoldPosition(Domain domain)
{
    sendJointTrajectory(domain, currentPositionsOrZeros_(domain));
}

void DavinciArmRosBridge::sendRefAngle(double rad)
{
    for (const auto domain : kCommandDomains) {
        sendAngleReference(domain, rad);
    }
}

void DavinciArmRosBridge::sendPwm(std::uint16_t us)
{
    (void)us;
}

void DavinciArmRosBridge::sendAutoMode(bool enabled)
{
    (void)enabled;
}

void DavinciArmRosBridge::sendStop()
{
    {
        std::scoped_lock lock(state_mtx_);
        estop_latched_until_ = SteadyClock::now() + kEstopHoldoff;
    }

    // Publish hold twice to absorb trailing queued commands
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (const auto domain : kCommandDomains) {
            const auto hold = currentPositionsOrZeros_(domain);

            {
                std::scoped_lock lock(state_mtx_);
                const auto& joints = topics_->jointNames();
                for (std::size_t i = 0; i < joints.size() && i < hold.size(); ++i) {
                    ref_position_by_joint_[joints[i]] = hold[i];
                }
            }

            auto& pub_map = (domain == Domain::Real) ? pub_joint_pos_real_ : pub_joint_pos_sim_;
            const auto& joints = topics_->jointNames();

            for (std::size_t i = 0; i < joints.size() && i < hold.size(); ++i) {
                const auto pub_it = pub_map.find(joints[i]);
                if (pub_it == pub_map.end() || !pub_it->second) {
                    continue;
                }

                std_msgs::msg::Float64 msg;
                msg.data = hold[i];
                publish_(pub_it->second, msg);
            }
        }
    }
}

void DavinciArmRosBridge::sendAngleReference(Domain domain, double rad)
{
    if (isEstopLatched_()) {
        return;
    }

    auto positions = currentPositionsOrZeros_(domain);
    if (positions.empty()) {
        return;
    }

    positions.front() = rad;
    sendJointTrajectory(domain, positions);
}

void DavinciArmRosBridge::sendPwmCommand(Domain domain, std::uint16_t us)
{
    (void)domain;
    (void)us;
}

void DavinciArmRosBridge::sendAutoModeCommand(Domain domain, bool enabled)
{
    (void)domain;
    (void)enabled;
}

bool DavinciArmRosBridge::isConnected() const noexcept
{
    return connected_;
}

bool DavinciArmRosBridge::isLive(Domain domain) const noexcept
{
    if (domain == Domain::Real) {
        return real_live_;
    }
    if (domain == Domain::Sim) {
        return sim_live_;
    }
    return false;
}

}  // namespace davinci_arm::infra::ros