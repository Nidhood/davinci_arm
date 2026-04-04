#pragma once

#include "davinci_arm_gui/core/services/calibration_interfaces.hpp"
#include "davinci_arm_gui/infra/ros/calibration_command_sink.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stop_token>
#include <vector>

namespace davinci_arm::models {
class TelemetryStore;
}

namespace davinci_arm::infra::ros {

class StepCalibrationCommander final : public davinci_arm::services::ICalibrationCommander {
public:
    explicit StepCalibrationCommander(
        std::shared_ptr<davinci_arm::models::TelemetryStore> store,
        std::shared_ptr<ICalibrationCommandSink> sink);

    bool runMotorSequence(
        const std::vector<double>& pwm_steps,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st,
        bool use_real,
        bool use_sim) override;

    bool runPhysicsSequence(
        const std::vector<double>& ref_steps_rad,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st,
        bool use_real,
        bool use_sim) override;

    void stopAll() override;

private:
    bool sleepOrStop_(std::chrono::duration<double> d, std::stop_token st) const;
    void clearTelemetry_(bool use_real, bool use_sim) const;
    void broadcastPwm_(std::uint16_t pwm_us, bool use_real, bool use_sim) const;
    void broadcastRefAngle_(double rad, bool use_real, bool use_sim) const;

private:
    std::shared_ptr<davinci_arm::models::TelemetryStore> store_;
    std::shared_ptr<ICalibrationCommandSink> sink_;
};

}  // namespace davinci_arm::infra::ros