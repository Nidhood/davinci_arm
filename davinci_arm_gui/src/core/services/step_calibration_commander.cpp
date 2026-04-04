#include "davinci_arm_gui/infra/ros/step_calibration_commander.hpp"

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_store.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace davinci_arm::infra::ros {

namespace {

std::uint16_t clampPwm(double v) {
    const int x = static_cast<int>(std::lround(v));
    return static_cast<std::uint16_t>(std::clamp(x, 800, 2200));
}

double sanitizeNonNegative(double v) {
    return std::max(0.0, v);
}

}  // namespace

StepCalibrationCommander::StepCalibrationCommander(
    std::shared_ptr<davinci_arm::models::TelemetryStore> store,
    std::shared_ptr<ICalibrationCommandSink> sink)
    : store_(std::move(store)),
      sink_(std::move(sink)) {}

bool StepCalibrationCommander::runMotorSequence(
    const std::vector<double>& pwm_steps,
    double step_duration_sec,
    double settling_time_sec,
    std::stop_token st,
    bool use_real,
    bool use_sim)
{
    if (!sink_ || !store_) return false;
    if (pwm_steps.empty()) return false;
    if (!use_real && !use_sim) return false;

    const double hold_sec = sanitizeNonNegative(step_duration_sec);
    const double settle_sec = sanitizeNonNegative(settling_time_sec);

    clearTelemetry_(use_real, use_sim);

    if (!sleepOrStop_(std::chrono::duration<double>(settle_sec), st)) {
        return false;
    }

    for (double pwm : pwm_steps) {
        if (st.stop_requested()) {
            return false;
        }

        broadcastPwm_(clampPwm(pwm), use_real, use_sim);

        if (!sleepOrStop_(std::chrono::duration<double>(settle_sec), st)) {
            return false;
        }

        if (!sleepOrStop_(std::chrono::duration<double>(hold_sec), st)) {
            return false;
        }
    }

    return !st.stop_requested();
}

bool StepCalibrationCommander::runPhysicsSequence(
    const std::vector<double>& ref_steps_rad,
    double step_duration_sec,
    double settling_time_sec,
    std::stop_token st,
    bool use_real,
    bool use_sim)
{
    if (!sink_ || !store_) return false;
    if (ref_steps_rad.empty()) return false;
    if (!use_real && !use_sim) return false;

    const double hold_sec = sanitizeNonNegative(step_duration_sec);
    const double settle_sec = sanitizeNonNegative(settling_time_sec);

    clearTelemetry_(use_real, use_sim);

    if (!sleepOrStop_(std::chrono::duration<double>(settle_sec), st)) {
        return false;
    }

    for (double rad : ref_steps_rad) {
        if (st.stop_requested()) {
            return false;
        }

        broadcastRefAngle_(rad, use_real, use_sim);

        if (!sleepOrStop_(std::chrono::duration<double>(settle_sec), st)) {
            return false;
        }

        if (!sleepOrStop_(std::chrono::duration<double>(hold_sec), st)) {
            return false;
        }
    }

    return !st.stop_requested();
}

void StepCalibrationCommander::stopAll() {
    if (!sink_) {
        return;
    }

    // Keep this conservative. Adjust these safe values if your system needs a different neutral command.
    broadcastPwm_(clampPwm(1000.0), true, true);
    broadcastRefAngle_(0.0, true, true);
}

bool StepCalibrationCommander::sleepOrStop_(std::chrono::duration<double> d, std::stop_token st) const {
    constexpr auto kTick = std::chrono::milliseconds(20);

    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(d);
    while (remaining.count() > 0) {
        if (st.stop_requested()) {
            return false;
        }

        const auto step = (remaining > kTick) ? kTick : remaining;
        std::this_thread::sleep_for(step);
        remaining -= step;
    }

    return !st.stop_requested();
}

void StepCalibrationCommander::clearTelemetry_(bool use_real, bool use_sim) const {
    if (!store_) {
        return;
    }

    if (use_real) {
        store_->clearDomain(davinci_arm::models::Domain::Real);
    }

    if (use_sim) {
        store_->clearDomain(davinci_arm::models::Domain::Sim);
    }
}

void StepCalibrationCommander::broadcastPwm_(std::uint16_t pwm_us, bool use_real, bool use_sim) const {
    if (!sink_) {
        return;
    }

    if (use_real) {
        sink_->sendPwmUs(davinci_arm::models::Domain::Real, pwm_us);
    }

    if (use_sim) {
        sink_->sendPwmUs(davinci_arm::models::Domain::Sim, pwm_us);
    }
}

void StepCalibrationCommander::broadcastRefAngle_(double rad, bool use_real, bool use_sim) const {
    if (!sink_) {
        return;
    }

    if (use_real) {
        sink_->sendRefAngleRad(davinci_arm::models::Domain::Real, rad);
    }

    if (use_sim) {
        sink_->sendRefAngleRad(davinci_arm::models::Domain::Sim, rad);
    }
}

}  // namespace davinci_arm::infra::ros