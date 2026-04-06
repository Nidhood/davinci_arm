#pragma once

#include <chrono>
#include <mutex>
#include <vector>

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/recording_stats.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

namespace davinci_arm::core::logging {

using SteadyClock = std::chrono::steady_clock;

class DataRecorder final {
public:
    DataRecorder() = default;

    void start(double duration_s);
    void stop();
    void clear();

    [[nodiscard]] bool isRecording() const;
    [[nodiscard]] davinci_arm::models::RecordingState state() const;

    void pushSample(const davinci_arm::models::TelemetrySample& sample);

    [[nodiscard]] std::vector<davinci_arm::models::TelemetrySample> recorded() const;
    [[nodiscard]] std::vector<davinci_arm::models::TelemetrySample> recorded(davinci_arm::models::Domain d) const;
    [[nodiscard]] davinci_arm::models::RecordingStats stats() const;

private:
    static double seconds_(const SteadyClock::duration& d);

private:
    mutable std::mutex mtx_;
    davinci_arm::models::RecordingState state_{davinci_arm::models::RecordingState::Idle};
    double duration_s_{0.0};
    SteadyClock::time_point t0_{};
    SteadyClock::time_point t_end_{};
    std::vector<davinci_arm::models::TelemetrySample> data_;
};

}  // namespace davinci_arm::core::logging