#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <vector>

#include "davinci_arm_gui/core/logging/data_recorder.hpp"
#include "davinci_arm_gui/core/models/recording_stats.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

namespace davinci_arm::core::services {

class RecorderService final {
public:
    using OnProgressFn = std::function<void(double remaining_s, std::size_t points_total)>;
    using OnCompletedFn = std::function<void(std::size_t points_total, double duration_s)>;

    RecorderService();

    void start(double duration_s);
    void stop();
    void clear();

    [[nodiscard]] bool isRecording() const;

    void onSample(const davinci_arm::models::TelemetrySample& sample);

    [[nodiscard]] std::vector<davinci_arm::models::TelemetrySample> recorded() const;
    [[nodiscard]] davinci_arm::models::RecordingStats stats() const;

    void setOnProgress(OnProgressFn cb);
    void setOnCompleted(OnCompletedFn cb);

private:
    mutable std::mutex mtx_;
    davinci_arm::core::logging::DataRecorder recorder_;

    OnProgressFn on_progress_{};
    OnCompletedFn on_completed_{};

    davinci_arm::models::RecordingState last_state_{davinci_arm::models::RecordingState::Idle};
    double last_progress_remaining_s_{-1.0};
    bool completed_emitted_{false};
};

}  // namespace davinci_arm::core::services
