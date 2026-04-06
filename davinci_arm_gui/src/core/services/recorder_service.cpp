#include "davinci_arm_gui/core/services/recorder_service.hpp"

namespace davinci_arm::core::services {

RecorderService::RecorderService() = default;

void RecorderService::start(double duration_s)
{
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.start(duration_s);
    last_state_ = davinci_arm::models::RecordingState::Recording;
    last_progress_remaining_s_ = -1.0;
    completed_emitted_ = false;
}

void RecorderService::stop()
{
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.stop();
}

void RecorderService::clear()
{
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.clear();
    last_state_ = davinci_arm::models::RecordingState::Idle;
    last_progress_remaining_s_ = -1.0;
    completed_emitted_ = false;
}

bool RecorderService::isRecording() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.isRecording();
}

void RecorderService::onSample(const davinci_arm::models::TelemetrySample& sample)
{
    OnProgressFn progress_cb;
    OnCompletedFn completed_cb;
    davinci_arm::models::RecordingStats st;
    bool emit_progress = false;
    bool emit_completed = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);

        recorder_.pushSample(sample);
        st = recorder_.stats();

        progress_cb = on_progress_;
        completed_cb = on_completed_;

        if (st.state != last_state_) {
            last_state_ = st.state;
            if (st.state == davinci_arm::models::RecordingState::Recording) {
                completed_emitted_ = false;
                last_progress_remaining_s_ = -1.0;
            }
        }

        if (progress_cb && st.state == davinci_arm::models::RecordingState::Recording) {
            if (last_progress_remaining_s_ < 0.0 ||
                    st.remaining_s <= (last_progress_remaining_s_ - 0.05)) {
                last_progress_remaining_s_ = st.remaining_s;
                emit_progress = true;
            }
        }

        if (completed_cb &&
                st.state == davinci_arm::models::RecordingState::Completed &&
                !completed_emitted_) {
            completed_emitted_ = true;
            emit_completed = true;
        }
    }

    if (emit_progress && progress_cb) {
        progress_cb(st.remaining_s, st.points_total);
    }

    if (emit_completed && completed_cb) {
        completed_cb(st.points_total, st.duration_s);
    }
}

std::vector<davinci_arm::models::TelemetrySample> RecorderService::recorded() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.recorded();
}

davinci_arm::models::RecordingStats RecorderService::stats() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.stats();
}

void RecorderService::setOnProgress(OnProgressFn cb)
{
    std::lock_guard<std::mutex> lock(mtx_);
    on_progress_ = std::move(cb);
}

void RecorderService::setOnCompleted(OnCompletedFn cb)
{
    std::lock_guard<std::mutex> lock(mtx_);
    on_completed_ = std::move(cb);
}

}  // namespace davinci_arm::core::services