#include "davinci_arm_gui/core/logging/data_recorder.hpp"

#include <algorithm>

namespace davinci_arm::core::logging {

void DataRecorder::start(double duration_s)
{
    std::lock_guard<std::mutex> lock(mtx_);

    data_.clear();
    duration_s_ = std::max(0.0, duration_s);
    t0_ = SteadyClock::now();
    t_end_ = t0_ + std::chrono::duration_cast<SteadyClock::duration>(
                 std::chrono::duration<double>(duration_s_));
    state_ = davinci_arm::models::RecordingState::Recording;
}

void DataRecorder::stop()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (state_ == davinci_arm::models::RecordingState::Recording) {
        state_ = davinci_arm::models::RecordingState::Stopped;
    }
}

void DataRecorder::clear()
{
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
    duration_s_ = 0.0;
    t0_ = {};
    t_end_ = {};
    state_ = davinci_arm::models::RecordingState::Idle;
}

bool DataRecorder::isRecording() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return state_ == davinci_arm::models::RecordingState::Recording;
}

davinci_arm::models::RecordingState DataRecorder::state() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (state_ == davinci_arm::models::RecordingState::Recording && SteadyClock::now() >= t_end_) {
        return davinci_arm::models::RecordingState::Completed;
    }
    return state_;
}

void DataRecorder::pushSample(const davinci_arm::models::TelemetrySample& sample)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (state_ != davinci_arm::models::RecordingState::Recording) {
        return;
    }
    if (!sample.valid) {
        return;
    }

    const auto now = SteadyClock::now();
    if (now >= t_end_) {
        state_ = davinci_arm::models::RecordingState::Completed;
        return;
    }

    data_.push_back(sample);
}

std::vector<davinci_arm::models::TelemetrySample> DataRecorder::recorded() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

std::vector<davinci_arm::models::TelemetrySample>
DataRecorder::recorded(davinci_arm::models::Domain d) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    std::vector<davinci_arm::models::TelemetrySample> out;
    out.reserve(data_.size());

    for (const auto& s : data_) {
        if (s.domain == d) {
            out.push_back(s);
        }
    }

    return out;
}

davinci_arm::models::RecordingStats DataRecorder::stats() const
{
    std::lock_guard<std::mutex> lock(mtx_);

    davinci_arm::models::RecordingStats st;
    st.duration_s = duration_s_;
    st.points_total = data_.size();

    for (const auto& s : data_) {
        if (s.domain == davinci_arm::models::Domain::Real) {
            st.points_real++;
        } else if (s.domain == davinci_arm::models::Domain::Sim) {
            st.points_sim++;
        }
    }

    const auto now = SteadyClock::now();
    const bool should_complete =
        (state_ == davinci_arm::models::RecordingState::Recording && now >= t_end_);

    st.state = should_complete ? davinci_arm::models::RecordingState::Completed : state_;

    if (t0_ != SteadyClock::time_point{}) {
        const auto effective_end =
            (st.state == davinci_arm::models::RecordingState::Completed) ? t_end_ : now;
        const auto elapsed = std::max(SteadyClock::duration::zero(), effective_end - t0_);
        st.elapsed_s = seconds_(elapsed);
    }

    if (st.state == davinci_arm::models::RecordingState::Recording) {
        const auto remaining = (t_end_ > now) ? (t_end_ - now) : SteadyClock::duration::zero();
        st.remaining_s = std::max(0.0, seconds_(remaining));
    } else {
        st.remaining_s = 0.0;
    }

    return st;
}

double DataRecorder::seconds_(const SteadyClock::duration& d)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

}  // namespace davinci_arm::core::logging