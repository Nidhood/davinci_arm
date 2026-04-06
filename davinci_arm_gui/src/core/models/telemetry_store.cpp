#include "davinci_arm_gui/core/models/telemetry_store.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace davinci_arm::models {

TelemetryStore::TelemetryStore(std::size_t max_history)
    : max_history_(std::max<std::size_t>(1, max_history))
{
    initializeBuffers_();
}

void TelemetryStore::initializeBuffers_()
{
    buffers_.emplace(Domain::Real, DomainBuffer{});
    buffers_.emplace(Domain::Sim, DomainBuffer{});
    buffers_.emplace(Domain::Ref, DomainBuffer{});
}

void TelemetryStore::trimHistoryUnlocked_(DomainBuffer& buffer) const
{
    if (buffer.history.size() <= max_history_) {
        return;
    }

    const auto erase_count = buffer.history.size() - max_history_;
    buffer.history.erase(
        buffer.history.begin(),
        buffer.history.begin() + static_cast<long>(erase_count));
}

void TelemetryStore::setMaxHistory(std::size_t max_history)
{
    std::lock_guard<std::mutex> lock(mtx_);
    max_history_ = std::max<std::size_t>(1, max_history);

    for (auto& [_, buffer] : buffers_) {
        trimHistoryUnlocked_(buffer);
    }
}

std::size_t TelemetryStore::maxHistory() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return max_history_;
}

void TelemetryStore::clear()
{
    std::lock_guard<std::mutex> lock(mtx_);

    for (auto& [_, buffer] : buffers_) {
        buffer.latest.reset();
        buffer.history.clear();
    }
}

void TelemetryStore::clearDomain(Domain domain)
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return;
    }

    it->second.latest.reset();
    it->second.history.clear();
}

void TelemetryStore::push(const TelemetrySample& sample)
{
    if (!sample.valid) {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(sample.domain);
    if (it == buffers_.end()) {
        return;
    }

    auto& buffer = it->second;
    buffer.latest = sample;
    buffer.history.push_back(sample);
    trimHistoryUnlocked_(buffer);
}

std::optional<TelemetrySample> TelemetryStore::latest(Domain domain) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return std::nullopt;
    }

    return it->second.latest;
}

std::optional<TelemetrySample> TelemetryStore::getLatestSample(Domain domain) const
{
    return latest(domain);
}

std::vector<TelemetrySample> TelemetryStore::history(
    Domain domain,
    std::size_t max_points) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return {};
    }

    return downsampleUniform_(
               it->second.history,
               std::max<std::size_t>(1, max_points));
}

std::vector<TelemetrySample> TelemetryStore::historyRange(
    Domain domain,
    std::size_t start_index,
    std::size_t end_index) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return {};
    }

    const auto& history = it->second.history;
    if (start_index >= history.size()) {
        return {};
    }

    const std::size_t actual_end = std::min(end_index, history.size());
    if (actual_end <= start_index) {
        return {};
    }

    return std::vector<TelemetrySample>(
               history.begin() + static_cast<long>(start_index),
               history.begin() + static_cast<long>(actual_end));
}

std::size_t TelemetryStore::size(Domain domain) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return 0;
    }

    return it->second.history.size();
}

std::size_t TelemetryStore::sampleCount(Domain domain) const
{
    return size(domain);
}

std::optional<TelemetrySample> TelemetryStore::getSample(
    Domain domain,
    std::size_t index) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return std::nullopt;
    }

    const auto& history = it->second.history;
    if (index >= history.size()) {
        return std::nullopt;
    }

    return history[index];
}

std::vector<double> TelemetryStore::extractSignal(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t start_index,
    std::size_t count) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return {};
    }

    const auto& history = it->second.history;
    if (start_index >= history.size()) {
        return {};
    }

    const std::size_t actual_count =
        (count == 0)
        ? (history.size() - start_index)
        : std::min(count, history.size() - start_index);

    std::vector<double> result;
    result.reserve(actual_count);

    for (std::size_t i = start_index; i < start_index + actual_count; ++i) {
        result.push_back(extractSignalValue_(history[i], signal));
    }

    return result;
}

std::vector<double> TelemetryStore::extractSignalDownsampled(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t max_points,
    std::size_t start_index) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return {};
    }

    const auto& history = it->second.history;
    if (start_index >= history.size()) {
        return {};
    }

    std::vector<TelemetrySample> subset(
        history.begin() + static_cast<long>(start_index),
        history.end());

    const auto downsampled = downsampleUniform_(subset, max_points);

    std::vector<double> result;
    result.reserve(downsampled.size());

    for (const auto& sample : downsampled) {
        result.push_back(extractSignalValue_(sample, signal));
    }

    return result;
}

std::optional<TelemetryStore::SignalStats> TelemetryStore::computeSignalStats(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t start_index,
    std::size_t count) const
{
    const auto values = extractSignal(domain, signal, start_index, count);
    if (values.empty()) {
        return std::nullopt;
    }

    SignalStats stats;
    stats.count = values.size();

    const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
    stats.min = *min_it;
    stats.max = *max_it;

    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.mean = sum / static_cast<double>(values.size());

    double sq_sum = 0.0;
    for (const double value : values) {
        const double diff = value - stats.mean;
        sq_sum += diff * diff;
    }

    stats.stddev = std::sqrt(sq_sum / static_cast<double>(values.size()));
    return stats;
}

std::vector<TelemetrySample> TelemetryStore::getRecentSamples(
    Domain domain,
    double time_window_sec) const
{
    std::lock_guard<std::mutex> lock(mtx_);

    const auto it = buffers_.find(domain);
    if (it == buffers_.end()) {
        return {};
    }

    const auto& history = it->second.history;
    if (history.empty()) {
        return {};
    }

    const auto latest_time = history.back().t;

    std::vector<TelemetrySample> result;
    for (auto rit = history.rbegin(); rit != history.rend(); ++rit) {
        const auto time_diff =
            std::chrono::duration<double>(latest_time - rit->t).count();

        if (time_diff > time_window_sec) {
            break;
        }

        result.push_back(*rit);
    }

    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<TelemetrySample> TelemetryStore::downsampleUniform_(
    const std::vector<TelemetrySample>& in,
    std::size_t max_points)
{
    if (in.size() <= max_points) {
        return in;
    }
    if (max_points == 0) {
        return {};
    }
    if (max_points == 1) {
        return {in.back()};
    }

    std::vector<TelemetrySample> out;
    out.reserve(max_points);

    const double step =
        static_cast<double>(in.size() - 1) /
        static_cast<double>(max_points - 1);

    for (std::size_t i = 0; i < max_points; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i * step);
        out.push_back(in[idx]);
    }

    return out;
}

double TelemetryStore::extractSignalValue_(
    const TelemetrySample& sample,
    TelemetrySignalType signal)
{
    switch (signal) {
    case TelemetrySignalType::Angle:
        return sample.arm_angle_rad;
    case TelemetrySignalType::MotorSpeed:
        return sample.motor_speed_rad_s;
    case TelemetrySignalType::PwmFeedback:
        return static_cast<double>(sample.pwm_us);
    case TelemetrySignalType::AngleRef:
        return sample.ref_angle_rad;
    default:
        return 0.0;
    }
}

}  // namespace davinci_arm::models