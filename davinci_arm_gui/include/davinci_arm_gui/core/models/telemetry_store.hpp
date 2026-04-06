#pragma once

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/domain_buffer.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace davinci_arm::models {

class TelemetryStore final {
public:
    struct SignalStats {
        double min{0.0};
        double max{0.0};
        double mean{0.0};
        double stddev{0.0};
        std::size_t count{0};
    };

    explicit TelemetryStore(std::size_t max_history = 10000);
    ~TelemetryStore() = default;

    TelemetryStore(const TelemetryStore&) = delete;
    TelemetryStore& operator=(const TelemetryStore&) = delete;
    TelemetryStore(TelemetryStore&&) = delete;
    TelemetryStore& operator=(TelemetryStore&&) = delete;

    void setMaxHistory(std::size_t max_history);
    [[nodiscard]] std::size_t maxHistory() const;

    void clear();
    void clearDomain(Domain domain);
    void push(const TelemetrySample& sample);

    [[nodiscard]] std::optional<TelemetrySample> latest(Domain domain) const;
    [[nodiscard]] std::optional<TelemetrySample> getLatestSample(Domain domain) const;

    [[nodiscard]] std::vector<TelemetrySample> history(
        Domain domain,
        std::size_t max_points = 1000) const;

    [[nodiscard]] std::vector<TelemetrySample> historyRange(
        Domain domain,
        std::size_t start_index,
        std::size_t end_index) const;

    [[nodiscard]] std::size_t size(Domain domain) const;
    [[nodiscard]] std::size_t sampleCount(Domain domain) const;

    [[nodiscard]] std::optional<TelemetrySample> getSample(
        Domain domain,
        std::size_t index) const;

    [[nodiscard]] std::vector<double> extractSignal(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t start_index = 0,
        std::size_t count = 0) const;

    [[nodiscard]] std::vector<double> extractSignalDownsampled(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t max_points,
        std::size_t start_index = 0) const;

    [[nodiscard]] std::optional<SignalStats> computeSignalStats(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t start_index = 0,
        std::size_t count = 0) const;

    [[nodiscard]] std::vector<TelemetrySample> getRecentSamples(
        Domain domain,
        double time_window_sec) const;

private:
    void initializeBuffers_();
    void trimHistoryUnlocked_(DomainBuffer& buffer) const;

    [[nodiscard]] static std::vector<TelemetrySample> downsampleUniform_(
        const std::vector<TelemetrySample>& in,
        std::size_t max_points);

    [[nodiscard]] static double extractSignalValue_(
        const TelemetrySample& sample,
        TelemetrySignalType signal);

private:
    mutable std::mutex mtx_;
    std::size_t max_history_;
    std::map<Domain, DomainBuffer> buffers_;
};

}  // namespace davinci_arm::models