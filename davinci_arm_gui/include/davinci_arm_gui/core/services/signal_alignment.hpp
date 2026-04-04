#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "davinci_arm_gui/core/models/plot_signal_type.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

namespace davinci_arm::core::services {

struct AlignedSignals final {
    std::vector<double> t_sec;
    std::vector<double> real_y;
    std::vector<double> sim_y;
};

struct ErrorMetrics final {
    double mse{0.0};
    double rmse{0.0};
    double max_abs{0.0};
    double mean_abs{0.0};
    double correlation{0.0};
    std::size_t n{0};
};

class SignalAlignment final {
public:
    static std::optional<AlignedSignals> alignAndResample(
        const std::vector<davinci_arm::models::TelemetrySample>& real_samples,
        const std::vector<davinci_arm::models::TelemetrySample>& sim_samples,
        davinci_arm::models::PlotSignalType signal,
        double dt_sec);

    static ErrorMetrics computeError(const AlignedSignals& a);

private:
    struct Series final {
        std::vector<double> t;
        std::vector<double> y;
    };

    static Series buildSeries_(const std::vector<davinci_arm::models::TelemetrySample>& samples,
                               davinci_arm::models::PlotSignalType signal);

    static double sampleToValue_(const davinci_arm::models::TelemetrySample& s,
                                 davinci_arm::models::PlotSignalType signal);

    static std::vector<double> linspace_(double t0, double t1, double dt);
    static std::vector<double> interpLinear_(const Series& s, const std::vector<double>& t_out);
    static double safeCorrelation_(const std::vector<double>& a, const std::vector<double>& b);
    static double radToDeg_(double rad);
};

}  // namespace davinci_arm::core::services
