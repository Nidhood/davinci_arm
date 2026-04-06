#include "davinci_arm_gui/ui/widgets/angle_ref_plot.hpp"

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"
#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"

#include <QVBoxLayout>
#include <cmath>
#include <numbers>
#include <optional>
#include <type_traits>

namespace davinci_arm::ui::widgets {

namespace {

using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;

template<typename T>
std::optional<TelemetrySignalType> signalTypeOf(const T& sample)
{
    if constexpr (requires { sample.signal; }) {
        return sample.signal;
    } else if constexpr (requires { sample.signal_type; }) {
        return sample.signal_type;
    } else if constexpr (requires { sample.telemetry_signal; }) {
        return sample.telemetry_signal;
    } else {
        return std::nullopt;
    }
}

// Map from [-pi, pi] to [0, 360]:
// -pi -> 0
//  0  -> 180
// +pi -> 360
double mapJointRadToUiDeg(double rad)
{
    constexpr double pi = std::numbers::pi;
    constexpr double two_pi = 2.0 * pi;
    constexpr double eps = 1e-9;

    double wrapped = std::remainder(rad, two_pi);

    if (std::abs(wrapped + pi) < eps) {
        return 0.0;
    }
    if (std::abs(wrapped - pi) < eps) {
        return 360.0;
    }

    double deg = (wrapped + pi) * 180.0 / pi;

    if (deg < 0.0) {
        deg += 360.0;
    }
    if (deg > 360.0) {
        deg -= 360.0;
    }

    return deg;
}

}  // namespace

AngleRefPlot::AngleRefPlot(QWidget* parent)
    : QWidget(parent)
{
    buildUi_();
}

void AngleRefPlot::buildUi_()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    chart_ = new ChartBase("", "deg", this);
    chart_->setSeriesLabels("Real", "Sim", "Ref");
    chart_->setShowSim(true);
    chart_->setShowRef(true);
    chart_->setRealLive(false);
    chart_->setSimLive(false);
    chart_->setRefLive(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(2200);

    applyLimits_();
    root->addWidget(chart_, 1);
}

std::chrono::steady_clock::time_point AngleRefPlot::sampleTime_(const davinci_arm::models::TelemetrySample& s)
{
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void AngleRefPlot::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept
{
    limits_ = limits;
    applyLimits_();
}

void AngleRefPlot::applyLimits_()
{
    if (!limits_ || !chart_) {
        return;
    }

    const auto& a = limits_->angleLimits();
    chart_->setYRange(a.min, a.max);
}

void AngleRefPlot::updateHeldRef_(double ref_deg)
{
    have_ref_ = true;
    held_ref_deg_ = ref_deg;
}

void AngleRefPlot::appendHeldRefAt_(double t_sec)
{
    if (!chart_ || !have_ref_) {
        return;
    }
    chart_->appendRef(t_sec, held_ref_deg_);
}

void AngleRefPlot::pushSample(const davinci_arm::models::TelemetrySample& sample)
{
    if (!chart_ || !sample.valid) {
        return;
    }

    const auto signal = signalTypeOf(sample);
    if (!signal.has_value()) {
        return;
    }

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();

    if (*signal == TelemetrySignalType::AngleRef) {
        const double ref_deg = mapJointRadToUiDeg(sample.ref_angle_rad);
        constexpr double kRefEpsDeg = 1e-6;

        // First reference sample: create the series.
        if (!have_ref_) {
            updateHeldRef_(ref_deg);
            chart_->appendRef(t_sec, ref_deg);
            chart_->setRefLive(true);
            return;
        }

        // Ignore repeated identical reference samples.
        if (std::abs(ref_deg - held_ref_deg_) <= kRefEpsDeg) {
            chart_->setRefLive(true);
            return;
        }

        // Create a proper step: hold old value until this time, then jump.
        chart_->appendRef(t_sec, held_ref_deg_);
        updateHeldRef_(ref_deg);
        chart_->appendRef(t_sec, ref_deg);
        chart_->setRefLive(true);
        return;
    }

    if (*signal != TelemetrySignalType::Angle) {
        return;
    }

    const double arm_deg = mapJointRadToUiDeg(sample.arm_angle_rad);

    if (sample.domain == Domain::Real || sample.domain == Domain::Sim) {
        chart_->append(t_sec, arm_deg, sample.domain);

        // Extend the held reference using only ONE timing source:
        // prefer REAL; use SIM only when REAL is not live.
        const bool extend_ref =
            (sample.domain == Domain::Real) ||
            (sample.domain == Domain::Sim && !live_real_);

        if (extend_ref) {
            appendHeldRefAt_(t_sec);
        }
    }
}

void AngleRefPlot::setStreamLive(davinci_arm::models::Domain domain, bool live)
{
    if (!chart_) {
        return;
    }

    if (domain == Domain::Real) {
        live_real_ = live;
        chart_->setRealLive(live);
    } else if (domain == Domain::Sim) {
        live_sim_ = live;
        chart_->setSimLive(live);
    } else if (domain == Domain::Ref) {
        chart_->setRefLive(live);
        return;
    }

    chart_->setRefLive(have_ref_);
}

void AngleRefPlot::clear()
{
    have_t0_ = false;
    t0_ = {};
    live_real_ = false;
    live_sim_ = false;
    have_ref_ = false;
    held_ref_deg_ = 0.0;

    if (chart_) {
        chart_->clear();
    }
}

}  // namespace davinci_arm::ui::widgets