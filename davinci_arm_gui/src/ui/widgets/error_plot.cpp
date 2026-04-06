#include "davinci_arm_gui/ui/widgets/error_plot.hpp"

#include "davinci_arm_gui/core/math/units.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"

#include <QVBoxLayout>

#include <cmath>
#include <numbers>

namespace davinci_arm::ui::widgets {

namespace {

double wrapRadPi(double rad)
{
    constexpr double pi = std::numbers::pi;
    constexpr double two_pi = 2.0 * pi;

    while (rad <= -pi) {
        rad += two_pi;
    }
    while (rad > pi) {
        rad -= two_pi;
    }
    return rad;
}

}  // namespace

ErrorPlot::ErrorPlot(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    chart_ = new ChartBase("", "abs error (deg)", this);
    chart_->setObjectName("chartWidget");
    chart_->setShowSim(false);
    chart_->setShowRef(false);
    chart_->setAutoRange(true);
    chart_->setSeriesLabels("Abs(Real-Sim)", "", "");

    layout->addWidget(chart_, 1);
}

void ErrorPlot::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept
{
    limits_ = limits;
    (void)limits_;
}

void ErrorPlot::setStreamLive(davinci_arm::models::Domain domain, bool live)
{
    if (domain == davinci_arm::models::Domain::Real) {
        real_live_ = live;
    }
    if (domain == davinci_arm::models::Domain::Sim) {
        sim_live_ = live;
    }

    const bool err_live = real_live_ && sim_live_;
    chart_->setRealLive(err_live);
}

double ErrorPlot::toSecSinceStart_(const std::chrono::steady_clock::time_point& tp)
{
    if (!t0_.has_value()) {
        t0_ = tp;
    }
    return std::chrono::duration<double>(tp - *t0_).count();
}

void ErrorPlot::pushSample(const davinci_arm::models::TelemetrySample& sample)
{
    if (!sample.valid || !std::isfinite(sample.arm_angle_rad)) {
        return;
    }

    (void)toSecSinceStart_(sample.t);

    if (sample.domain == davinci_arm::models::Domain::Real) {
        have_real_ = true;
        tp_real_ = sample.t;
        a_real_ = sample.arm_angle_rad;
        tryEmitError_();
    } else if (sample.domain == davinci_arm::models::Domain::Sim) {
        have_sim_ = true;
        tp_sim_ = sample.t;
        a_sim_ = sample.arm_angle_rad;
        tryEmitError_();
    }
}

void ErrorPlot::tryEmitError_()
{
    if (!(real_live_ && sim_live_)) {
        return;
    }
    if (!(have_real_ && have_sim_)) {
        return;
    }

    const double dt = std::abs(std::chrono::duration<double>(tp_real_ - tp_sim_).count());
    if (dt > sync_tol_s_) {
        return;
    }

    const double err_rad = std::fabs(wrapRadPi(a_real_ - a_sim_));
    const double err_deg = davinci_arm::core::math::rad_to_deg(err_rad);
    const auto tp = (tp_real_ > tp_sim_) ? tp_real_ : tp_sim_;
    const double t = toSecSinceStart_(tp);

    chart_->append(t, err_deg, davinci_arm::models::Domain::Real);
}

void ErrorPlot::clear()
{
    have_real_ = false;
    have_sim_ = false;
    t0_.reset();
    if (chart_) {
        chart_->clear();
    }
}

void ErrorPlot::applyTheme(const davinci_arm::ui::style::ThemeSpec& spec)
{
    if (chart_) {
        chart_->applyTheme(spec);
    }
}

}  // namespace davinci_arm::ui::widgets