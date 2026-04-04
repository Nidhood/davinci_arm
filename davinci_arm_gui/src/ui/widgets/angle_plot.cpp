#include "davinci_arm_gui/ui/widgets/angle_plot.hpp"

#include "davinci_arm_gui/core/math/units.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"

#include <QFrame>
#include <QVBoxLayout>

namespace davinci_arm::ui::widgets {

AnglePlot::AnglePlot(QWidget* parent)
    : QWidget(parent) {
    root_ = new QVBoxLayout(this);
    root_->setContentsMargins(0, 0, 0, 0);
    root_->setSpacing(0);

    chart_ = new ChartBase("", "deg", this);
    chart_->setSeriesLabels("Real", "Sim", "Ref");
    chart_->setShowSim(true);
    chart_->setShowRef(true);
    chart_->setIntegerYAxis(false);
    chart_->setAutoRange(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(2200);
    chart_->setRealLive(false);
    chart_->setSimLive(false);
    chart_->setRefLive(false);

    frame_ = static_cast<QFrame*>(makeChartFrame_(chart_, "rtpChartFrame"));
    root_->addWidget(frame_, 1);
}

QWidget* AnglePlot::makeChartFrame_(QWidget* child, const char* object_name) {
    auto* frame = new QFrame();
    frame->setObjectName(object_name);
    frame->setFrameShape(QFrame::NoFrame);

    auto* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(child, 1);
    return frame;
}

std::chrono::steady_clock::time_point AnglePlot::sampleTime_(const davinci_arm::models::TelemetrySample& s) {
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void AnglePlot::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    if (limits_ && chart_) {
        const auto& r = limits_->angleLimits();
        chart_->setYRange(r.min, r.max);
    }
}

void AnglePlot::setStreamLive(davinci_arm::models::Domain domain, bool live) {
    if (!chart_) {
        return;
    }

    if (domain == davinci_arm::models::Domain::Real) {
        chart_->setRealLive(live);
    } else if (domain == davinci_arm::models::Domain::Sim) {
        chart_->setSimLive(live);
    } else if (domain == davinci_arm::models::Domain::Ref) {
        chart_->setRefLive(live);
    }
}

void AnglePlot::pushSample(const davinci_arm::models::TelemetrySample& sample) {
    if (!sample.valid || !chart_) {
        return;
    }

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();
    const double angle_deg = davinci_arm::core::math::rad_to_deg(sample.arm_angle_rad);
    const double ref_deg = davinci_arm::core::math::rad_to_deg(sample.ref_angle_rad);

    if (sample.domain == davinci_arm::models::Domain::Real ||
            sample.domain == davinci_arm::models::Domain::Sim) {
        chart_->append(t_sec, angle_deg, sample.domain);
    }

    chart_->appendRef(t_sec, ref_deg);
}

void AnglePlot::clear() {
    have_t0_ = false;
    if (chart_) {
        chart_->clear();
    }
}

}  // namespace davinci_arm::ui::widgets
