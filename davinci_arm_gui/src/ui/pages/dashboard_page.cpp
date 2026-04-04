#include "davinci_arm_gui/ui/pages/dashboard_page.hpp"

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"
#include "davinci_arm_gui/core/models/telemetry_store.hpp"
#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/angle_ref_plot.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QScrollArea>
#include <QVBoxLayout>

#include <array>
#include <optional>
#include <string>
#include <type_traits>

namespace davinci_arm::ui::pages {

namespace {

using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;

constexpr int kJointChartMinHeightPx = 430;
constexpr int kOuterMarginPx = 16;
constexpr int kGridSpacingPx = 12;
constexpr double kChartWindowSeconds = 30.0;

constexpr std::array<const char*, DashboardPage::kJointCount> kJointNames {{
        "shoulder_link_joint",
        "bicep_link_joint",
        "arm_link_joint",
        "wrist_link_joint",
        "end_effector_link_joint",
    }};

constexpr std::array<const char*, DashboardPage::kJointCount> kJointTitles {{
        "Shoulder angle",
        "Bicep angle",
        "Arm angle",
        "Wrist angle",
        "End effector angle",
    }};

template<typename T>
std::optional<std::string> jointNameOf(const T& sample)
{
    if constexpr (requires { sample.joint_name; }) {
        return sample.joint_name;
    } else if constexpr (requires { sample.label; }) {
        return sample.label;
    } else if constexpr (requires { sample.series; }) {
        return sample.series;
    } else if constexpr (requires { sample.channel; }) {
        return sample.channel;
    } else {
        return std::nullopt;
    }
}

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

template<typename T>
std::optional<double> timeSecOf(const T& sample)
{
    if constexpr (requires { sample.time_sec; }) {
        return static_cast<double>(sample.time_sec);
    } else if constexpr (requires { sample.timestamp_sec; }) {
        return static_cast<double>(sample.timestamp_sec);
    } else if constexpr (requires { sample.t_sec; }) {
        return static_cast<double>(sample.t_sec);
    } else if constexpr (requires { sample.time_s; }) {
        return static_cast<double>(sample.time_s);
    } else if constexpr (requires { sample.t; }) {
        using U = std::decay_t<decltype(sample.t)>;
        if constexpr (std::is_arithmetic_v<U>) {
            return static_cast<double>(sample.t);
        } else {
            return std::nullopt;
        }
    } else {
        return std::nullopt;
    }
}

davinci_arm::ui::widgets::ChartBase* chartOf(QWidget* host)
{
    if (!host) {
        return nullptr;
    }
    return host->findChild<davinci_arm::ui::widgets::ChartBase*>();
}

}  // namespace

DashboardPage::DashboardPage(QWidget* parent)
    : QWidget(parent)
{
    buildUi_();
    wireLimits_();
    setStreamLive(Domain::Real, real_live_);
    setStreamLive(Domain::Sim, sim_live_);
}

void DashboardPage::setTelemetryStore(davinci_arm::models::TelemetryStore* store) noexcept
{
    store_ = store;
}

void DashboardPage::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept
{
    limits_ = limits;
    wireLimits_();
}

void DashboardPage::wireLimits_() noexcept
{
    for (auto* plot : joint_plots_) {
        if (plot) {
            plot->setLimitsRegistry(limits_);
        }
    }
}

void DashboardPage::buildUi_()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(kOuterMarginPx, kOuterMarginPx, 10, kOuterMarginPx);
    root->setSpacing(0);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setFrameShape(QFrame::NoFrame);
    scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area_->setObjectName("dashboardScrollArea");

    scroll_area_->setWidget(buildScrollContent_());
    root->addWidget(scroll_area_, 1);
}

QWidget* DashboardPage::buildScrollContent_()
{
    auto* content = new QWidget(scroll_area_);
    auto* root = new QVBoxLayout(content);
    root->setContentsMargins(0, 0, 4, 0);
    root->setSpacing(kGridSpacingPx);

    auto* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(kGridSpacingPx);
    grid->setVerticalSpacing(kGridSpacingPx);

    for (std::size_t i = 0; i < kJointTitles.size(); ++i) {
        auto* plot = buildJointPlot_(i, QString::fromUtf8(kJointTitles[i]), content);
        grid->addWidget(plot, static_cast<int>(i / 2), static_cast<int>(i % 2));
    }

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    root->addLayout(grid);
    root->addStretch(1);
    return content;
}

QWidget* DashboardPage::buildJointPlot_(std::size_t joint_index, const QString& title, QWidget* parent)
{
    auto* plot = new davinci_arm::ui::widgets::AngleRefPlot(parent);
    plot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    plot->setMinimumHeight(kJointChartMinHeightPx);

    if (auto* chart = chartOf(plot)) {
        chart->setTitle(title);
        chart->setSeriesLabels("Real", "Sim", "Ref");
        chart->setShowSim(true);
        chart->setShowRef(true);
        chart->setWindowSeconds(kChartWindowSeconds);
        chart->setMaxPoints(2200);
    }

    joint_plots_[joint_index] = plot;
    return plot;
}

int DashboardPage::jointIndexOf_(const davinci_arm::models::TelemetrySample& sample) const noexcept
{
    const auto sample_joint = jointNameOf(sample);
    if (!sample_joint.has_value()) {
        return -1;
    }

    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        if (*sample_joint == kJointNames[i]) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

void DashboardPage::updateAdaptiveDensity_(Domain domain, double t_sec) noexcept
{
    sample_rate_estimator_.observe(domain, t_sec);
    const auto dt = sample_rate_estimator_.dtEma(domain);
    if (!dt.has_value()) {
        return;
    }

    const int max_points =
        davinci_arm::core::charts::SampleRateEstimator::recommendedMaxPoints(
            kChartWindowSeconds,
            *dt);

    for (auto* plot : joint_plots_) {
        if (auto* chart = chartOf(plot)) {
            chart->setMaxPoints(max_points);
        }
    }
}

void DashboardPage::onTelemetry(const davinci_arm::models::TelemetrySample& sample)
{
    if (!sample.valid) {
        return;
    }

    const auto signal = signalTypeOf(sample);
    if (!signal.has_value()) {
        return;
    }

    if (*signal != TelemetrySignalType::Angle && *signal != TelemetrySignalType::AngleRef) {
        return;
    }

    const int joint_index = jointIndexOf_(sample);
    if (joint_index < 0 || joint_index >= static_cast<int>(joint_plots_.size())) {
        return;
    }

    if (const auto t_sec = timeSecOf(sample); t_sec.has_value()) {
        updateAdaptiveDensity_(sample.domain, *t_sec);
    }

    if (auto* plot = joint_plots_[static_cast<std::size_t>(joint_index)]) {
        plot->pushSample(sample);
    }
}

void DashboardPage::setStreamLive(davinci_arm::models::Domain domain, bool live)
{
    if (domain == Domain::Real) {
        real_live_ = live;
    } else if (domain == Domain::Sim) {
        sim_live_ = live;
    }

    for (auto* plot : joint_plots_) {
        if (plot) {
            plot->setStreamLive(domain, live);
        }
    }
}

}  // namespace davinci_arm::ui::pages
