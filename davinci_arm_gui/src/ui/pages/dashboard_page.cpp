#include "davinci_arm_gui/ui/pages/dashboard_page.hpp"

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/telemetry_store.hpp"
#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/angle_ref_plot.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QScrollArea>
#include <QVBoxLayout>

#include <array>

namespace davinci_arm::ui::pages {

namespace {
constexpr int kJointChartMinHeightPx = 430;
constexpr int kOuterMarginPx = 16;
constexpr int kGridSpacingPx = 12;
}  // namespace

DashboardPage::DashboardPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
    wireLimits_();
    setStreamLive(davinci_arm::models::Domain::Real, real_live_);
    setStreamLive(davinci_arm::models::Domain::Sim, sim_live_);
}

void DashboardPage::setTelemetryStore(davinci_arm::models::TelemetryStore* store) noexcept {
    store_ = store;
}

void DashboardPage::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    wireLimits_();
}

void DashboardPage::wireLimits_() noexcept {
    for (auto* plot : joint_plots_) {
        if (plot) {
            plot->setLimitsRegistry(limits_);
        }
    }
}

void DashboardPage::buildUi_() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(kOuterMarginPx, kOuterMarginPx, 10, kOuterMarginPx);
    root->setSpacing(0);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setFrameShape(QFrame::NoFrame);
    scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area_->setObjectName("dashboardScrollArea");
    scroll_area_->setStyleSheet(
        "QScrollArea { background: transparent; border: none; }"
        "QScrollArea > QWidget > QWidget { background: transparent; }"
        "QScrollBar:vertical {"
        "  background: rgba(255,255,255,0.04);"
        "  width: 10px;"
        "  margin: 0px;"
        "  border-radius: 5px;"
        "}"
        "QScrollBar::handle:vertical {"
        "  background: rgba(99,168,255,0.78);"
        "  min-height: 36px;"
        "  border-radius: 5px;"
        "}"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }"
        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }");

    scroll_area_->setWidget(buildScrollContent_());
    root->addWidget(scroll_area_, 1);
}

QWidget* DashboardPage::buildScrollContent_() {
    auto* content = new QWidget(scroll_area_);
    auto* root = new QVBoxLayout(content);
    root->setContentsMargins(0, 0, 4, 0);
    root->setSpacing(kGridSpacingPx);

    auto* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(kGridSpacingPx);
    grid->setVerticalSpacing(kGridSpacingPx);

    static const std::array<QString, 6> joint_titles = {
        "Joint 1 · Base angle",
        "Joint 2 · Shoulder angle",
        "Joint 3 · Elbow angle",
        "Joint 4 · Wrist Roll angle",
        "Joint 5 · Wrist Pitch angle",
        "Joint 6 · Tool angle"
    };

    for (std::size_t i = 0; i < joint_titles.size(); ++i) {
        auto* plot = buildJointPlot_(i, joint_titles[i], content);
        grid->addWidget(plot, static_cast<int>(i / 2), static_cast<int>(i % 2));
    }

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    root->addLayout(grid);
    root->addStretch(1);
    return content;
}

QWidget* DashboardPage::buildJointPlot_(std::size_t joint_index, const QString& title, QWidget* parent) {
    auto* plot = new davinci_arm::ui::widgets::AngleRefPlot(parent);
    plot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    plot->setMinimumHeight(kJointChartMinHeightPx);
    plot->setContentsMargins(0, 0, 0, 0);

    if (auto* chart = plot->findChild<davinci_arm::ui::widgets::ChartBase*>()) {
        chart->setTitle(title);
        chart->setSeriesLabels("Real", "Sim", "Ref");
        chart->setShowSim(true);
        chart->setShowRef(true);
        chart->setWindowSeconds(30.0);
        chart->setMaxPoints(2200);
    }

    joint_plots_[joint_index] = plot;
    return plot;
}

void DashboardPage::onTelemetry(const davinci_arm::models::TelemetrySample& sample) {
    if (!sample.valid) {
        return;
    }

    for (auto* plot : joint_plots_) {
        if (plot) {
            plot->pushSample(sample);
        }
    }
}

void DashboardPage::setStreamLive(davinci_arm::models::Domain domain, bool live) {
    if (domain == davinci_arm::models::Domain::Real) {
        real_live_ = live;
    } else if (domain == davinci_arm::models::Domain::Sim) {
        sim_live_ = live;
    }

    for (auto* plot : joint_plots_) {
        if (plot) {
            plot->setStreamLive(domain, live);
        }
    }
}

}  // namespace davinci_arm::ui::pages
