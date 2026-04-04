#pragma once

#include "davinci_arm_gui/core/charts/sample_rate_estimator.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"

#include <QWidget>

#include <array>
#include <cstddef>

class QScrollArea;

namespace davinci_arm::models {
class TelemetryStore;
struct TelemetrySample;
}

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::ui::widgets {
class AngleRefPlot;
}

namespace davinci_arm::ui::pages {

class DashboardPage final : public QWidget {
    Q_OBJECT

public:
    static constexpr std::size_t kJointCount = 5;

    explicit DashboardPage(QWidget* parent = nullptr);

    void setTelemetryStore(davinci_arm::models::TelemetryStore* store) noexcept;
    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void onTelemetry(const davinci_arm::models::TelemetrySample& sample);
    void setStreamLive(davinci_arm::models::Domain domain, bool live);

private:
    void buildUi_();
    QWidget* buildScrollContent_();
    QWidget* buildJointPlot_(std::size_t joint_index, const QString& title, QWidget* parent);
    void wireLimits_() noexcept;
    [[nodiscard]] int jointIndexOf_(const davinci_arm::models::TelemetrySample& sample) const noexcept;
    void updateAdaptiveDensity_(davinci_arm::models::Domain domain, double t_sec) noexcept;

private:
    davinci_arm::models::TelemetryStore* store_{nullptr};
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    QScrollArea* scroll_area_{nullptr};
    std::array<davinci_arm::ui::widgets::AngleRefPlot*, kJointCount> joint_plots_{};

    davinci_arm::core::charts::SampleRateEstimator sample_rate_estimator_{};

    bool real_live_{true};
    bool sim_live_{true};
};

}  // namespace davinci_arm::ui::pages
