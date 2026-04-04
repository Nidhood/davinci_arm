#pragma once

#include <QWidget>

#include <optional>

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/ui/style/theme.hpp"

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::ui::widgets {
class ChartBase;

class ErrorPlot final : public QWidget {
    Q_OBJECT

public:
    explicit ErrorPlot(QWidget* parent = nullptr);

    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setStreamLive(davinci_arm::models::Domain domain, bool live);
    void pushSample(const davinci_arm::models::TelemetrySample& sample);
    void clear();
    void applyTheme(const davinci_arm::ui::style::ThemeSpec& spec);

private:
    double toSecSinceStart_(const std::chrono::steady_clock::time_point& tp);
    void tryEmitError_();

private:
    ChartBase* chart_{nullptr};
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    bool real_live_{false};
    bool sim_live_{false};
    bool have_real_{false};
    bool have_sim_{false};
    std::optional<std::chrono::steady_clock::time_point> t0_{};
    std::chrono::steady_clock::time_point tp_real_{};
    std::chrono::steady_clock::time_point tp_sim_{};
    double a_real_{0.0};
    double a_sim_{0.0};
    double sync_tol_s_{0.05};
};

}  // namespace davinci_arm::ui::widgets
