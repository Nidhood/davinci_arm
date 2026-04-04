#pragma once

#include <QWidget>
#include <chrono>

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::models {
struct TelemetrySample;
enum class Domain;
}

namespace davinci_arm::ui::widgets {
class ChartBase;

class AngleRefPlot final : public QWidget {
    Q_OBJECT

public:
    explicit AngleRefPlot(QWidget* parent = nullptr);

    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setStreamLive(davinci_arm::models::Domain domain, bool live);
    void pushSample(const davinci_arm::models::TelemetrySample& sample);
    void clear();

private:
    void buildUi_();
    void applyLimits_();
    static std::chrono::steady_clock::time_point sampleTime_(const davinci_arm::models::TelemetrySample& s);
    void updateHeldRef_(davinci_arm::models::Domain domain, double ref_deg);
    void appendHeldRefAt_(double t_sec);

private:
    ChartBase* chart_{nullptr};
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    bool have_t0_{false};
    std::chrono::steady_clock::time_point t0_{};
    bool live_real_{false};
    bool live_sim_{false};
    bool have_real_ref_{false};
    bool have_ref_{false};
    double held_ref_deg_{0.0};
};

}  // namespace davinci_arm::ui::widgets
