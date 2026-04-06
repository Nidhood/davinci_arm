#pragma once

#include <QVector>
#include <QWidget>
#include <QString>

#include <array>
#include <cstddef>
#include <cstdint>

#include "davinci_arm_gui/core/charts/sample_rate_estimator.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

QT_BEGIN_NAMESPACE
class QButtonGroup;
class QDoubleSpinBox;
class QFrame;
class QProgressBar;
class QPushButton;
class QSlider;
QT_END_NAMESPACE

namespace davinci_arm::core::services {
class RecorderService;
}

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::ui::widgets {
class AngleRefPlot;
class TrackingErrorPlot;
}

namespace davinci_arm::ui::pages {

class ControlPanelPage final : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanelPage(QWidget* parent = nullptr);

    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setRecorderService(davinci_arm::core::services::RecorderService* recorder);

    void onTelemetry(const davinci_arm::models::TelemetrySample& sample);
    void setStreamLive(davinci_arm::models::Domain domain, bool live);

signals:
    void stopRequested();
    void refAngleChanged(double rad);
    void jointReferenceChanged(int jointIndex, double deg);
    void jointBatchCommandRequested(const QVector<double>& jointsDeg);
    void startRecordingRequested(double durationSec);
    void stopRecordingRequested();
    void exportRecordingRequested();

private slots:
    void onFocusButtonClicked_(int index);
    void onZeroAllClicked_();
    void onStartRecordingClicked_();
    void onStopRecordingClicked_();
    void onExportRecordingClicked_();

private:
    struct JointWidgets final {
        QPushButton* focus_button{nullptr};
        QSlider* slider{nullptr};
        QDoubleSpinBox* spin{nullptr};
    };

    void buildUi_();
    QWidget* buildLeftColumn_();
    QWidget* buildRightColumn_();
    QFrame* buildReferencePanel_();
    QFrame* buildRecordingPanel_();
    void connectSignals_();

    void applyAngleRanges_();
    void updateRecordingUi_();
    void updateChartTitles_();
    void updateAdaptiveDensity_(davinci_arm::models::Domain domain, double t_sec) noexcept;

    void setActiveJointIndex_(int index);
    int activeJointIndex_() const noexcept;

    QVector<double> collectJointRefsDeg_() const;
    void publishJointReference_(int jointIndex, double deg, bool force);
    void setJointUiDeg_(int jointIndex, double deg);
    void applyAllZero_();

    QString labelForJoint_(int jointIndex) const;
    QString activeJointName_() const;
    int resolveJointIndex_(const davinci_arm::models::TelemetrySample& sample) const noexcept;

private:
    static constexpr std::size_t kJointCount = 5;

    davinci_arm::core::services::RecorderService* recorder_service_{nullptr};
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    QButtonGroup* focus_group_{nullptr};
    std::array<JointWidgets, kJointCount> joint_widgets_{};
    QPushButton* zero_all_btn_{nullptr};

    QDoubleSpinBox* recording_duration_{nullptr};
    QProgressBar* recording_progress_{nullptr};
    QPushButton* start_recording_btn_{nullptr};
    QPushButton* stop_recording_btn_{nullptr};
    QPushButton* export_recording_btn_{nullptr};

    davinci_arm::ui::widgets::AngleRefPlot* angle_ref_plot_{nullptr};
    davinci_arm::ui::widgets::TrackingErrorPlot* error_plot_{nullptr};

    davinci_arm::core::charts::SampleRateEstimator sample_rate_estimator_{};

    bool real_live_{false};
    bool sim_live_{false};
    int active_joint_index_{0};
    std::array<qint64, kJointCount> last_emit_ms_{};
};

}  // namespace davinci_arm::ui::pages