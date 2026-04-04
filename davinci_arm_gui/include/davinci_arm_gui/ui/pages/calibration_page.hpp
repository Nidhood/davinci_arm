#pragma once

#include <QWidget>

#include "davinci_arm_gui/core/models/calibration_types.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

#include <array>
#include <cstddef>

QT_BEGIN_NAMESPACE
class QButtonGroup;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QTableWidget;
QT_END_NAMESPACE

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::services {
class CalibrationService;
}

namespace davinci_arm::ui::widgets {
class AngleRefPlot;
class ErrorPlot;
class ChartBase;
}

namespace davinci_arm::ui::pages {

class CalibrationPage final : public QWidget {
    Q_OBJECT

public:
    explicit CalibrationPage(QWidget* parent = nullptr);

    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setCalibrationService(davinci_arm::services::CalibrationService* service) noexcept;

    void setStreamLive(davinci_arm::models::Domain domain, bool live);
    void onTelemetry(const davinci_arm::models::TelemetrySample& sample);

signals:
    void startCalibrationRequested(davinci_arm::models::CalibrationConfig cfg);
    void stopCalibrationRequested();
    void applyParametersRequested();
    void resetCalibrationRequested();

private slots:
    void onFocusButtonClicked_(int index);
    void onStartClicked_();
    void onStopClicked_();
    void onApplyClicked_();
    void onResetClicked_();

private:
    static constexpr std::size_t kJointCount = 5;

    void buildUi_();
    QWidget* buildLeftColumn_();
    QWidget* buildRightColumn_();
    QWidget* buildFocusPanel_();
    QWidget* buildConfigPanel_();
    QWidget* buildActionPanel_();
    QWidget* buildParameterPanel_();
    QWidget* buildLogPanel_();

    void connectSignals_();
    void applyAngleRanges_();
    void updateChartTitles_();
    void updateStatusUi_(const QString& status, int progress_pct);
    void updateParameterEstimates_(double real_deg, double ref_deg);
    int activeJointIndex_() const noexcept;
    QString labelForJoint_(int joint_index) const;
    [[nodiscard]] davinci_arm::models::CalibrationConfig buildConfig_() const;

private:
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    davinci_arm::services::CalibrationService* calibration_service_{nullptr};

    QButtonGroup* focus_group_{nullptr};
    std::array<QPushButton*, kJointCount> focus_buttons_{};
    int active_joint_index_{0};

    QLabel* status_value_{nullptr};
    QProgressBar* progress_bar_{nullptr};

    QComboBox* calibration_type_{nullptr};
    QComboBox* excitation_profile_{nullptr};
    QDoubleSpinBox* amplitude_spin_{nullptr};
    QDoubleSpinBox* duration_spin_{nullptr};
    QDoubleSpinBox* repetitions_spin_{nullptr};

    QTableWidget* identified_params_table_{nullptr};
    QPlainTextEdit* notes_log_{nullptr};

    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QPushButton* apply_btn_{nullptr};
    QPushButton* reset_btn_{nullptr};

    davinci_arm::ui::widgets::AngleRefPlot* angle_ref_plot_{nullptr};
    davinci_arm::ui::widgets::ErrorPlot* error_plot_{nullptr};

    bool real_live_{false};
    bool sim_live_{false};
};

}  // namespace davinci_arm::ui::pages
