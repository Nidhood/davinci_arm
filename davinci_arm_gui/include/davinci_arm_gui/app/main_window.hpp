#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QMetaObject>
#include <QStackedWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVector>
#include <qtabbar.h>

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/theme_id.hpp"

class QLabel;

namespace davinci_arm::core::services {
class RecorderService;
}

namespace davinci_arm::models {
class TelemetryStore;
}

namespace davinci_arm::infra::ros {
class LimitsRegistry;
}

namespace davinci_arm::services {
class CalibrationService;
}

namespace davinci_arm::ui::pages {
class CalibrationPage;
class ControlPanelPage;
class DashboardPage;
class DrawingPage;
}

namespace davinci_arm::app {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

    void setTelemetryStore(davinci_arm::models::TelemetryStore* store);
    void setRecorderService(davinci_arm::core::services::RecorderService* recorder);
    void setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits);
    void setCalibrationService(davinci_arm::services::CalibrationService* calibration_service);

    void setConnected(bool connected);
    void setControlMode(const QString& mode);
    void setSystemStatus(const QString& status);

    void onTelemetry(const davinci_arm::models::TelemetrySample& sample);
    void setStreamLive(davinci_arm::models::Domain domain, bool live);

signals:
    void refAngleChanged(double rad);
    void pwmChanged(std::uint16_t pwm_us);
    void autoModeChanged(bool enabled);
    void stopRequested();
    void jointBatchCommandRequested(const QVector<double>& jointsDeg);

protected:
    void showEvent(QShowEvent* event) override;

private:
    void buildUi_();
    void buildTopBar_();
    void connectPageSignals_();
    void setupStatusBar_();
    void setupScreenTracking_();
    void updateModeIndicator_(bool auto_mode);

    void onNavChanged_(int row);
    void onThemeAction_();
    void onThemeChanged_(davinci_arm::models::ThemeId id);

    void applyResponsiveInitialGeometry_();
    void clampToCurrentScreen_();

    void updateStatusBarStyle_();
    void updateStatusLabels_();
    void updateTopBarStyle_();

private:
    QStackedWidget* stack_{nullptr};
    QToolBar* topbar_{nullptr};
    QToolButton* theme_btn_{nullptr};
    QTabBar* top_tabs_{nullptr};

    davinci_arm::ui::pages::DashboardPage* dashboard_{nullptr};
    davinci_arm::ui::pages::ControlPanelPage* control_{nullptr};
    davinci_arm::ui::pages::CalibrationPage* calibration_{nullptr};
    davinci_arm::ui::pages::DrawingPage* drawing_{nullptr};

    davinci_arm::models::TelemetryStore* store_{nullptr};
    davinci_arm::core::services::RecorderService* recorder_{nullptr};
    const davinci_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    davinci_arm::services::CalibrationService* calibration_service_{nullptr};

    QLabel* status_connection_{nullptr};
    QLabel* status_mode_{nullptr};
    QLabel* status_real_{nullptr};
    QLabel* status_sim_{nullptr};
    QTimer* status_timer_{nullptr};

    bool overall_connected_{false};
    bool seen_real_{false};
    bool seen_sim_{false};
    QElapsedTimer real_timer_;
    QElapsedTimer sim_timer_;

    bool initial_geometry_applied_{false};
    QMetaObject::Connection avail_geom_conn_;
};

}  // namespace davinci_arm::app
