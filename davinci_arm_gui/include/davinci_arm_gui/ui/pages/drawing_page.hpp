#pragma once

#include <QWidget>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QTableWidget;
class QTabWidget;

namespace davinci_arm::ui::pages {

class SheetPreviewWidget;

class DrawingPage final : public QWidget {
    Q_OBJECT

public:
    explicit DrawingPage(QWidget* parent = nullptr);

    void setWorkspaceReady(bool ready);
    void setPlanningStatus(const QString& text);
    void setExecutionStatus(const QString& text);
    void setPlanningProgress(int percent);
    void setExecutionProgress(int percent);
    void setCanvasPlaceholderText(const QString& text);
    void setSheetSizeCm(double width_cm, double height_cm);

signals:
    void importSvgRequested();
    void clearCanvasRequested();
    void previewTrajectoryRequested();
    void generateTrajectoryRequested();
    void sendToPlannerRequested();
    void stopOperationRequested();
    void finishOperationRequested();
    void exportWaypointsRequested();

private:
    void buildUi_();
    QWidget* buildWorkspaceColumn_();
    QWidget* buildControlsColumn_();
    QWidget* buildPlanTab_();
    QWidget* buildWaypointsTab_();
    QWidget* buildProgressSection_();
    QWidget* buildToolsSection_(QWidget* parent);
    QWidget* buildPlanningSection_(QWidget* parent);
    QWidget* buildPlanActionsSection_(QWidget* parent);
    void wireSignals_();
    void updateStatusChips_();
    void updateSheetSizeFromUi_();

private:
    bool workspace_ready_{true};
    QString planning_status_text_{"Idle"};
    QString execution_status_text_{"Idle"};

    QLabel* workspace_chip_{nullptr};
    QLabel* planning_chip_{nullptr};
    QLabel* execution_chip_{nullptr};
    SheetPreviewWidget* sheet_preview_{nullptr};

    QProgressBar* planning_progress_{nullptr};
    QProgressBar* execution_progress_{nullptr};
    QLabel* planning_percent_label_{nullptr};
    QLabel* execution_percent_label_{nullptr};

    QTabWidget* side_tabs_{nullptr};
    QButtonGroup* tool_group_{nullptr};
    QPushButton* freehand_btn_{nullptr};
    QPushButton* line_btn_{nullptr};
    QPushButton* spline_btn_{nullptr};
    QPushButton* circle_btn_{nullptr};
    QPushButton* erase_btn_{nullptr};

    QComboBox* plane_selector_{nullptr};
    QDoubleSpinBox* scale_spin_{nullptr};
    QDoubleSpinBox* sample_step_spin_{nullptr};
    QDoubleSpinBox* speed_spin_{nullptr};
    QDoubleSpinBox* z_offset_spin_{nullptr};
    QComboBox* orientation_selector_{nullptr};
    QCheckBox* close_path_check_{nullptr};
    QCheckBox* snap_grid_check_{nullptr};
    QDoubleSpinBox* sheet_width_spin_{nullptr};
    QDoubleSpinBox* sheet_height_spin_{nullptr};

    QPushButton* import_svg_btn_{nullptr};
    QPushButton* clear_btn_{nullptr};
    QPushButton* preview_btn_{nullptr};
    QPushButton* generate_btn_{nullptr};
    QPushButton* stop_operation_btn_{nullptr};
    QPushButton* finish_operation_btn_{nullptr};
    QPushButton* export_btn_{nullptr};

    QLabel* waypoint_summary_{nullptr};
    QTableWidget* waypoint_table_{nullptr};
};

}  // namespace davinci_arm::ui::pages
