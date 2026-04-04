#include "davinci_arm_gui/ui/pages/drawing_page.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace davinci_arm::ui::pages {

class SheetPreviewWidget final : public QWidget {
public:
    explicit SheetPreviewWidget(QWidget* parent = nullptr)
        : QWidget(parent) {
        setMinimumHeight(640);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setSheetSizeCm(double width_cm, double height_cm) {
        sheet_width_cm_ = std::clamp(width_cm, 1.0, 500.0);
        sheet_height_cm_ = std::clamp(height_cm, 1.0, 500.0);
        update();
    }

    void setOverlayText(const QString& text) {
        overlay_text_ = text;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    double sheet_width_cm_{21.0};
    double sheet_height_cm_{29.7};
    QString overlay_text_{QStringLiteral("Attach top-view robot overlay and drawing preview here")};
};

namespace {

QWidget* makeSectionBox(const QString& title, QWidget* content, QWidget* parent) {
    auto* box = new QGroupBox(title, parent);
    box->setStyleSheet(
        "QGroupBox {"
        "border: 1px solid rgba(255,255,255,0.10);"
        "border-radius: 14px;"
        "margin-top: 10px;"
        "padding-top: 10px;"
        "background: rgba(255,255,255,0.02);"
        "}"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; font-weight: 700; }");

    auto* layout = new QVBoxLayout(box);
    layout->setContentsMargins(14, 16, 14, 14);
    layout->addWidget(content);
    return box;
}

QString chipStyle(const QString& background) {
    return QString(
               "QLabel {"
               "padding: 5px 12px;"
               "border-radius: 11px;"
               "font-weight: 800;"
               "letter-spacing: 0.4px;"
               "background: %1;"
               "color: white;"
               "}")
           .arg(background);
}

QLabel* makeChip(const QString& text, const QString& background, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumHeight(26);
    label->setStyleSheet(chipStyle(background));
    return label;
}

QString toolButtonStyle() {
    return QStringLiteral(
               "QPushButton {"
               "  min-height: 46px;"
               "  border-radius: 11px;"
               "  border: 1px solid rgba(0,255,255,0.55);"
               "  background: rgba(10, 18, 40, 0.70);"
               "  color: white;"
               "  font-weight: 700;"
               "  padding: 8px 12px;"
               "}"
               "QPushButton:hover {"
               "  border: 1px solid rgba(0,255,255,0.9);"
               "  background: rgba(18, 30, 60, 0.82);"
               "}"
               "QPushButton:checked {"
               "  background: rgba(123, 97, 255, 0.38);"
               "  border: 1px solid rgba(123, 97, 255, 1.0);"
               "}");
}

QString actionButtonStyle(const QString& border, const QString& fill_hover) {
    return QString(
               "QPushButton {"
               "  min-height: 42px;"
               "  border-radius: 10px;"
               "  border: 1px solid %1;"
               "  background: rgba(10, 18, 40, 0.72);"
               "  color: white;"
               "  font-weight: 800;"
               "  letter-spacing: 0.6px;"
               "  padding: 6px 12px;"
               "}"
               "QPushButton:hover {"
               "  background: %2;"
               "}")
           .arg(border, fill_hover);
}

void styleProgressBar(QProgressBar* bar, const QString& chunk_color) {
    if (!bar) return;
    bar->setTextVisible(false);
    bar->setMinimumHeight(14);
    bar->setMaximumHeight(14);
    bar->setStyleSheet(QString(
                           "QProgressBar {"
                           "  border: 1px solid rgba(255,255,255,0.12);"
                           "  border-radius: 7px;"
                           "  background: rgba(255,255,255,0.05);"
                           "}"
                           "QProgressBar::chunk {"
                           "  border-radius: 7px;"
                           "  background: %1;"
                           "}")
                       .arg(chunk_color));
}

void styleField(QWidget* w) {
    if (!w) return;
    w->setMinimumHeight(38);
    w->setStyleSheet(
        "background: rgba(10,18,40,0.72);"
        "border: 1px solid rgba(255,255,255,0.10);"
        "border-radius: 9px;"
        "padding: 0 8px;");
}

QLabel* makeFieldLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setStyleSheet("font-weight: 700; color: rgba(255,255,255,0.90);");
    return l;
}

void drawArrowHead(QPainter& painter, const QPointF& tip, const QPointF& dir, const QColor& color) {
    const double len = std::hypot(dir.x(), dir.y());
    if (len < 1e-6) return;

    const QPointF u(dir.x() / len, dir.y() / len);
    const QPointF n(-u.y(), u.x());

    QPolygonF tri;
    tri << tip
        << (tip - u * 10.0 + n * 4.5)
        << (tip - u * 10.0 - n * 4.5);

    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    painter.drawPolygon(tri);
    painter.restore();
}

}  // namespace

void SheetPreviewWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const QRectF outer = rect().adjusted(4.0, 4.0, -4.0, -4.0);

    painter.setPen(QPen(QColor(255, 255, 255, 24), 1.0));
    painter.setBrush(QColor(3, 10, 32, 220));
    painter.drawRoundedRect(outer, 14.0, 14.0);

    const QRectF content = outer.adjusted(18.0, 18.0, -18.0, -18.0);
    const qreal top_band = 44.0;
    const qreal bottom_band = 48.0;
    const qreal side_band = 44.0;
    const QRectF available = content.adjusted(side_band, top_band, -side_band, -bottom_band);

    if (available.width() <= 10.0 || available.height() <= 10.0) return;

    const double scale = std::min(available.width() / sheet_width_cm_, available.height() / sheet_height_cm_);
    const QSizeF sheet_size(sheet_width_cm_ * scale, sheet_height_cm_ * scale);
    const QRectF sheet_rect(
        available.center().x() - (sheet_size.width() / 2.0),
        available.center().y() - (sheet_size.height() / 2.0),
        sheet_size.width(),
        sheet_size.height());

    const bool fit_by_width = (available.width() / sheet_width_cm_) < (available.height() / sheet_height_cm_);
    const QColor accent = fit_by_width ? QColor("#63A8FF") : QColor("#61D095");

    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 90));
    painter.drawRoundedRect(sheet_rect.translated(8.0, 10.0), 10.0, 10.0);
    painter.restore();

    painter.save();
    painter.setPen(QPen(QColor(190, 210, 255, 110), 1.5));
    painter.setBrush(QColor(245, 248, 255, 232));
    painter.drawRoundedRect(sheet_rect, 12.0, 12.0);
    painter.restore();

    const QRectF printable = sheet_rect.adjusted(sheet_rect.width() * 0.06,
                             sheet_rect.height() * 0.06,
                             -sheet_rect.width() * 0.06,
                             -sheet_rect.height() * 0.06);
    painter.save();
    painter.setPen(QPen(QColor(40, 72, 122, 85), 1.0, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(printable, 8.0, 8.0);

    painter.setPen(QPen(QColor(55, 95, 150, 40), 1.0));
    for (int i = 1; i < 4; ++i) {
        const qreal x = printable.left() + printable.width() * (static_cast<qreal>(i) / 4.0);
        painter.drawLine(QPointF(x, printable.top()), QPointF(x, printable.bottom()));
    }
    for (int i = 1; i < 4; ++i) {
        const qreal y = printable.top() + printable.height() * (static_cast<qreal>(i) / 4.0);
        painter.drawLine(QPointF(printable.left(), y), QPointF(printable.right(), y));
    }
    painter.restore();

    painter.save();
    QFont overlay_font = painter.font();
    overlay_font.setPointSize(22);
    overlay_font.setBold(true);
    painter.setFont(overlay_font);
    painter.setPen(QColor(12, 18, 34, 65));
    painter.drawText(printable.adjusted(28.0, 28.0, -28.0, -28.0), Qt::AlignCenter | Qt::TextWordWrap, overlay_text_);
    painter.restore();

    const qreal top_y = sheet_rect.top() - 18.0;
    painter.save();
    painter.setPen(QPen(QColor(170, 190, 225, 150), 1.2));
    painter.drawLine(QPointF(sheet_rect.left(), top_y), QPointF(sheet_rect.right(), top_y));
    painter.drawLine(QPointF(sheet_rect.left(), top_y - 7.0), QPointF(sheet_rect.left(), top_y + 7.0));
    painter.drawLine(QPointF(sheet_rect.right(), top_y - 7.0), QPointF(sheet_rect.right(), top_y + 7.0));
    drawArrowHead(painter, QPointF(sheet_rect.left(), top_y), QPointF(1.0, 0.0), QColor(170, 190, 225, 180));
    drawArrowHead(painter, QPointF(sheet_rect.right(), top_y), QPointF(-1.0, 0.0), QColor(170, 190, 225, 180));

    QFont dim_font = painter.font();
    dim_font.setPointSize(11);
    dim_font.setBold(true);
    painter.setFont(dim_font);
    painter.setPen(QColor(228, 235, 248));
    painter.drawText(QRectF(sheet_rect.left(), top_y - 26.0, sheet_rect.width(), 18.0),
                     Qt::AlignCenter,
                     QStringLiteral("Width · %1 cm").arg(sheet_width_cm_, 0, 'f', 1));
    painter.restore();

    const qreal left_x = sheet_rect.left() - 18.0;
    painter.save();
    painter.setPen(QPen(QColor(170, 190, 225, 120), 1.2));
    painter.drawLine(QPointF(left_x, sheet_rect.top()), QPointF(left_x, sheet_rect.bottom()));
    painter.drawLine(QPointF(left_x - 7.0, sheet_rect.top()), QPointF(left_x + 7.0, sheet_rect.top()));
    painter.drawLine(QPointF(left_x - 7.0, sheet_rect.bottom()), QPointF(left_x + 7.0, sheet_rect.bottom()));
    drawArrowHead(painter, QPointF(left_x, sheet_rect.top()), QPointF(0.0, 1.0), QColor(170, 190, 225, 180));
    drawArrowHead(painter, QPointF(left_x, sheet_rect.bottom()), QPointF(0.0, -1.0), QColor(170, 190, 225, 180));
    painter.translate(left_x - 24.0, sheet_rect.center().y());
    painter.rotate(-90.0);
    QFont height_font = painter.font();
    height_font.setPointSize(11);
    height_font.setBold(true);
    painter.setFont(height_font);
    painter.setPen(QColor(228, 235, 248));
    painter.drawText(QRectF(-sheet_rect.height() / 2.0, -10.0, sheet_rect.height(), 20.0),
                     Qt::AlignCenter,
                     QStringLiteral("Height · %1 cm").arg(sheet_height_cm_, 0, 'f', 1));
    painter.restore();

    const QRectF bottom_pill(sheet_rect.center().x() - 120.0, sheet_rect.bottom() + 14.0, 240.0, 26.0);
    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(10, 18, 40, 210));
    painter.drawRoundedRect(bottom_pill, 13.0, 13.0);
    painter.setPen(accent);
    QFont pill_font = painter.font();
    pill_font.setPointSize(10);
    pill_font.setBold(true);
    painter.setFont(pill_font);
    painter.drawText(bottom_pill,
                     Qt::AlignCenter,
                     QStringLiteral("%1 × %2 cm  ·  fit by %3")
                     .arg(sheet_width_cm_, 0, 'f', 1)
                     .arg(sheet_height_cm_, 0, 'f', 1)
                     .arg(fit_by_width ? QStringLiteral("width") : QStringLiteral("height")));
    painter.restore();
}

DrawingPage::DrawingPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
    wireSignals_();
    updateStatusChips_();
    updateSheetSizeFromUi_();
}

void DrawingPage::buildUi_() {
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(16);

    auto* workspace = buildWorkspaceColumn_();
    auto* controls = buildControlsColumn_();

    workspace->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    controls->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    controls->setMinimumWidth(520);
    controls->setMaximumWidth(520);

    root->addWidget(workspace, 1);
    root->addWidget(controls, 0);
}

QWidget* DrawingPage::buildWorkspaceColumn_() {
    auto* left = new QWidget(this);
    auto* layout = new QVBoxLayout(left);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    auto* workspace_frame = new QFrame(left);
    workspace_frame->setStyleSheet(
        "QFrame {"
        "border: 1px solid rgba(255,255,255,0.10);"
        "border-radius: 16px;"
        "background: rgba(255,255,255,0.025);"
        "}");

    auto* workspace_layout = new QVBoxLayout(workspace_frame);
    workspace_layout->setContentsMargins(14, 14, 14, 14);
    workspace_layout->setSpacing(12);

    auto* chip_row = new QHBoxLayout();
    chip_row->setSpacing(8);
    workspace_chip_ = makeChip("WORKSPACE READY", "#1E8E5A", workspace_frame);
    planning_chip_ = makeChip("PLANNING · IDLE", "#7B61FF", workspace_frame);
    execution_chip_ = makeChip("EXECUTION · IDLE", "#2962FF", workspace_frame);
    chip_row->addWidget(workspace_chip_, 0);
    chip_row->addWidget(planning_chip_, 0);
    chip_row->addWidget(execution_chip_, 0);
    chip_row->addStretch(1);

    sheet_preview_ = new SheetPreviewWidget(workspace_frame);

    workspace_layout->addLayout(chip_row);
    workspace_layout->addWidget(sheet_preview_, 1);

    layout->addWidget(workspace_frame, 1);
    layout->addWidget(buildProgressSection_(), 0);
    return left;
}

QWidget* DrawingPage::buildControlsColumn_() {
    auto* right = new QWidget(this);
    auto* layout = new QVBoxLayout(right);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    side_tabs_ = new QTabWidget(right);
    side_tabs_->setDocumentMode(true);
    side_tabs_->setStyleSheet(
        "QTabWidget::pane { border: 0px; background: transparent; }"
        "QTabBar::tab {"
        "  min-width: 120px;"
        "  min-height: 34px;"
        "  padding: 7px 14px;"
        "  margin-right: 8px;"
        "  border-radius: 10px;"
        "  background: rgba(10,18,40,0.70);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  font-weight: 700;"
        "}"
        "QTabBar::tab:selected {"
        "  border: 1px solid rgba(0,255,255,0.65);"
        "  background: rgba(18,30,60,0.85);"
        "}");

    side_tabs_->addTab(buildPlanTab_(), "Plan");
    side_tabs_->addTab(buildWaypointsTab_(), "Waypoints");

    layout->addWidget(side_tabs_, 1);
    return right;
}

QWidget* DrawingPage::buildPlanTab_() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 10, 0, 0);
    layout->setSpacing(18);

    auto* tools_section = buildToolsSection_(page);
    auto* planning_section = buildPlanningSection_(page);
    auto* actions_section = buildPlanActionsSection_(page);

    tools_section->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    planning_section->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    actions_section->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

    layout->addWidget(tools_section, 5);
    layout->addWidget(planning_section, 4);
    layout->addWidget(actions_section, 0);

    return page;
}

QWidget* DrawingPage::buildToolsSection_(QWidget* parent) {
    auto* tools_content = new QWidget(parent);
    auto* tools_grid = new QGridLayout(tools_content);
    tools_grid->setContentsMargins(0, 4, 0, 4);
    tools_grid->setHorizontalSpacing(12);
    tools_grid->setVerticalSpacing(14);

    freehand_btn_ = new QPushButton("Freehand", parent);
    line_btn_ = new QPushButton("Line", parent);
    spline_btn_ = new QPushButton("Spline", parent);
    circle_btn_ = new QPushButton("Circle", parent);
    erase_btn_ = new QPushButton("Erase", parent);
    import_svg_btn_ = new QPushButton("Import SVG", parent);
    clear_btn_ = new QPushButton("Clear", parent);

    tool_group_ = new QButtonGroup(this);
    tool_group_->setExclusive(true);
    for (auto* button : {
                freehand_btn_, line_btn_, spline_btn_, circle_btn_, erase_btn_
            }) {
        button->setCheckable(true);
        button->setStyleSheet(toolButtonStyle());
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        tool_group_->addButton(button);
    }
    freehand_btn_->setChecked(true);

    import_svg_btn_->setStyleSheet(actionButtonStyle("rgba(80, 200, 255, 0.85)", "rgba(80, 200, 255, 0.18)"));
    clear_btn_->setStyleSheet(actionButtonStyle("rgba(255, 80, 120, 0.85)", "rgba(255, 80, 120, 0.18)"));
    import_svg_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    clear_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    tools_grid->addWidget(freehand_btn_, 0, 0);
    tools_grid->addWidget(line_btn_, 0, 1);
    tools_grid->addWidget(spline_btn_, 1, 0);
    tools_grid->addWidget(circle_btn_, 1, 1);
    tools_grid->addWidget(erase_btn_, 2, 0, 1, 2);
    tools_grid->addWidget(import_svg_btn_, 3, 0);
    tools_grid->addWidget(clear_btn_, 3, 1);

    tools_grid->setRowStretch(0, 1);
    tools_grid->setRowStretch(1, 1);
    tools_grid->setRowStretch(2, 1);
    tools_grid->setRowStretch(3, 1);

    return makeSectionBox("Path tools", tools_content, parent);
}

QWidget* DrawingPage::buildPlanningSection_(QWidget* parent) {
    auto* plan_content = new QWidget(parent);
    auto* plan_outer = new QVBoxLayout(plan_content);
    plan_outer->setContentsMargins(0, 2, 0, 2);
    plan_outer->setSpacing(14);

    auto* plan_grid = new QGridLayout();
    plan_grid->setHorizontalSpacing(16);
    plan_grid->setVerticalSpacing(14);
    plan_grid->setColumnStretch(1, 1);
    plan_grid->setColumnStretch(3, 1);

    plane_selector_ = new QComboBox(parent);
    plane_selector_->addItems({"XY", "XZ", "YZ", "Tool plane"});
    scale_spin_ = new QDoubleSpinBox(parent);
    scale_spin_->setRange(0.01, 1000.0);
    scale_spin_->setValue(1.0);
    scale_spin_->setSuffix(" x");

    sample_step_spin_ = new QDoubleSpinBox(parent);
    sample_step_spin_->setRange(0.1, 100.0);
    sample_step_spin_->setValue(2.0);
    sample_step_spin_->setSuffix(" mm");
    speed_spin_ = new QDoubleSpinBox(parent);
    speed_spin_->setRange(0.1, 500.0);
    speed_spin_->setValue(40.0);
    speed_spin_->setSuffix(" mm/s");

    sheet_width_spin_ = new QDoubleSpinBox(parent);
    sheet_width_spin_->setRange(1.0, 500.0);
    sheet_width_spin_->setValue(21.0);
    sheet_width_spin_->setDecimals(1);
    sheet_width_spin_->setSuffix(" cm");

    sheet_height_spin_ = new QDoubleSpinBox(parent);
    sheet_height_spin_->setRange(1.0, 500.0);
    sheet_height_spin_->setValue(29.7);
    sheet_height_spin_->setDecimals(1);
    sheet_height_spin_->setSuffix(" cm");

    z_offset_spin_ = new QDoubleSpinBox(parent);
    z_offset_spin_->setRange(-500.0, 500.0);
    z_offset_spin_->setValue(0.0);
    z_offset_spin_->setSuffix(" mm");
    orientation_selector_ = new QComboBox(parent);
    orientation_selector_->addItems({"Keep tool normal", "Fixed tool frame", "Tangent to path"});

    close_path_check_ = new QCheckBox("Close path", parent);
    snap_grid_check_ = new QCheckBox("Snap to grid", parent);
    snap_grid_check_->setChecked(true);

    for (auto* field : {
                static_cast<QWidget*>(plane_selector_), static_cast<QWidget*>(scale_spin_),
                static_cast<QWidget*>(sample_step_spin_), static_cast<QWidget*>(speed_spin_),
                static_cast<QWidget*>(sheet_width_spin_), static_cast<QWidget*>(sheet_height_spin_),
                static_cast<QWidget*>(z_offset_spin_), static_cast<QWidget*>(orientation_selector_)
            }) {
        styleField(field);
    }

    int row = 0;
    plan_grid->addWidget(makeFieldLabel("Plane", parent), row, 0);
    plan_grid->addWidget(plane_selector_, row, 1);
    plan_grid->addWidget(makeFieldLabel("Scale", parent), row, 2);
    plan_grid->addWidget(scale_spin_, row, 3);
    ++row;

    plan_grid->addWidget(makeFieldLabel("Sample", parent), row, 0);
    plan_grid->addWidget(sample_step_spin_, row, 1);
    plan_grid->addWidget(makeFieldLabel("Speed", parent), row, 2);
    plan_grid->addWidget(speed_spin_, row, 3);
    ++row;

    plan_grid->addWidget(makeFieldLabel("Sheet W", parent), row, 0);
    plan_grid->addWidget(sheet_width_spin_, row, 1);
    plan_grid->addWidget(makeFieldLabel("Sheet H", parent), row, 2);
    plan_grid->addWidget(sheet_height_spin_, row, 3);
    ++row;

    plan_grid->addWidget(makeFieldLabel("Z offset", parent), row, 0);
    plan_grid->addWidget(z_offset_spin_, row, 1);
    plan_grid->addWidget(makeFieldLabel("Orientation", parent), row, 2);
    plan_grid->addWidget(orientation_selector_, row, 3);

    auto* checks_row = new QHBoxLayout();
    checks_row->setSpacing(20);
    checks_row->addWidget(close_path_check_);
    checks_row->addWidget(snap_grid_check_);
    checks_row->addStretch(1);

    plan_outer->addLayout(plan_grid, 1);
    plan_outer->addLayout(checks_row, 0);
    plan_outer->addStretch(1);

    return makeSectionBox("Planning", plan_content, parent);
}

QWidget* DrawingPage::buildPlanActionsSection_(QWidget* parent) {
    auto* action_content = new QWidget(parent);
    auto* action_layout = new QGridLayout(action_content);
    action_layout->setContentsMargins(0, 0, 0, 0);
    action_layout->setHorizontalSpacing(12);
    action_layout->setVerticalSpacing(12);

    preview_btn_ = new QPushButton("Plan", parent);
    generate_btn_ = new QPushButton("Execute", parent);
    stop_operation_btn_ = new QPushButton("Stop", parent);
    finish_operation_btn_ = new QPushButton("Finish Operation", parent);

    preview_btn_->setStyleSheet(actionButtonStyle("rgba(80, 200, 255, 0.85)", "rgba(80, 200, 255, 0.18)"));
    generate_btn_->setStyleSheet(actionButtonStyle("rgba(30, 142, 90, 0.90)", "rgba(30, 142, 90, 0.20)"));
    stop_operation_btn_->setStyleSheet(actionButtonStyle("rgba(255, 80, 120, 0.92)", "rgba(255, 80, 120, 0.20)"));
    finish_operation_btn_->setStyleSheet(actionButtonStyle("rgba(255, 209, 102, 0.90)", "rgba(255, 209, 102, 0.20)"));

    action_layout->addWidget(preview_btn_, 0, 0);
    action_layout->addWidget(generate_btn_, 0, 1);
    action_layout->addWidget(stop_operation_btn_, 1, 0);
    action_layout->addWidget(finish_operation_btn_, 1, 1);
    action_layout->setColumnStretch(0, 1);
    action_layout->setColumnStretch(1, 1);

    return action_content;
}

QWidget* DrawingPage::buildWaypointsTab_() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 10, 0, 0);
    layout->setSpacing(14);

    auto* content = new QWidget(page);
    auto* v = new QVBoxLayout(content);
    v->setContentsMargins(0, 2, 0, 2);
    v->setSpacing(12);

    waypoint_summary_ = new QLabel("0 points · ready for preview", page);
    waypoint_summary_->setStyleSheet("color: rgba(255,255,255,0.72); font-weight: 700;");

    waypoint_table_ = new QTableWidget(6, 5, page);
    waypoint_table_->setHorizontalHeaderLabels({"#", "X", "Y", "Z", "Yaw"});
    waypoint_table_->verticalHeader()->setVisible(false);
    waypoint_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    waypoint_table_->horizontalHeader()->setMinimumSectionSize(44);
    waypoint_table_->setMinimumHeight(320);
    waypoint_table_->setStyleSheet(
        "QTableWidget {"
        "background: rgba(5,12,30,0.72);"
        "border: 1px solid rgba(255,255,255,0.08);"
        "border-radius: 10px;"
        "gridline-color: rgba(255,255,255,0.08);"
        "}");

    for (int row = 0; row < waypoint_table_->rowCount(); ++row) {
        waypoint_table_->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
        waypoint_table_->setItem(row, 1, new QTableWidgetItem("--"));
        waypoint_table_->setItem(row, 2, new QTableWidgetItem("--"));
        waypoint_table_->setItem(row, 3, new QTableWidgetItem("--"));
        waypoint_table_->setItem(row, 4, new QTableWidgetItem("--"));
    }

    export_btn_ = new QPushButton("Export waypoints", page);
    export_btn_->setStyleSheet(actionButtonStyle("rgba(255, 209, 102, 0.90)", "rgba(255, 209, 102, 0.20)"));

    v->addWidget(waypoint_summary_);
    v->addWidget(waypoint_table_, 1);
    v->addWidget(export_btn_);

    layout->addWidget(makeSectionBox("Waypoint preview", content, page), 1);
    return page;
}

QWidget* DrawingPage::buildProgressSection_() {
    auto* content = new QWidget(this);
    auto* layout = new QGridLayout(content);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setHorizontalSpacing(16);
    layout->setVerticalSpacing(12);

    planning_progress_ = new QProgressBar(content);
    execution_progress_ = new QProgressBar(content);
    planning_progress_->setRange(0, 100);
    execution_progress_->setRange(0, 100);
    planning_progress_->setValue(0);
    execution_progress_->setValue(0);

    styleProgressBar(planning_progress_, "qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #7B61FF, stop:1 #63A8FF)");
    styleProgressBar(execution_progress_, "qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #1E8E5A, stop:1 #61D095)");

    planning_percent_label_ = new QLabel("0%", content);
    execution_percent_label_ = new QLabel("0%", content);
    planning_percent_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    execution_percent_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    planning_percent_label_->setStyleSheet("font-weight: 700;");
    execution_percent_label_->setStyleSheet("font-weight: 700;");

    auto* planning_title = new QLabel("Planning completion", content);
    auto* execution_title = new QLabel("Execution completion", content);
    planning_title->setStyleSheet("font-weight: 700;");
    execution_title->setStyleSheet("font-weight: 700;");

    layout->addWidget(planning_title, 0, 0);
    layout->addWidget(planning_percent_label_, 0, 1);
    layout->addWidget(planning_progress_, 1, 0, 1, 2);

    layout->addWidget(execution_title, 0, 2);
    layout->addWidget(execution_percent_label_, 0, 3);
    layout->addWidget(execution_progress_, 1, 2, 1, 2);

    layout->setColumnStretch(0, 1);
    layout->setColumnStretch(2, 1);
    return makeSectionBox("Progress", content, this);
}

void DrawingPage::wireSignals_() {
    connect(import_svg_btn_, &QPushButton::clicked, this, &DrawingPage::importSvgRequested);
    connect(clear_btn_, &QPushButton::clicked, this, &DrawingPage::clearCanvasRequested);
    connect(preview_btn_, &QPushButton::clicked, this, [this]() {
        emit previewTrajectoryRequested();
        emit generateTrajectoryRequested();
    });
    connect(generate_btn_, &QPushButton::clicked, this, &DrawingPage::sendToPlannerRequested);
    connect(stop_operation_btn_, &QPushButton::clicked, this, &DrawingPage::stopOperationRequested);
    connect(finish_operation_btn_, &QPushButton::clicked, this, &DrawingPage::finishOperationRequested);
    connect(export_btn_, &QPushButton::clicked, this, &DrawingPage::exportWaypointsRequested);

    connect(sheet_width_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
        updateSheetSizeFromUi_();
    });
    connect(sheet_height_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
        updateSheetSizeFromUi_();
    });
}

void DrawingPage::updateStatusChips_() {
    if (workspace_chip_) {
        workspace_chip_->setText(workspace_ready_ ? "WORKSPACE READY" : "WORKSPACE WAITING");
        workspace_chip_->setStyleSheet(chipStyle(workspace_ready_ ? "#1E8E5A" : "#7A7A7A"));
    }
    if (planning_chip_) {
        planning_chip_->setText(QString("PLANNING · %1").arg(planning_status_text_.toUpper()));
        planning_chip_->setStyleSheet(chipStyle("#7B61FF"));
    }
    if (execution_chip_) {
        execution_chip_->setText(QString("EXECUTION · %1").arg(execution_status_text_.toUpper()));
        execution_chip_->setStyleSheet(chipStyle("#2962FF"));
    }
}

void DrawingPage::updateSheetSizeFromUi_() {
    if (!sheet_preview_ || !sheet_width_spin_ || !sheet_height_spin_) return;
    sheet_preview_->setSheetSizeCm(sheet_width_spin_->value(), sheet_height_spin_->value());
}

void DrawingPage::setWorkspaceReady(bool ready) {
    workspace_ready_ = ready;
    updateStatusChips_();
}

void DrawingPage::setPlanningStatus(const QString& text) {
    planning_status_text_ = text.trimmed().isEmpty() ? QStringLiteral("Idle") : text.trimmed();
    updateStatusChips_();
}

void DrawingPage::setExecutionStatus(const QString& text) {
    execution_status_text_ = text.trimmed().isEmpty() ? QStringLiteral("Idle") : text.trimmed();
    updateStatusChips_();
}

void DrawingPage::setPlanningProgress(int percent) {
    const int clamped = std::clamp(percent, 0, 100);
    if (planning_progress_) planning_progress_->setValue(clamped);
    if (planning_percent_label_) planning_percent_label_->setText(QString("%1%").arg(clamped));
}

void DrawingPage::setExecutionProgress(int percent) {
    const int clamped = std::clamp(percent, 0, 100);
    if (execution_progress_) execution_progress_->setValue(clamped);
    if (execution_percent_label_) execution_percent_label_->setText(QString("%1%").arg(clamped));
}

void DrawingPage::setCanvasPlaceholderText(const QString& text) {
    if (sheet_preview_) {
        sheet_preview_->setOverlayText(text.trimmed().isEmpty()
                                       ? QStringLiteral("Attach top-view robot overlay and drawing preview here")
                                       : text);
    }
}

void DrawingPage::setSheetSizeCm(double width_cm, double height_cm) {
    if (!sheet_width_spin_ || !sheet_height_spin_) return;

    const QSignalBlocker block_w(*sheet_width_spin_);
    const QSignalBlocker block_h(*sheet_height_spin_);
    sheet_width_spin_->setValue(std::clamp(width_cm, 1.0, 500.0));
    sheet_height_spin_->setValue(std::clamp(height_cm, 1.0, 500.0));
    updateSheetSizeFromUi_();
}

}  // namespace davinci_arm::ui::pages
