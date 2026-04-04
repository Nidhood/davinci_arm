#include "davinci_arm_gui/ui/pages/calibration_page.hpp"

#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/angle_ref_plot.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"
#include "davinci_arm_gui/ui/widgets/error_plot.hpp"

#include <QAbstractSpinBox>
#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>

namespace davinci_arm::ui::pages {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr std::size_t kJointCount = 5;

struct JointSpec final {
    const char* label;
};

constexpr std::array<JointSpec, kJointCount> kJointSpecs{{
        {"Shoulder"},
        {"Bicep"},
        {"Arm"},
        {"Wrist"},
        {"End Effector"},
    }};

QFrame* makePanel(QWidget* parent) {
    auto* panel = new QFrame(parent);
    panel->setObjectName("panel");
    panel->setFrameShape(QFrame::NoFrame);
    panel->setAttribute(Qt::WA_StyledBackground, true);
    panel->setStyleSheet(
        "QFrame#panel {"
        "background: rgba(255,255,255,0.018);"
        "border: 1px solid rgba(255,255,255,0.08);"
        "border-radius: 14px;"
        "}");
    return panel;
}

void setChartTitle(QWidget* host, const QString& title) {
    if (!host) return;
    auto* chart = host->findChild<davinci_arm::ui::widgets::ChartBase*>();
    if (chart) chart->setTitle(title);
}

QPushButton* makeFocusButton(const QString& text, QWidget* parent) {
    auto* btn = new QPushButton(text, parent);
    btn->setCheckable(true);
    btn->setMinimumHeight(38);
    btn->setStyleSheet(
        "QPushButton {"
        "padding: 8px 12px;"
        "font-weight: 700;"
        "border-radius: 10px;"
        "border: 1px solid rgba(99,168,255,0.55);"
        "background: rgba(255,255,255,0.02);"
        "}"
        "QPushButton:hover { background: rgba(99,168,255,0.10); }"
        "QPushButton:checked {"
        "background: rgba(123,97,255,0.24);"
        "border: 1px solid rgba(123,97,255,0.95);"
        "}");
    return btn;
}

QPushButton* makeActionButton(const QString& text, QWidget* parent) {
    auto* btn = new QPushButton(text, parent);
    btn->setMinimumHeight(38);
    return btn;
}

}  // namespace

CalibrationPage::CalibrationPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
    connectSignals_();
    applyAngleRanges_();
    updateChartTitles_();
    updateStatusUi_(QStringLiteral("Idle"), 0);
}

void CalibrationPage::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    if (angle_ref_plot_) angle_ref_plot_->setLimitsRegistry(limits_);
    if (error_plot_) error_plot_->setLimitsRegistry(limits_);
    applyAngleRanges_();
}

void CalibrationPage::setCalibrationService(davinci_arm::services::CalibrationService* service) noexcept {
    calibration_service_ = service;
}

void CalibrationPage::buildUi_() {
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(16);

    root->addWidget(buildLeftColumn_(), 5);
    root->addWidget(buildRightColumn_(), 7);
}

QWidget* CalibrationPage::buildLeftColumn_() {
    auto* left = new QWidget(this);
    auto* layout = new QVBoxLayout(left);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    layout->addWidget(buildFocusPanel_(), 0);
    layout->addWidget(buildConfigPanel_(), 0);
    layout->addWidget(buildActionPanel_(), 0);
    layout->addWidget(buildParameterPanel_(), 1);
    layout->addWidget(buildLogPanel_(), 1);

    return left;
}

QWidget* CalibrationPage::buildRightColumn_() {
    auto* right = new QWidget(this);
    auto* layout = new QVBoxLayout(right);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    angle_ref_plot_ = new davinci_arm::ui::widgets::AngleRefPlot(right);
    angle_ref_plot_->setMinimumHeight(360);

    error_plot_ = new davinci_arm::ui::widgets::ErrorPlot(right);
    error_plot_->setMinimumHeight(300);

    layout->addWidget(angle_ref_plot_, 1);
    layout->addWidget(error_plot_, 1);

    return right;
}

QWidget* CalibrationPage::buildFocusPanel_() {
    auto* panel = makePanel(this);
    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setHorizontalSpacing(8);
    layout->setVerticalSpacing(8);

    focus_group_ = new QButtonGroup(this);
    focus_group_->setExclusive(true);

    for (std::size_t i = 0; i < kJointSpecs.size(); ++i) {
        auto* btn = makeFocusButton(QString::fromUtf8(kJointSpecs[i].label), panel);
        focus_buttons_[i] = btn;
        focus_group_->addButton(btn, static_cast<int>(i));
        layout->addWidget(btn, static_cast<int>(i / 3), static_cast<int>(i % 3));
    }

    if (focus_buttons_[0]) {
        focus_buttons_[0]->setChecked(true);
    }

    return panel;
}

QWidget* CalibrationPage::buildConfigPanel_() {
    auto* panel = makePanel(this);
    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(10);

    calibration_type_ = new QComboBox(panel);
    calibration_type_->addItems({"Static offset", "Friction sweep", "Dynamic model", "Closed-loop tuning"});

    excitation_profile_ = new QComboBox(panel);
    excitation_profile_->addItems({"Step", "Chirp", "PRBS", "Ramp"});

    amplitude_spin_ = new QDoubleSpinBox(panel);
    amplitude_spin_->setRange(0.1, 360.0);
    amplitude_spin_->setValue(15.0);
    amplitude_spin_->setSuffix(" deg");
    amplitude_spin_->setDecimals(1);
    amplitude_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);

    duration_spin_ = new QDoubleSpinBox(panel);
    duration_spin_->setRange(0.1, 600.0);
    duration_spin_->setValue(20.0);
    duration_spin_->setSuffix(" s");
    duration_spin_->setDecimals(1);
    duration_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);

    repetitions_spin_ = new QDoubleSpinBox(panel);
    repetitions_spin_->setRange(1.0, 100.0);
    repetitions_spin_->setValue(3.0);
    repetitions_spin_->setDecimals(0);
    repetitions_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);

    layout->addWidget(calibration_type_, 0, 0, 1, 2);
    layout->addWidget(excitation_profile_, 0, 2, 1, 2);
    layout->addWidget(amplitude_spin_, 1, 0, 1, 2);
    layout->addWidget(duration_spin_, 1, 2);
    layout->addWidget(repetitions_spin_, 1, 3);
    layout->setColumnStretch(0, 1);
    layout->setColumnStretch(1, 1);
    layout->setColumnStretch(2, 1);
    layout->setColumnStretch(3, 1);

    return panel;
}

QWidget* CalibrationPage::buildActionPanel_() {
    auto* panel = makePanel(this);
    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(10);

    status_value_ = new QLabel(QStringLiteral("Idle"), panel);
    status_value_->setStyleSheet("font-weight: 700;");

    progress_bar_ = new QProgressBar(panel);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setTextVisible(true);
    progress_bar_->setFormat("%p%");

    start_btn_ = makeActionButton(QStringLiteral("START"), panel);
    stop_btn_ = makeActionButton(QStringLiteral("STOP"), panel);
    apply_btn_ = makeActionButton(QStringLiteral("APPLY"), panel);
    reset_btn_ = makeActionButton(QStringLiteral("RESET"), panel);

    layout->addWidget(status_value_, 0, 0);
    layout->addWidget(progress_bar_, 0, 1, 1, 3);
    layout->addWidget(start_btn_, 1, 0);
    layout->addWidget(stop_btn_, 1, 1);
    layout->addWidget(apply_btn_, 1, 2);
    layout->addWidget(reset_btn_, 1, 3);
    layout->setColumnStretch(0, 1);
    layout->setColumnStretch(1, 1);
    layout->setColumnStretch(2, 1);
    layout->setColumnStretch(3, 1);

    return panel;
}

QWidget* CalibrationPage::buildParameterPanel_() {
    auto* panel = makePanel(this);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(10);

    identified_params_table_ = new QTableWidget(6, 2, panel);
    identified_params_table_->setHorizontalHeaderLabels({"Parameter", "Value"});
    identified_params_table_->verticalHeader()->setVisible(false);
    identified_params_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    identified_params_table_->setItem(0, 0, new QTableWidgetItem("Zero offset"));
    identified_params_table_->setItem(1, 0, new QTableWidgetItem("Viscous damping"));
    identified_params_table_->setItem(2, 0, new QTableWidgetItem("Static friction"));
    identified_params_table_->setItem(3, 0, new QTableWidgetItem("Gear ratio"));
    identified_params_table_->setItem(4, 0, new QTableWidgetItem("Kp / controller"));
    identified_params_table_->setItem(5, 0, new QTableWidgetItem("Kd / controller"));

    for (int row = 0; row < identified_params_table_->rowCount(); ++row) {
        identified_params_table_->setItem(row, 1, new QTableWidgetItem("--"));
    }

    layout->addWidget(identified_params_table_, 1);
    return panel;
}

QWidget* CalibrationPage::buildLogPanel_() {
    auto* panel = makePanel(this);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(10);

    notes_log_ = new QPlainTextEdit(panel);
    notes_log_->setPlaceholderText(
        "Calibration notes, backend messages, detected issues, rejected samples or operator remarks.");
    notes_log_->setMinimumHeight(160);

    layout->addWidget(notes_log_, 1);
    return panel;
}

void CalibrationPage::connectSignals_() {
    if (focus_group_) {
        connect(focus_group_, &QButtonGroup::idClicked, this, &CalibrationPage::onFocusButtonClicked_);
    }

    connect(start_btn_, &QPushButton::clicked, this, &CalibrationPage::onStartClicked_);
    connect(stop_btn_, &QPushButton::clicked, this, &CalibrationPage::onStopClicked_);
    connect(apply_btn_, &QPushButton::clicked, this, &CalibrationPage::onApplyClicked_);
    connect(reset_btn_, &QPushButton::clicked, this, &CalibrationPage::onResetClicked_);
}

void CalibrationPage::applyAngleRanges_() {
    if (!limits_) return;

    const auto& angle_limits = limits_->angleLimits();
    if (amplitude_spin_) {
        amplitude_spin_->setRange(0.1, std::max(0.1, angle_limits.max - angle_limits.min));
    }
}

void CalibrationPage::updateChartTitles_() {
    const QString joint = labelForJoint_(activeJointIndex_());
    setChartTitle(angle_ref_plot_, QStringLiteral("Calibration response · %1").arg(joint));
    setChartTitle(error_plot_, QStringLiteral("Calibration error · %1").arg(joint));
}

void CalibrationPage::updateStatusUi_(const QString& status, int progress_pct) {
    if (status_value_) status_value_->setText(status);
    if (progress_bar_) progress_bar_->setValue(std::clamp(progress_pct, 0, 100));
}

void CalibrationPage::updateParameterEstimates_(double real_deg, double ref_deg) {
    if (!identified_params_table_) return;

    const double error_deg = ref_deg - real_deg;
    identified_params_table_->item(0, 1)->setText(QString::number(error_deg, 'f', 2));
    identified_params_table_->item(1, 1)->setText(QString::number(std::fabs(error_deg) * 0.015, 'f', 4));
    identified_params_table_->item(2, 1)->setText(QString::number(std::fabs(error_deg) * 0.010, 'f', 4));
    identified_params_table_->item(3, 1)->setText(QStringLiteral("1.0000"));
    identified_params_table_->item(4, 1)->setText(QString::number(std::fabs(error_deg) * 0.10, 'f', 3));
    identified_params_table_->item(5, 1)->setText(QString::number(std::fabs(error_deg) * 0.02, 'f', 3));
}

int CalibrationPage::activeJointIndex_() const noexcept {
    return active_joint_index_;
}

QString CalibrationPage::labelForJoint_(int joint_index) const {
    const int idx = std::clamp(joint_index, 0, static_cast<int>(kJointSpecs.size()) - 1);
    return QString::fromUtf8(kJointSpecs[static_cast<std::size_t>(idx)].label);
}

davinci_arm::models::CalibrationConfig CalibrationPage::buildConfig_() const {
    davinci_arm::models::CalibrationConfig cfg{};
    if (duration_spin_) cfg.duration_sec = duration_spin_->value();
    return cfg;
}

void CalibrationPage::onFocusButtonClicked_(int index) {
    active_joint_index_ = std::clamp(index, 0, static_cast<int>(kJointCount) - 1);
    updateChartTitles_();
}

void CalibrationPage::onStartClicked_() {
    updateStatusUi_(QStringLiteral("Running"), 5);
    if (notes_log_) notes_log_->appendPlainText(QStringLiteral("Calibration started."));
    emit startCalibrationRequested(buildConfig_());
}

void CalibrationPage::onStopClicked_() {
    updateStatusUi_(QStringLiteral("Stopped"), progress_bar_ ? progress_bar_->value() : 0);
    if (notes_log_) notes_log_->appendPlainText(QStringLiteral("Calibration stopped."));
    emit stopCalibrationRequested();
}

void CalibrationPage::onApplyClicked_() {
    if (notes_log_) notes_log_->appendPlainText(QStringLiteral("Apply identified parameters requested."));
    emit applyParametersRequested();
}

void CalibrationPage::onResetClicked_() {
    updateStatusUi_(QStringLiteral("Idle"), 0);

    if (notes_log_) notes_log_->appendPlainText(QStringLiteral("Calibration reset."));
    if (angle_ref_plot_) angle_ref_plot_->clear();
    if (error_plot_) error_plot_->clear();

    if (identified_params_table_) {
        for (int row = 0; row < identified_params_table_->rowCount(); ++row) {
            if (identified_params_table_->item(row, 1)) {
                identified_params_table_->item(row, 1)->setText(QStringLiteral("--"));
            }
        }
    }

    emit resetCalibrationRequested();
}

void CalibrationPage::setStreamLive(davinci_arm::models::Domain domain, bool live) {
    if (domain == davinci_arm::models::Domain::Real) {
        real_live_ = live;
    } else if (domain == davinci_arm::models::Domain::Sim) {
        sim_live_ = live;
    }

    if (angle_ref_plot_) angle_ref_plot_->setStreamLive(domain, live);
    if (error_plot_) error_plot_->setStreamLive(domain, live);
}

void CalibrationPage::onTelemetry(const davinci_arm::models::TelemetrySample& sample) {
    if (!sample.valid) return;

    if (angle_ref_plot_) angle_ref_plot_->pushSample(sample);
    if (error_plot_) error_plot_->pushSample(sample);

    const double angle_deg = sample.arm_angle_rad * kRadToDeg;
    const double ref_deg = sample.ref_angle_rad * kRadToDeg;

    updateParameterEstimates_(angle_deg, ref_deg);

    const QString status = (sample.domain == davinci_arm::models::Domain::Real)
                           ? QStringLiteral("Real feedback")
                           : (sample.domain == davinci_arm::models::Domain::Sim)
                           ? QStringLiteral("Sim feedback")
                           : QStringLiteral("Reference only");

    const int next_progress = progress_bar_ ? std::min(progress_bar_->value() + 1, 100) : 0;
    updateStatusUi_(status, next_progress);
}

}  // namespace davinci_arm::ui::pages
