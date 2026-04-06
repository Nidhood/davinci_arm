#include "davinci_arm_gui/ui/pages/control_panel_page.hpp"

#include "davinci_arm_gui/core/models/csv_export_options.hpp"
#include "davinci_arm_gui/core/models/recording_state.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"
#include "davinci_arm_gui/core/services/recorder_service.hpp"
#include "davinci_arm_gui/infra/ros/limits_registry.hpp"
#include "davinci_arm_gui/ui/widgets/angle_ref_plot.hpp"
#include "davinci_arm_gui/ui/widgets/chart_base.hpp"
#include "davinci_arm_gui/ui/widgets/export_preview_dialog.hpp"
#include "davinci_arm_gui/ui/widgets/tracking_error_plot.hpp"

#include <QAbstractSpinBox>
#include <QButtonGroup>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <type_traits>

namespace davinci_arm::ui::pages {

namespace {

using davinci_arm::models::Domain;
using davinci_arm::models::TelemetrySignalType;

constexpr int kEmitThrottleMs = 40;
constexpr double kChartWindowSeconds = 30.0;
constexpr std::size_t kJointCount = 5;

struct JointSpec final {
    const char* label;
    const char* joint_name;
};

constexpr std::array<JointSpec, kJointCount> kJointSpecs{{
        {"Shoulder", "shoulder_link_joint"},
        {"Bicep", "bicep_link_joint"},
        {"Arm", "arm_link_joint"},
        {"Wrist", "wrist_link_joint"},
        {"End Effector", "end_effector_link_joint"},
    }};

QFrame* makePanel(QWidget* parent)
{
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

davinci_arm::ui::widgets::ChartBase* chartOf(QWidget* host)
{
    if (!host) {
        return nullptr;
    }
    return host->findChild<davinci_arm::ui::widgets::ChartBase*>();
}

void setChartTitle(QWidget* host, const QString& title)
{
    if (auto* chart = chartOf(host)) {
        chart->setTitle(title);
    }
}

template<typename T>
std::optional<std::string> jointNameOf(const T& sample)
{
    if constexpr (requires { sample.joint_name; }) {
        return sample.joint_name;
    } else if constexpr (requires { sample.label; }) {
        return sample.label;
    } else if constexpr (requires { sample.series; }) {
        return sample.series;
    } else if constexpr (requires { sample.channel; }) {
        return sample.channel;
    } else {
        return std::nullopt;
    }
}

template<typename T>
std::optional<TelemetrySignalType> signalTypeOf(const T& sample)
{
    if constexpr (requires { sample.signal; }) {
        return sample.signal;
    } else if constexpr (requires { sample.signal_type; }) {
        return sample.signal_type;
    } else if constexpr (requires { sample.telemetry_signal; }) {
        return sample.telemetry_signal;
    } else {
        return std::nullopt;
    }
}

template<typename T>
std::optional<double> timeSecOf(const T& sample)
{
    if constexpr (requires { sample.time_sec; }) {
        return static_cast<double>(sample.time_sec);
    } else if constexpr (requires { sample.timestamp_sec; }) {
        return static_cast<double>(sample.timestamp_sec);
    } else if constexpr (requires { sample.t_sec; }) {
        return static_cast<double>(sample.t_sec);
    } else if constexpr (requires { sample.time_s; }) {
        return static_cast<double>(sample.time_s);
    } else if constexpr (requires { sample.t; }) {
        using U = std::decay_t<decltype(sample.t)>;
        if constexpr (std::is_arithmetic_v<U>) {
            return static_cast<double>(sample.t);
        } else {
            return std::nullopt;
        }
    } else {
        return std::nullopt;
    }
}

}  // namespace

ControlPanelPage::ControlPanelPage(QWidget* parent)
    : QWidget(parent)
{
    buildUi_();
    connectSignals_();
    applyAngleRanges_();
    updateChartTitles_();
    updateRecordingUi_();

    auto* ui_refresh_timer = new QTimer(this);
    ui_refresh_timer->setInterval(250);
    connect(ui_refresh_timer, &QTimer::timeout, this, [this]() {
        updateRecordingUi_();
    });
    ui_refresh_timer->start();
}

void ControlPanelPage::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) noexcept
{
    limits_ = limits;
    if (angle_ref_plot_) {
        angle_ref_plot_->setLimitsRegistry(limits_);
    }
    if (error_plot_) {
        error_plot_->setLimitsRegistry(limits_);
    }
    applyAngleRanges_();
}

void ControlPanelPage::setRecorderService(davinci_arm::core::services::RecorderService* recorder)
{
    recorder_service_ = recorder;
    updateRecordingUi_();
}

void ControlPanelPage::buildUi_()
{
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(16);

    root->addWidget(buildLeftColumn_(), 5);
    root->addWidget(buildRightColumn_(), 7);
}

QWidget* ControlPanelPage::buildLeftColumn_()
{
    auto* left = new QWidget(this);
    auto* layout = new QVBoxLayout(left);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    layout->addWidget(buildReferencePanel_(), 1);
    layout->addWidget(buildRecordingPanel_(), 0);

    return left;
}

QWidget* ControlPanelPage::buildRightColumn_()
{
    auto* right = new QWidget(this);
    auto* layout = new QVBoxLayout(right);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);

    angle_ref_plot_ = new davinci_arm::ui::widgets::AngleRefPlot(right);
    angle_ref_plot_->setMinimumHeight(360);

    error_plot_ = new davinci_arm::ui::widgets::TrackingErrorPlot(right);
    error_plot_->setMinimumHeight(300);

    if (auto* chart = chartOf(angle_ref_plot_)) {
        chart->setWindowSeconds(kChartWindowSeconds);
    }
    if (auto* chart = chartOf(error_plot_)) {
        chart->setWindowSeconds(kChartWindowSeconds);
    }

    layout->addWidget(angle_ref_plot_, 1);
    layout->addWidget(error_plot_, 1);

    return right;
}

QFrame* ControlPanelPage::buildReferencePanel_()
{
    auto* panel = makePanel(this);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(14);

    auto* focus_row = new QGridLayout();
    focus_row->setHorizontalSpacing(8);
    focus_row->setVerticalSpacing(8);

    focus_group_ = new QButtonGroup(this);
    focus_group_->setExclusive(true);

    for (std::size_t i = 0; i < kJointSpecs.size(); ++i) {
        auto* btn = new QPushButton(QString::fromUtf8(kJointSpecs[i].label), panel);
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
        joint_widgets_[i].focus_button = btn;
        focus_group_->addButton(btn, static_cast<int>(i));
        focus_row->addWidget(btn, static_cast<int>(i / 3), static_cast<int>(i % 3));
    }

    if (joint_widgets_[0].focus_button) {
        joint_widgets_[0].focus_button->setChecked(true);
    }

    layout->addLayout(focus_row);

    for (std::size_t i = 0; i < kJointSpecs.size(); ++i) {
        auto* row = new QWidget(panel);
        auto* row_layout = new QGridLayout(row);
        row_layout->setContentsMargins(0, 0, 0, 0);
        row_layout->setHorizontalSpacing(10);
        row_layout->setVerticalSpacing(6);

        joint_widgets_[i].slider = new QSlider(Qt::Horizontal, row);
        joint_widgets_[i].slider->setRange(0, 360);
        joint_widgets_[i].slider->setValue(180);

        joint_widgets_[i].spin = new QDoubleSpinBox(row);
        joint_widgets_[i].spin->setRange(0.0, 360.0);
        joint_widgets_[i].spin->setDecimals(1);
        joint_widgets_[i].spin->setSuffix(" °");
        joint_widgets_[i].spin->setValue(180.0);
        joint_widgets_[i].spin->setMinimumWidth(108);
        joint_widgets_[i].spin->setButtonSymbols(QAbstractSpinBox::NoButtons);

        row_layout->addWidget(joint_widgets_[i].slider, 0, 0);
        row_layout->addWidget(joint_widgets_[i].spin, 0, 1);
        row_layout->setColumnStretch(0, 1);

        layout->addWidget(row);
    }

    zero_all_btn_ = new QPushButton("CENTER ALL", panel);
    zero_all_btn_->setMinimumHeight(38);
    layout->addWidget(zero_all_btn_);

    return panel;
}

QFrame* ControlPanelPage::buildRecordingPanel_()
{
    auto* panel = makePanel(this);
    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(10);

    recording_duration_ = new QDoubleSpinBox(panel);
    recording_duration_->setRange(1.0, 3600.0);
    recording_duration_->setValue(120.0);
    recording_duration_->setDecimals(1);
    recording_duration_->setSuffix(" s");
    recording_duration_->setButtonSymbols(QAbstractSpinBox::NoButtons);

    start_recording_btn_ = new QPushButton("START REC", panel);
    stop_recording_btn_ = new QPushButton("STOP REC", panel);
    export_recording_btn_ = new QPushButton("EXPORT CSV", panel);

    recording_progress_ = new QProgressBar(panel);
    recording_progress_->setRange(0, 100);
    recording_progress_->setValue(0);
    recording_progress_->setTextVisible(true);
    recording_progress_->setFormat("%p%");

    layout->addWidget(recording_duration_, 0, 0);
    layout->addWidget(start_recording_btn_, 0, 1);
    layout->addWidget(stop_recording_btn_, 0, 2);
    layout->addWidget(recording_progress_, 1, 0, 1, 3);
    layout->addWidget(export_recording_btn_, 2, 0, 1, 3);

    return panel;
}

void ControlPanelPage::connectSignals_()
{
    if (focus_group_) {
        connect(focus_group_, &QButtonGroup::idClicked, this, &ControlPanelPage::onFocusButtonClicked_);
    }

    for (std::size_t i = 0; i < joint_widgets_.size(); ++i) {
        auto* slider = joint_widgets_[i].slider;
        auto* spin = joint_widgets_[i].spin;
        if (!slider || !spin) {
            continue;
        }

        connect(slider, &QSlider::valueChanged, this, [this, i](int value) {
            const QSignalBlocker blocker(*joint_widgets_[i].spin);
            joint_widgets_[i].spin->setValue(static_cast<double>(value));
            publishJointReference_(static_cast<int>(i), static_cast<double>(value), false);
        });

        connect(slider, &QSlider::sliderReleased, this, [this, i]() {
            publishJointReference_(static_cast<int>(i), joint_widgets_[i].spin->value(), true);
        });

        connect(
            spin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
        [this, i](double value) {
            const QSignalBlocker blocker(*joint_widgets_[i].slider);
            joint_widgets_[i].slider->setValue(static_cast<int>(std::lround(value)));
            publishJointReference_(static_cast<int>(i), value, false);
        });
    }

    connect(zero_all_btn_, &QPushButton::clicked, this, &ControlPanelPage::onZeroAllClicked_);
    connect(start_recording_btn_, &QPushButton::clicked, this, &ControlPanelPage::onStartRecordingClicked_);
    connect(stop_recording_btn_, &QPushButton::clicked, this, &ControlPanelPage::onStopRecordingClicked_);
    connect(export_recording_btn_, &QPushButton::clicked, this, &ControlPanelPage::onExportRecordingClicked_);
}

void ControlPanelPage::applyAngleRanges_()
{
    double min_deg = 0.0;
    double max_deg = 360.0;

    if (limits_) {
        const auto& angle_limits = limits_->angleLimits();
        min_deg = angle_limits.min;
        max_deg = angle_limits.max;
    }

    for (auto& joint : joint_widgets_) {
        if (joint.slider) {
            joint.slider->setRange(
                static_cast<int>(std::floor(min_deg)),
                static_cast<int>(std::ceil(max_deg)));
        }
        if (joint.spin) {
            joint.spin->setRange(min_deg, max_deg);
        }
    }
}

void ControlPanelPage::updateRecordingUi_()
{
    if (!recording_progress_) {
        return;
    }

    if (!recorder_service_) {
        recording_progress_->setValue(0);
        if (export_recording_btn_) {
            export_recording_btn_->setEnabled(false);
        }
        return;
    }

    const auto stats = recorder_service_->stats();
    const int progress = (stats.duration_s > 0.0)
                         ? static_cast<int>(std::lround((stats.elapsed_s / stats.duration_s) * 100.0))
                         : 0;

    switch (stats.state) {
    case davinci_arm::models::RecordingState::Idle:
        recording_progress_->setValue(0);
        break;
    case davinci_arm::models::RecordingState::Recording:
    case davinci_arm::models::RecordingState::Stopped:
        recording_progress_->setValue(std::clamp(progress, 0, 100));
        break;
    case davinci_arm::models::RecordingState::Completed:
        recording_progress_->setValue(100);
        break;
    }

    if (export_recording_btn_) {
        export_recording_btn_->setEnabled(!recorder_service_->recorded().empty());
    }
}

void ControlPanelPage::updateChartTitles_()
{
    const QString focused = labelForJoint_(active_joint_index_);
    setChartTitle(angle_ref_plot_, focused + " | reference vs response");
    setChartTitle(error_plot_, focused + " | tracking error");
}

void ControlPanelPage::updateAdaptiveDensity_(Domain domain, double t_sec) noexcept
{
    sample_rate_estimator_.observe(domain, t_sec);
    const auto dt = sample_rate_estimator_.dtEma(domain);
    if (!dt.has_value()) {
        return;
    }

    const int max_points =
        davinci_arm::core::charts::SampleRateEstimator::recommendedMaxPoints(
            kChartWindowSeconds,
            *dt);

    if (auto* chart = chartOf(angle_ref_plot_)) {
        chart->setMaxPoints(max_points);
    }
    if (auto* chart = chartOf(error_plot_)) {
        chart->setMaxPoints(max_points);
    }
}

void ControlPanelPage::setActiveJointIndex_(int index)
{
    active_joint_index_ = std::clamp(index, 0, static_cast<int>(kJointCount) - 1);
    updateChartTitles_();

    sample_rate_estimator_.clear();
    if (angle_ref_plot_) {
        angle_ref_plot_->clear();
    }
    if (error_plot_) {
        error_plot_->clear();
    }
}

int ControlPanelPage::activeJointIndex_() const noexcept
{
    return active_joint_index_;
}

QVector<double> ControlPanelPage::collectJointRefsDeg_() const
{
    QVector<double> values;
    values.reserve(static_cast<int>(kJointCount));
    for (const auto& joint : joint_widgets_) {
        values.push_back(joint.spin ? joint.spin->value() : 180.0);
    }
    return values;
}

void ControlPanelPage::publishJointReference_(int jointIndex, double deg, bool force)
{
    if (jointIndex < 0 || jointIndex >= static_cast<int>(kJointCount)) {
        return;
    }

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (!force && (now_ms - last_emit_ms_[static_cast<std::size_t>(jointIndex)]) < kEmitThrottleMs) {
        return;
    }
    last_emit_ms_[static_cast<std::size_t>(jointIndex)] = now_ms;

    emit jointReferenceChanged(jointIndex, deg);
    emit jointBatchCommandRequested(collectJointRefsDeg_());
}

void ControlPanelPage::setJointUiDeg_(int jointIndex, double deg)
{
    if (jointIndex < 0 || jointIndex >= static_cast<int>(kJointCount)) {
        return;
    }

    auto& joint = joint_widgets_[static_cast<std::size_t>(jointIndex)];
    if (!joint.slider || !joint.spin) {
        return;
    }

    const double clamped = std::clamp(deg, joint.spin->minimum(), joint.spin->maximum());
    const QSignalBlocker spin_blocker(*joint.spin);
    const QSignalBlocker slider_blocker(*joint.slider);
    joint.spin->setValue(clamped);
    joint.slider->setValue(static_cast<int>(std::lround(clamped)));
}

void ControlPanelPage::applyAllZero_()
{
    for (int i = 0; i < static_cast<int>(kJointCount); ++i) {
        setJointUiDeg_(i, 180.0);
    }
    emit jointBatchCommandRequested(collectJointRefsDeg_());
}

QString ControlPanelPage::labelForJoint_(int jointIndex) const
{
    if (jointIndex < 0 || jointIndex >= static_cast<int>(kJointCount)) {
        return QStringLiteral("Joint");
    }
    return QString::fromUtf8(kJointSpecs[static_cast<std::size_t>(jointIndex)].label);
}

QString ControlPanelPage::activeJointName_() const
{
    if (active_joint_index_ < 0 || active_joint_index_ >= static_cast<int>(kJointCount)) {
        return {};
    }
    return QString::fromUtf8(kJointSpecs[static_cast<std::size_t>(active_joint_index_)].joint_name);
}

int ControlPanelPage::resolveJointIndex_(const davinci_arm::models::TelemetrySample& sample) const noexcept
{
    const auto sample_joint = jointNameOf(sample);
    if (!sample_joint.has_value()) {
        return -1;
    }

    for (std::size_t i = 0; i < kJointSpecs.size(); ++i) {
        if (*sample_joint == kJointSpecs[i].joint_name) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

void ControlPanelPage::onFocusButtonClicked_(int index)
{
    setActiveJointIndex_(index);
}

void ControlPanelPage::onZeroAllClicked_()
{
    applyAllZero_();
}

void ControlPanelPage::onStartRecordingClicked_()
{
    if (recorder_service_) {
        recorder_service_->start(recording_duration_ ? recording_duration_->value() : 0.0);
    }
    updateRecordingUi_();
    emit startRecordingRequested(recording_duration_ ? recording_duration_->value() : 0.0);
}

void ControlPanelPage::onStopRecordingClicked_()
{
    if (recorder_service_) {
        recorder_service_->stop();
    }
    updateRecordingUi_();
    emit stopRecordingRequested();
}

void ControlPanelPage::onExportRecordingClicked_()
{
    emit exportRecordingRequested();

    if (!recorder_service_) {
        QMessageBox::warning(this, "Export", "Recorder service is not available.");
        return;
    }

    const auto recorded = recorder_service_->recorded();
    if (recorded.empty()) {
        QMessageBox::information(this, "Export", "There is no recorded telemetry to export yet.");
        return;
    }

    QVector<davinci_arm::models::TelemetrySample> samples;
    samples.reserve(static_cast<int>(recorded.size()));

    for (const auto& sample : recorded) {
        const auto signal = signalTypeOf(sample);
        if (!sample.valid || !signal.has_value()) {
            continue;
        }

        if (*signal == TelemetrySignalType::Angle || *signal == TelemetrySignalType::AngleRef) {
            samples.push_back(sample);
        }
    }

    if (samples.isEmpty()) {
        QMessageBox::information(this, "Export", "There is no angle/reference telemetry to export yet.");
        return;
    }

    davinci_arm::models::CsvExportOptions default_opts;
    default_opts.include_header_comments = true;
    default_opts.decimals = 6;
    default_opts.columns = {
        "joint_name",
        "domain",
        "t_rel_s",
        "arm_angle_deg",
        "ref_angle_deg",
        "tracking_error_deg",
        "valid"
    };

    auto* dialog = new davinci_arm::ui::widgets::ExportPreviewDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setSamples(samples);
    dialog->setDefaultOptions(default_opts);
    dialog->setDefaultFilename(
        QStringLiteral("DavinciArm_recording_%1.csv")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
    dialog->open();
}

void ControlPanelPage::onTelemetry(const davinci_arm::models::TelemetrySample& sample)
{
    if (!sample.valid) {
        updateRecordingUi_();
        return;
    }

    // Keep recording/export UI fresh regardless of active joint selection.
    updateRecordingUi_();

    const auto signal = signalTypeOf(sample);
    if (!signal.has_value()) {
        return;
    }

    if (*signal != TelemetrySignalType::Angle && *signal != TelemetrySignalType::AngleRef) {
        return;
    }

    if (resolveJointIndex_(sample) != active_joint_index_) {
        return;
    }

    if (const auto t_sec = timeSecOf(sample); t_sec.has_value()) {
        updateAdaptiveDensity_(sample.domain, *t_sec);
    }

    if (angle_ref_plot_) {
        angle_ref_plot_->pushSample(sample);
    }
    if (error_plot_) {
        error_plot_->pushSample(sample);
    }
}

void ControlPanelPage::setStreamLive(davinci_arm::models::Domain domain, bool live)
{
    if (angle_ref_plot_) {
        angle_ref_plot_->setStreamLive(domain, live);
    }
    if (error_plot_) {
        error_plot_->setStreamLive(domain, live);
    }

    if (domain == Domain::Real) {
        real_live_ = live;
    } else if (domain == Domain::Sim) {
        sim_live_ = live;
    }
}

}  // namespace davinci_arm::ui::pages