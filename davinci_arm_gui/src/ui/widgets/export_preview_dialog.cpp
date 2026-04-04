#include "davinci_arm_gui/ui/widgets/export_preview_dialog.hpp"

#include "davinci_arm_gui/core/logging/data_exporter.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace davinci_arm::ui::widgets {

namespace {

QStandardItem* mkItem(const QString& txt) {
    auto* item = new QStandardItem(txt);
    item->setEditable(false);
    return item;
}

QString domainToUi(davinci_arm::models::Domain d) {
    switch (d) {
    case davinci_arm::models::Domain::Real:
        return QStringLiteral("Real");
    case davinci_arm::models::Domain::Sim:
        return QStringLiteral("Sim");
    case davinci_arm::models::Domain::Ref:
        return QStringLiteral("Ref");
    default:
        return QStringLiteral("Unknown");
    }
}

double radToDeg(double rad) {
    return rad * (180.0 / 3.14159265358979323846);
}

double trackingErrorRad(const davinci_arm::models::TelemetrySample& s) {
    return s.arm_angle_rad - s.ref_angle_rad;
}

double relTimeSeconds(const davinci_arm::models::TelemetrySample& s,
                      const davinci_arm::models::TelemetrySample& first) {
    return std::chrono::duration<double>(s.t - first.t).count();
}

}  // namespace

ExportPreviewDialog::ExportPreviewDialog(QWidget* parent)
    : QDialog(parent) {
    columns_ = {
        {QStringLiteral("Domain"), QStringLiteral("Domain")},
        {QStringLiteral("t_rel_s"), QStringLiteral("t_rel_s")},
        {QStringLiteral("arm_angle_rad"), QStringLiteral("arm_angle_rad")},
        {QStringLiteral("arm_angle_deg"), QStringLiteral("arm_angle_deg")},
        {QStringLiteral("ref_angle_rad"), QStringLiteral("ref_angle_rad")},
        {QStringLiteral("ref_angle_deg"), QStringLiteral("ref_angle_deg")},
        {QStringLiteral("tracking_error_rad"), QStringLiteral("tracking_error_rad")},
        {QStringLiteral("tracking_error_deg"), QStringLiteral("tracking_error_deg")},
        {QStringLiteral("motor_speed_rad_s"), QStringLiteral("motor_speed_rad_s")},
        {QStringLiteral("motor_speed_deg_s"), QStringLiteral("motor_speed_deg_s")},
        {QStringLiteral("pwm_us"), QStringLiteral("pwm_us")},
        {QStringLiteral("valid"), QStringLiteral("valid")},
    };

    default_opt_.include_header_comments = true;
    default_opt_.decimals = 6;
    default_opt_.columns = {
        "Domain",
        "t_rel_s",
        "arm_angle_deg",
        "ref_angle_deg",
        "tracking_error_deg",
        "motor_speed_rad_s",
        "pwm_us",
        "valid"
    };

    setWindowTitle(QStringLiteral("Export Preview"));
    setModal(true);
    resize(1080, 620);

    buildUi_();
}

void ExportPreviewDialog::setDefaultOptions(const davinci_arm::models::CsvExportOptions& opt) {
    default_opt_ = opt;

    if (include_header_comments_) {
        include_header_comments_->setChecked(default_opt_.include_header_comments);
    }
    if (decimals_) {
        decimals_->setValue(static_cast<double>(std::clamp(default_opt_.decimals, 0, 12)));
    }

    if (columns_list_) {
        for (int i = 0; i < columns_list_->count(); ++i) {
            if (auto* item = columns_list_->item(i)) {
                item->setCheckState(Qt::Unchecked);
            }
        }

        for (const auto& col : default_opt_.columns) {
            setCheckboxStateByKey_(QString::fromStdString(col), true);
        }
    }

    onOptionsChanged_();
}

void ExportPreviewDialog::setSamples(const QVector<davinci_arm::models::TelemetrySample>& samples) {
    samples_ = samples;
    rebuildModel_();
}

void ExportPreviewDialog::setDefaultFilename(const QString& name) {
    default_filename_ = name.trimmed();
}

void ExportPreviewDialog::buildUi_() {
    title_ = new QLabel(QStringLiteral("Telemetry Export Preview"), this);
    title_->setObjectName(QStringLiteral("title"));

    table_ = new QTableView(this);
    table_->setObjectName(QStringLiteral("panel"));
    table_->setAlternatingRowColors(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);

    model_ = new QStandardItemModel(this);
    table_->setModel(model_);

    auto* options_panel = buildOptionsPanel_();

    save_ = new QPushButton(QStringLiteral("Save CSV"), this);
    close_ = new QPushButton(QStringLiteral("Close"), this);

    connect(save_, &QPushButton::clicked, this, &ExportPreviewDialog::onSaveCsv_);
    connect(close_, &QPushButton::clicked, this, &QDialog::accept);

    auto* buttons = new QHBoxLayout();
    buttons->addStretch(1);
    buttons->addWidget(save_);
    buttons->addWidget(close_);

    auto* center = new QHBoxLayout();
    center->setSpacing(12);
    center->addWidget(options_panel);
    center->addWidget(table_, 1);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(10);
    root->addWidget(title_);
    root->addLayout(center, 1);
    root->addLayout(buttons);

    setDefaultOptions(default_opt_);
}

QWidget* ExportPreviewDialog::buildOptionsPanel_() {
    auto* container = new QWidget(this);
    container->setObjectName(QStringLiteral("optionsPanel"));

    options_group_ = new QGroupBox(QStringLiteral("Export Options"), container);
    options_group_->setObjectName(QStringLiteral("controlGroup"));

    columns_list_ = new QListWidget(options_group_);
    columns_list_->setObjectName(QStringLiteral("panel"));
    columns_list_->setMinimumWidth(320);

    for (const auto& col : columns_) {
        auto* item = new QListWidgetItem(col.label, columns_list_);
        item->setData(Qt::UserRole, col.key);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        columns_list_->addItem(item);
    }

    include_header_comments_ = new QCheckBox(QStringLiteral("Include header comments"), options_group_);
    include_header_comments_->setChecked(true);

    decimals_ = new QDoubleSpinBox(options_group_);
    decimals_->setDecimals(0);
    decimals_->setRange(0, 12);
    decimals_->setValue(6);
    decimals_->setSuffix(QStringLiteral(" decimals"));

    auto* format_row = new QHBoxLayout();
    format_row->addWidget(include_header_comments_, 1);
    format_row->addWidget(decimals_, 0);

    auto* filters_box = new QGroupBox(QStringLiteral("Filters (preview + export)"), options_group_);
    filters_box->setObjectName(QStringLiteral("controlSubGroup"));

    filter_real_ = new QCheckBox(QStringLiteral("Real"), filters_box);
    filter_sim_ = new QCheckBox(QStringLiteral("Sim"), filters_box);
    filter_ref_ = new QCheckBox(QStringLiteral("Ref"), filters_box);
    show_valid_only_ = new QCheckBox(QStringLiteral("Valid only"), filters_box);

    filter_real_->setChecked(true);
    filter_sim_->setChecked(true);
    filter_ref_->setChecked(true);
    show_valid_only_->setChecked(true);

    auto* filters_layout = new QVBoxLayout(filters_box);
    filters_layout->setSpacing(6);
    filters_layout->addWidget(filter_real_);
    filters_layout->addWidget(filter_sim_);
    filters_layout->addWidget(filter_ref_);
    filters_layout->addWidget(show_valid_only_);

    auto* options_layout = new QVBoxLayout(options_group_);
    options_layout->setContentsMargins(12, 12, 12, 12);
    options_layout->setSpacing(10);
    options_layout->addWidget(new QLabel(QStringLiteral("Columns:"), options_group_));
    options_layout->addWidget(columns_list_, 1);
    options_layout->addLayout(format_row);
    options_layout->addWidget(filters_box);

    auto* root = new QVBoxLayout(container);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(options_group_);

    connect(columns_list_, &QListWidget::itemChanged, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(include_header_comments_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(decimals_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ExportPreviewDialog::onOptionsChanged_);
    connect(filter_real_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(filter_sim_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(filter_ref_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(show_valid_only_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);

    return container;
}

void ExportPreviewDialog::onOptionsChanged_() {
    rebuildModel_();
}

bool ExportPreviewDialog::isColumnChecked_(const QString& key) const {
    if (!columns_list_) {
        return false;
    }

    for (int i = 0; i < columns_list_->count(); ++i) {
        const auto* item = columns_list_->item(i);
        if (item && item->data(Qt::UserRole).toString() == key) {
            return item->checkState() == Qt::Checked;
        }
    }

    return false;
}

void ExportPreviewDialog::setCheckboxStateByKey_(const QString& key, bool checked) {
    if (!columns_list_) {
        return;
    }

    for (int i = 0; i < columns_list_->count(); ++i) {
        auto* item = columns_list_->item(i);
        if (item && item->data(Qt::UserRole).toString() == key) {
            item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
            return;
        }
    }
}

davinci_arm::models::CsvExportOptions ExportPreviewDialog::collectOptions_() const {
    davinci_arm::models::CsvExportOptions opt;
    opt.include_header_comments = include_header_comments_ ? include_header_comments_->isChecked() : true;
    opt.decimals = decimals_ ? std::clamp(static_cast<int>(decimals_->value()), 0, 12) : 6;
    opt.sort_by_time = true;

    for (const auto& col : columns_) {
        if (isColumnChecked_(col.key)) {
            opt.columns.push_back(col.key.toStdString());
        }
    }

    if (opt.columns.empty()) {
        opt.columns = {
            "Domain",
            "t_rel_s",
            "arm_angle_deg"
        };
    }

    return opt;
}

std::vector<davinci_arm::models::TelemetrySample>
ExportPreviewDialog::toStd_(const QVector<davinci_arm::models::TelemetrySample>& v) const {
    std::vector<davinci_arm::models::TelemetrySample> out;
    out.reserve(static_cast<std::size_t>(v.size()));
    for (const auto& sample : v) {
        out.push_back(sample);
    }
    return out;
}

void ExportPreviewDialog::rebuildModel_() {
    model_->clear();

    std::vector<QString> keys;
    keys.reserve(columns_.size());
    for (const auto& col : columns_) {
        if (isColumnChecked_(col.key)) {
            keys.push_back(col.key);
        }
    }

    if (keys.empty()) {
        keys = {
            QStringLiteral("Domain"),
            QStringLiteral("t_rel_s"),
            QStringLiteral("arm_angle_deg")
        };
    }

    QStringList headers;
    for (const auto& key : keys) {
        headers << key;
    }
    model_->setHorizontalHeaderLabels(headers);

    const bool want_real = filter_real_ ? filter_real_->isChecked() : true;
    const bool want_sim = filter_sim_ ? filter_sim_->isChecked() : true;
    const bool want_ref = filter_ref_ ? filter_ref_->isChecked() : true;
    const bool valid_only = show_valid_only_ ? show_valid_only_->isChecked() : false;
    const int decimals = decimals_ ? static_cast<int>(decimals_->value()) : 6;

    std::vector<davinci_arm::models::TelemetrySample> filtered;
    filtered.reserve(static_cast<std::size_t>(samples_.size()));

    for (const auto& sample : samples_) {
        if (valid_only && !sample.valid) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Real && !want_real) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Sim && !want_sim) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Ref && !want_ref) {
            continue;
        }
        filtered.push_back(sample);
    }

    std::sort(filtered.begin(), filtered.end(), [](const auto& a, const auto& b) {
        return a.t < b.t;
    });

    model_->setRowCount(0);
    if (filtered.empty()) {
        table_->resizeColumnsToContents();
        return;
    }

    const auto& first = filtered.front();
    int row = 0;

    for (const auto& sample : filtered) {
        model_->insertRow(row);

        for (int c = 0; c < static_cast<int>(keys.size()); ++c) {
            const auto& key = keys[static_cast<std::size_t>(c)];

            if (key == QStringLiteral("Domain")) {
                model_->setItem(row, c, mkItem(domainToUi(sample.domain)));
            } else if (key == QStringLiteral("t_rel_s")) {
                model_->setItem(row, c, mkItem(QString::number(relTimeSeconds(sample, first), 'f', decimals)));
            } else if (key == QStringLiteral("arm_angle_rad")) {
                model_->setItem(row, c, mkItem(QString::number(sample.arm_angle_rad, 'f', decimals)));
            } else if (key == QStringLiteral("arm_angle_deg")) {
                model_->setItem(row, c, mkItem(QString::number(radToDeg(sample.arm_angle_rad), 'f', decimals)));
            } else if (key == QStringLiteral("ref_angle_rad")) {
                model_->setItem(row, c, mkItem(QString::number(sample.ref_angle_rad, 'f', decimals)));
            } else if (key == QStringLiteral("ref_angle_deg")) {
                model_->setItem(row, c, mkItem(QString::number(radToDeg(sample.ref_angle_rad), 'f', decimals)));
            } else if (key == QStringLiteral("tracking_error_rad")) {
                model_->setItem(row, c, mkItem(QString::number(trackingErrorRad(sample), 'f', decimals)));
            } else if (key == QStringLiteral("tracking_error_deg")) {
                model_->setItem(row, c, mkItem(QString::number(radToDeg(trackingErrorRad(sample)), 'f', decimals)));
            } else if (key == QStringLiteral("motor_speed_rad_s")) {
                model_->setItem(row, c, mkItem(QString::number(sample.motor_speed_rad_s, 'f', decimals)));
            } else if (key == QStringLiteral("motor_speed_deg_s")) {
                model_->setItem(row, c, mkItem(QString::number(radToDeg(sample.motor_speed_rad_s), 'f', decimals)));
            } else if (key == QStringLiteral("pwm_us")) {
                model_->setItem(row, c, mkItem(QString::number(sample.pwm_us)));
            } else if (key == QStringLiteral("valid")) {
                model_->setItem(row, c, mkItem(sample.valid ? QStringLiteral("true") : QStringLiteral("false")));
            } else {
                model_->setItem(row, c, mkItem(QString()));
            }
        }

        ++row;
        if (row >= 3000) {
            break;
        }
    }

    table_->resizeColumnsToContents();
}

void ExportPreviewDialog::onSaveCsv_() {
    if (samples_.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Export"), QStringLiteral("No samples available."));
        return;
    }

    const QString suggested = default_filename_.isEmpty()
                              ? QStringLiteral("telemetry.csv")
                              : default_filename_;

    const QString path = QFileDialog::getSaveFileName(
                             this,
                             QStringLiteral("Save CSV"),
                             suggested,
                             QStringLiteral("CSV Files (*.csv);;All Files (*)"));

    if (path.isEmpty()) {
        return;
    }

    QVector<davinci_arm::models::TelemetrySample> filtered;
    filtered.reserve(samples_.size());

    const bool want_real = filter_real_ ? filter_real_->isChecked() : true;
    const bool want_sim = filter_sim_ ? filter_sim_->isChecked() : true;
    const bool want_ref = filter_ref_ ? filter_ref_->isChecked() : true;
    const bool valid_only = show_valid_only_ ? show_valid_only_->isChecked() : false;

    for (const auto& sample : samples_) {
        if (valid_only && !sample.valid) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Real && !want_real) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Sim && !want_sim) {
            continue;
        }
        if (sample.domain == davinci_arm::models::Domain::Ref && !want_ref) {
            continue;
        }
        filtered.push_back(sample);
    }

    if (filtered.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Export"), QStringLiteral("No samples match the selected filters."));
        return;
    }

    const auto options = collectOptions_();

    davinci_arm::core::logging::DataExporter exporter;
    const bool ok = exporter.exportToCsv(path.toStdString(), toStd_(filtered), options);

    if (!ok) {
        QMessageBox::critical(
            this,
            QStringLiteral("Export Failed"),
            QStringLiteral("Failed to export data:\n%1").arg(QString::fromStdString(exporter.lastError())));
        return;
    }

    QMessageBox::information(
        this,
        QStringLiteral("Export Complete"),
        QStringLiteral("Saved %1 samples to:\n%2").arg(filtered.size()).arg(path));
}

}  // namespace davinci_arm::ui::widgets
