#pragma once

#include <QDialog>
#include <QVector>

#include <vector>

#include "davinci_arm_gui/core/models/csv_export_options.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

QT_BEGIN_NAMESPACE
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QStandardItemModel;
class QTableView;
QT_END_NAMESPACE

namespace davinci_arm::ui::widgets {

class ExportPreviewDialog final : public QDialog {
    Q_OBJECT

public:
    explicit ExportPreviewDialog(QWidget* parent = nullptr);

    void setSamples(const QVector<davinci_arm::models::TelemetrySample>& samples);
    void setDefaultOptions(const davinci_arm::models::CsvExportOptions& opt);
    void setDefaultFilename(const QString& name);

private slots:
    void onOptionsChanged_();
    void onSaveCsv_();

private:
    struct ColumnDef {
        QString key;
        QString label;
    };

    void buildUi_();
    QWidget* buildOptionsPanel_();
    void rebuildModel_();
    davinci_arm::models::CsvExportOptions collectOptions_() const;
    std::vector<davinci_arm::models::TelemetrySample> toStd_(const QVector<davinci_arm::models::TelemetrySample>& v) const;

    void setCheckboxStateByKey_(const QString& key, bool checked);
    bool isColumnChecked_(const QString& key) const;

private:
    QLabel* title_{nullptr};
    QTableView* table_{nullptr};
    QStandardItemModel* model_{nullptr};
    QString default_filename_;

    QGroupBox* options_group_{nullptr};
    QListWidget* columns_list_{nullptr};
    QCheckBox* include_header_comments_{nullptr};
    QDoubleSpinBox* decimals_{nullptr};

    QCheckBox* filter_real_{nullptr};
    QCheckBox* filter_sim_{nullptr};
    QCheckBox* filter_ref_{nullptr};
    QCheckBox* show_valid_only_{nullptr};

    QPushButton* save_{nullptr};
    QPushButton* close_{nullptr};

    QVector<davinci_arm::models::TelemetrySample> samples_;
    std::vector<ColumnDef> columns_;
    davinci_arm::models::CsvExportOptions default_opt_;
};

}  // namespace davinci_arm::ui::widgets
