#pragma once

#include <string>
#include <vector>

#include "davinci_arm_gui/core/models/csv_export_options.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

namespace davinci_arm::core::logging {

class DataExporter final {
public:
    DataExporter() = default;

    bool exportToCsv(const std::string& filename,
                     const std::vector<davinci_arm::models::TelemetrySample>& samples,
                     const davinci_arm::models::CsvExportOptions& opt = {});

    [[nodiscard]] const std::string& lastError() const noexcept {
        return last_error_;
    }

private:
    static std::vector<std::string> defaultColumns_();
    static bool isSupportedColumn_(const std::string& col);
    static std::string domainToString_(davinci_arm::models::Domain d);
    static double relTimeSeconds_(const davinci_arm::models::TelemetrySample& s,
                                  const davinci_arm::models::TelemetrySample& first);
    static double trackingErrorRad_(const davinci_arm::models::TelemetrySample& s);
    static double radToDeg_(double rad);
    static std::string formatFloat_(double v, int decimals);
    static std::string join_(const std::vector<std::string>& cols, char sep);
    static std::string escapeCommentValue_(const std::string& value);

private:
    std::string last_error_;
};

}  // namespace davinci_arm::core::logging
