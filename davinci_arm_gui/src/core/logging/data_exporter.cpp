#include "davinci_arm_gui/core/logging/data_exporter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace davinci_arm::core::logging {

std::vector<std::string> DataExporter::defaultColumns_() {
    return {
        "Domain",
        "t_rel_s",
        "arm_angle_deg",
        "ref_angle_deg",
        "tracking_error_deg",
        "motor_speed_rad_s",
        "pwm_us",
        "valid"
    };
}

bool DataExporter::isSupportedColumn_(const std::string& col) {
    static const std::vector<std::string> kSupported = {
        "Domain",
        "t_rel_s",
        "arm_angle_rad",
        "arm_angle_deg",
        "motor_speed_rad_s",
        "motor_speed_deg_s",
        "pwm_us",
        "ref_angle_rad",
        "ref_angle_deg",
        "tracking_error_rad",
        "tracking_error_deg",
        "valid"
    };

    return std::find(kSupported.begin(), kSupported.end(), col) != kSupported.end();
}

bool DataExporter::exportToCsv(const std::string& filename,
                               const std::vector<davinci_arm::models::TelemetrySample>& samples,
                               const davinci_arm::models::CsvExportOptions& opt) {
    last_error_.clear();

    if (samples.empty()) {
        last_error_ = "No data to export (samples is empty).";
        return false;
    }

    const int decimals = std::clamp(opt.decimals, 0, 12);
    const auto columns = opt.columns.empty() ? defaultColumns_() : opt.columns;

    for (const auto& col : columns) {
        if (!isSupportedColumn_(col)) {
            last_error_ = "Unknown column name in export request: " + col;
            return false;
        }
    }

    try {
        const std::filesystem::path p(filename);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to create output directory: ") + e.what();
        return false;
    }

    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        last_error_ = "Failed to open output file: " + filename;
        return false;
    }

    std::vector<davinci_arm::models::TelemetrySample> data = samples;
    if (opt.sort_by_time) {
        std::sort(data.begin(), data.end(), [](const auto& a, const auto& b) {
            return a.t < b.t;
        });
    }

    const auto& first = data.front();

    if (opt.include_header_comments) {
        out << "# DavinciArm telemetry export\n";
        out << "# Points=" << data.size() << "\n";
        out << "# Columns=" << escapeCommentValue_(join_(columns, ',')) << "\n";
        out << "# Decimals=" << decimals << "\n";
        out << "# SortedByTime=" << (opt.sort_by_time ? "true" : "false") << "\n";
        out << "#\n";
    }

    for (std::size_t i = 0; i < columns.size(); ++i) {
        out << columns[i];
        if (i + 1 < columns.size()) {
            out << ',';
        }
    }
    out << '\n';

    for (const auto& s : data) {
        for (std::size_t i = 0; i < columns.size(); ++i) {
            const auto& col = columns[i];

            if (col == "Domain") {
                out << domainToString_(s.domain);
            } else if (col == "t_rel_s") {
                out << formatFloat_(relTimeSeconds_(s, first), decimals);
            } else if (col == "arm_angle_rad") {
                out << formatFloat_(s.arm_angle_rad, decimals);
            } else if (col == "arm_angle_deg") {
                out << formatFloat_(radToDeg_(s.arm_angle_rad), decimals);
            } else if (col == "motor_speed_rad_s") {
                out << formatFloat_(s.motor_speed_rad_s, decimals);
            } else if (col == "motor_speed_deg_s") {
                out << formatFloat_(radToDeg_(s.motor_speed_rad_s), decimals);
            } else if (col == "pwm_us") {
                out << s.pwm_us;
            } else if (col == "ref_angle_rad") {
                out << formatFloat_(s.ref_angle_rad, decimals);
            } else if (col == "ref_angle_deg") {
                out << formatFloat_(radToDeg_(s.ref_angle_rad), decimals);
            } else if (col == "tracking_error_rad") {
                out << formatFloat_(trackingErrorRad_(s), decimals);
            } else if (col == "tracking_error_deg") {
                out << formatFloat_(radToDeg_(trackingErrorRad_(s)), decimals);
            } else if (col == "valid") {
                out << (s.valid ? "1" : "0");
            }

            if (i + 1 < columns.size()) {
                out << ',';
            }
        }
        out << '\n';
    }

    out.flush();
    if (!out.good()) {
        last_error_ = "Failed while writing CSV data.";
        return false;
    }

    return true;
}

std::string DataExporter::domainToString_(davinci_arm::models::Domain d) {
    switch (d) {
    case davinci_arm::models::Domain::Real:
        return "real";
    case davinci_arm::models::Domain::Sim:
        return "sim";
    case davinci_arm::models::Domain::Ref:
        return "ref";
    default:
        return "unknown";
    }
}

double DataExporter::relTimeSeconds_(const davinci_arm::models::TelemetrySample& s,
                                     const davinci_arm::models::TelemetrySample& first) {
    using namespace std::chrono;
    return duration_cast<duration<double>>(s.t - first.t).count();
}

double DataExporter::trackingErrorRad_(const davinci_arm::models::TelemetrySample& s) {
    return s.arm_angle_rad - s.ref_angle_rad;
}

double DataExporter::radToDeg_(double rad) {
    return rad * (180.0 / 3.14159265358979323846);
}

std::string DataExporter::formatFloat_(double v, int decimals) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(decimals) << v;
    return ss.str();
}

std::string DataExporter::join_(const std::vector<std::string>& cols, char sep) {
    std::ostringstream ss;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        ss << cols[i];
        if (i + 1 < cols.size()) {
            ss << sep;
        }
    }
    return ss.str();
}

std::string DataExporter::escapeCommentValue_(const std::string& value) {
    std::string out = value;
    std::replace(out.begin(), out.end(), '\n', ' ');
    std::replace(out.begin(), out.end(), '\r', ' ');
    return out;
}

}  // namespace davinci_arm::core::logging
