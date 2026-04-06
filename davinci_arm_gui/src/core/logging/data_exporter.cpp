#include "davinci_arm_gui/core/logging/data_exporter.hpp"

#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>

namespace davinci_arm::core::logging {

using davinci_arm::models::TelemetrySample;
using davinci_arm::models::TelemetrySignalType;

namespace {

std::string normalizeColumn(std::string col)
{
    if (col == "Domain") return "domain";
    if (col == "Joint" || col == "joint") return "joint_name";
    if (col == "Signal") return "signal";
    return col;
}

}  // namespace

std::vector<std::string> DataExporter::defaultColumns_()
{
    return {
        "joint_name",
        "domain",
        "t_rel_s",
        "arm_angle_deg",
        "ref_angle_deg",
        "tracking_error_deg",
        "valid"
    };
}

bool DataExporter::isSupportedColumn_(const std::string& col)
{
    const auto key = normalizeColumn(col);

    static const std::vector<std::string> kSupported = {
        "joint_name",
        "domain",
        "signal",
        "t_rel_s",
        "arm_angle_rad",
        "arm_angle_deg",
        "ref_angle_rad",
        "ref_angle_deg",
        "tracking_error_rad",
        "tracking_error_deg",
        "motor_speed_rad_s",   // tolerated as legacy
        "motor_speed_deg_s",   // tolerated as legacy
        "pwm_us",              // tolerated as legacy
        "valid"
    };

    return std::find(kSupported.begin(), kSupported.end(), key) != kSupported.end();
}

bool DataExporter::exportToCsv(const std::string& filename,
                               const std::vector<TelemetrySample>& samples,
                               const davinci_arm::models::CsvExportOptions& opt)
{
    last_error_.clear();

    if (samples.empty()) {
        last_error_ = "No data to export (samples is empty).";
        return false;
    }

    const int decimals = std::clamp(opt.decimals, 0, 12);
    const auto requested_columns = opt.columns.empty() ? defaultColumns_() : opt.columns;

    for (const auto& col : requested_columns) {
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

    std::vector<TelemetrySample> data = samples;
    if (opt.sort_by_time) {
        std::sort(data.begin(), data.end(), [](const auto& a, const auto& b) {
            return a.t < b.t;
        });
    }

    const auto& first = data.front();

    if (opt.include_header_comments) {
        out << "# DavinciArm telemetry export\n";
        out << "# Points=" << data.size() << "\n";
        out << "# Columns=" << escapeCommentValue_(join_(requested_columns, ',')) << "\n";
        out << "# Decimals=" << decimals << "\n";
        out << "# SortedByTime=" << (opt.sort_by_time ? "true" : "false") << "\n";
        out << "#\n";
    }

    for (std::size_t i = 0; i < requested_columns.size(); ++i) {
        out << requested_columns[i];
        if (i + 1 < requested_columns.size()) {
            out << ',';
        }
    }
    out << '\n';

    for (const auto& s : data) {
        for (std::size_t i = 0; i < requested_columns.size(); ++i) {
            const auto key = normalizeColumn(requested_columns[i]);

            if (key == "joint_name") {
                out << s.joint_name;
            } else if (key == "domain") {
                out << domainToString_(s.domain);
            } else if (key == "signal") {
                out << signalToString_(s.signal);
            } else if (key == "valid") {
                out << (s.valid ? "1" : "0");
            } else if (key == "t_rel_s") {
                out << formatFloat_(relTimeSeconds_(s, first), decimals);
            } else {
                const auto value = columnNumericValue_(s, key);
                if (value.has_value()) {
                    out << formatFloat_(*value, decimals);
                }
            }

            if (i + 1 < requested_columns.size()) {
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

std::string DataExporter::domainToString_(davinci_arm::models::Domain d)
{
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

std::string DataExporter::signalToString_(davinci_arm::models::TelemetrySignalType s)
{
    switch (s) {
    case TelemetrySignalType::Angle:
        return "angle";
    case TelemetrySignalType::MotorSpeed:
        return "motor_speed";
    case TelemetrySignalType::PwmFeedback:
        return "pwm_feedback";
    case TelemetrySignalType::AngleRef:
        return "angle_ref";
    default:
        return "unknown";
    }
}

double DataExporter::relTimeSeconds_(const TelemetrySample& s,
                                     const TelemetrySample& first)
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(s.t - first.t).count();
}

double DataExporter::wrapRadPi_(double rad)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double two_pi = 2.0 * pi;

    while (rad <= -pi) {
        rad += two_pi;
    }
    while (rad > pi) {
        rad -= two_pi;
    }
    return rad;
}

double DataExporter::trackingErrorRad_(const TelemetrySample& s)
{
    return wrapRadPi_(s.ref_angle_rad - s.arm_angle_rad);
}

double DataExporter::radToDeg_(double rad)
{
    return rad * (180.0 / 3.14159265358979323846);
}

double DataExporter::mapJointRadToUiDeg_(double rad)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double two_pi = 2.0 * pi;
    constexpr double eps = 1e-9;

    double wrapped = std::remainder(rad, two_pi);

    if (std::abs(wrapped + pi) < eps) {
        return 0.0;
    }
    if (std::abs(wrapped - pi) < eps) {
        return 360.0;
    }

    double deg = (wrapped + pi) * 180.0 / pi;

    if (deg < 0.0) {
        deg += 360.0;
    }
    if (deg > 360.0) {
        deg -= 360.0;
    }

    return deg;
}

std::optional<double> DataExporter::columnNumericValue_(const TelemetrySample& s,
        const std::string& col)
{
    if (col == "motor_speed_rad_s") {
        return s.motor_speed_rad_s;
    }
    if (col == "motor_speed_deg_s") {
        return radToDeg_(s.motor_speed_rad_s);
    }
    if (col == "pwm_us") {
        return static_cast<double>(s.pwm_us);
    }

    switch (s.signal) {
    case TelemetrySignalType::Angle:
        if (col == "arm_angle_rad") return s.arm_angle_rad;
        if (col == "arm_angle_deg") return mapJointRadToUiDeg_(s.arm_angle_rad);
        if (col == "ref_angle_rad") return s.ref_angle_rad;
        if (col == "ref_angle_deg") return mapJointRadToUiDeg_(s.ref_angle_rad);
        if (col == "tracking_error_rad") return trackingErrorRad_(s);
        if (col == "tracking_error_deg") return radToDeg_(trackingErrorRad_(s));
        return std::nullopt;

    case TelemetrySignalType::AngleRef:
        if (col == "ref_angle_rad") return s.ref_angle_rad;
        if (col == "ref_angle_deg") return mapJointRadToUiDeg_(s.ref_angle_rad);
        return std::nullopt;

    case TelemetrySignalType::MotorSpeed:
        if (col == "motor_speed_rad_s") return s.motor_speed_rad_s;
        if (col == "motor_speed_deg_s") return radToDeg_(s.motor_speed_rad_s);
        return std::nullopt;

    case TelemetrySignalType::PwmFeedback:
    default:
        return std::nullopt;
    }
}

std::string DataExporter::formatFloat_(double v, int decimals)
{
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(decimals) << v;
    return ss.str();
}

std::string DataExporter::join_(const std::vector<std::string>& cols, char sep)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        ss << cols[i];
        if (i + 1 < cols.size()) {
            ss << sep;
        }
    }
    return ss.str();
}

std::string DataExporter::escapeCommentValue_(const std::string& value)
{
    std::string out = value;
    std::replace(out.begin(), out.end(), '\n', ' ');
    std::replace(out.begin(), out.end(), '\r', ' ');
    return out;
}

}  // namespace davinci_arm::core::logging