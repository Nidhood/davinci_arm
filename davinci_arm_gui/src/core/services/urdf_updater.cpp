#include "davinci_arm_gui/core/services/urdf_updater.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

namespace davinci_arm::services {

UrdfUpdater::UrdfUpdater(const std::filesystem::path& workspace_path)
    : workspace_path_(workspace_path)
{
    control_xacro_path_ = workspace_path_ / "src" / "davinci_arm_gazebo_control" / "urdf" / "davinci_arm_control.xacro";
    urdf_xacro_path_ = workspace_path_ / "src" / "davinci_arm_description" / "urdf" / "davinci_arm.urdf.xacro";
    backup_dir_ = workspace_path_ / "calibration_backups";
    std::filesystem::create_directories(backup_dir_);
}

bool UrdfUpdater::updateMotorParameters(const davinci_arm::models::MotorVelocityParams& params)
{
    last_error_.clear();
    if (!ensurePathsExist_()) return false;

    const auto content = readFile_(control_xacro_path_);
    if (!content) {
        last_error_ = "Failed to read control xacro file.";
        return false;
    }

    std::string updated = *content;
    updated = replaceParameter_(updated, "Kw", params.Kw);
    updated = replaceParameter_(updated, "tau_w", params.tau_w);
    updated = replaceParameter_(updated, "L_w", params.L_w);
    updated = replaceParameter_(updated, "motor_cmd_scale", params.motor_cmd_scale);

    if (!writeFile_(control_xacro_path_, updated)) {
        last_error_ = "Failed to write control xacro file.";
        return false;
    }

    return true;
}

bool UrdfUpdater::updatePhysicsParameters(const davinci_arm::models::ArmPhysicsParams& params)
{
    last_error_.clear();
    if (!ensurePathsExist_()) return false;

    const auto content = readFile_(urdf_xacro_path_);
    if (!content) {
        last_error_ = "Failed to read URDF xacro file.";
        return false;
    }

    std::string updated = *content;
    updated = replaceParameterInTag_(updated, "arm_link", "mass", params.mass);
    updated = replaceParameterInTag_(updated, "arm_link", "iyy", params.inertia_yy);
    updated = replaceParameterInTag_(updated, "arm_link_joint", "damping", params.damping);
    updated = replaceParameterInTag_(updated, "arm_link_joint", "friction", params.friction);

    if (!writeFile_(urdf_xacro_path_, updated)) {
        last_error_ = "Failed to write URDF xacro file.";
        return false;
    }

    return true;
}

bool UrdfUpdater::updateAllParameters(
    const davinci_arm::models::MotorVelocityParams& motor_params,
    const davinci_arm::models::ArmPhysicsParams& physics_params)
{
    if (!createBackup()) return false;

    if (!updateMotorParameters(motor_params)) {
        restoreFromBackup();
        return false;
    }

    if (!updatePhysicsParameters(physics_params)) {
        restoreFromBackup();
        return false;
    }

    return true;
}

bool UrdfUpdater::createBackup()
{
    last_error_.clear();
    if (!ensurePathsExist_()) return false;

    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "backup_" << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S");
    const auto backup_subdir = backup_dir_ / oss.str();
    std::filesystem::create_directories(backup_subdir);

    if (!backupFile_(control_xacro_path_, backup_subdir)) return false;
    if (!backupFile_(urdf_xacro_path_, backup_subdir)) return false;
    return true;
}

bool UrdfUpdater::restoreFromBackup()
{
    last_error_.clear();
    if (!std::filesystem::exists(backup_dir_)) {
        last_error_ = "Backup directory does not exist.";
        return false;
    }

    std::filesystem::path latest_backup;
    std::filesystem::file_time_type latest_time{};

    try {
        for (const auto& entry : std::filesystem::directory_iterator(backup_dir_)) {
            if (!entry.is_directory()) continue;
            const auto t = std::filesystem::last_write_time(entry.path());
            if (latest_backup.empty() || t > latest_time) {
                latest_backup = entry.path();
                latest_time = t;
            }
        }

        if (latest_backup.empty()) {
            last_error_ = "No backup found.";
            return false;
        }

        std::filesystem::copy_file(
            latest_backup / control_xacro_path_.filename(),
            control_xacro_path_,
            std::filesystem::copy_options::overwrite_existing);

        std::filesystem::copy_file(
            latest_backup / urdf_xacro_path_.filename(),
            urdf_xacro_path_,
            std::filesystem::copy_options::overwrite_existing);
    } catch (const std::exception& e) {
        last_error_ = std::string("Restore failed: ") + e.what();
        return false;
    }

    return true;
}

bool UrdfUpdater::validateUrdfFile(const std::filesystem::path& path) const
{
    if (!std::filesystem::exists(path)) return false;
    const auto content = readFile_(path);
    if (!content) return false;
    return content->find("<robot") != std::string::npos;
}

std::optional<std::string> UrdfUpdater::getLastError() const noexcept
{
    if (last_error_.empty()) return std::nullopt;
    return last_error_;
}

bool UrdfUpdater::ensurePathsExist_()
{
    if (!std::filesystem::exists(control_xacro_path_)) {
        last_error_ = "Control xacro path does not exist: " + control_xacro_path_.string();
        return false;
    }
    if (!std::filesystem::exists(urdf_xacro_path_)) {
        last_error_ = "URDF xacro path does not exist: " + urdf_xacro_path_.string();
        return false;
    }
    return true;
}

bool UrdfUpdater::backupFile_(const std::filesystem::path& src, const std::filesystem::path& dst_dir)
{
    try {
        std::filesystem::copy_file(src, dst_dir / src.filename(), std::filesystem::copy_options::overwrite_existing);
    } catch (const std::exception& e) {
        last_error_ = std::string("Backup failed for ") + src.string() + ": " + e.what();
        return false;
    }
    return true;
}

bool UrdfUpdater::writeFile_(const std::filesystem::path& path, const std::string& content)
{
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;
    out << content;
    return out.good();
}

std::optional<std::string> UrdfUpdater::readFile_(const std::filesystem::path& path) const
{
    std::ifstream in(path);
    if (!in.is_open()) return std::nullopt;
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

std::string UrdfUpdater::replaceParameter_(
    const std::string& content,
    const std::string& param_name,
    double value) const
{
    std::ostringstream value_ss;
    value_ss << std::fixed << std::setprecision(6) << value;

    const std::regex pattern(
        "(<param\\s+name=\\\"" + param_name + "\\\"[^>]*>)([^<]*)(</param>)",
        std::regex::icase);

    return std::regex_replace(content, pattern, "$1" + value_ss.str() + "$3");
}

std::string UrdfUpdater::replaceParameterInTag_(
    const std::string& content,
    const std::string& tag_name,
    const std::string& param_name,
    double value) const
{
    std::ostringstream value_ss;
    value_ss << std::fixed << std::setprecision(6) << value;

    std::string updated = content;

    if (param_name == "mass") {
        updated = std::regex_replace(updated,
                                     std::regex("(<mass\\s+value=\\\")[^\\\"]*(\\\")"),
                                     "$1" + value_ss.str() + "$2");
    } else if (param_name == "iyy") {
        updated = std::regex_replace(updated,
                                     std::regex("(iyy=\\\")[^\\\"]*(\\\")"),
                                     "$1" + value_ss.str() + "$2");
    } else if (param_name == "damping") {
        updated = std::regex_replace(updated,
                                     std::regex("(damping=\\\")[^\\\"]*(\\\")"),
                                     "$1" + value_ss.str() + "$2");
    } else if (param_name == "friction") {
        updated = std::regex_replace(updated,
                                     std::regex("(friction=\\\")[^\\\"]*(\\\")"),
                                     "$1" + value_ss.str() + "$2");
    }

    (void)tag_name;
    return updated;
}

} // namespace davinci_arm::services
