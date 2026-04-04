#pragma once

#include "davinci_arm_gui/core/models/calibration_types.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace davinci_arm::services {

class UrdfUpdater final {
public:
    explicit UrdfUpdater(const std::filesystem::path& workspace_path);

    bool updateMotorParameters(const davinci_arm::models::MotorVelocityParams& params);
    bool updatePhysicsParameters(const davinci_arm::models::ArmPhysicsParams& params);
    bool updateAllParameters(
        const davinci_arm::models::MotorVelocityParams& motor_params,
        const davinci_arm::models::ArmPhysicsParams& physics_params);

    bool createBackup();
    bool restoreFromBackup();

    [[nodiscard]] bool validateUrdfFile(const std::filesystem::path& path) const;
    [[nodiscard]] std::optional<std::string> getLastError() const noexcept;

private:
    bool ensurePathsExist_();
    bool backupFile_(const std::filesystem::path& src, const std::filesystem::path& dst_dir);

    bool writeFile_(const std::filesystem::path& path, const std::string& content);
    [[nodiscard]] std::optional<std::string> readFile_(const std::filesystem::path& path) const;

    [[nodiscard]] std::string replaceParameter_(
        const std::string& content,
        const std::string& param_name,
        double value) const;

    [[nodiscard]] std::string replaceParameterInTag_(
        const std::string& content,
        const std::string& tag_name,
        const std::string& param_name,
        double value) const;

private:
    std::filesystem::path workspace_path_;
    std::filesystem::path control_xacro_path_;
    std::filesystem::path urdf_xacro_path_;
    std::filesystem::path backup_dir_;

    mutable std::string last_error_;
};

} // namespace davinci_arm::services
