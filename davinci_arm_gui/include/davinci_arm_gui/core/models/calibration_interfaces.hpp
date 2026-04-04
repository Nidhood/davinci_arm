#pragma once

#include "davinci_arm_gui/core/models/calibration_types.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"

#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace davinci_arm::services {

class ISimParamApplier {
public:
    virtual ~ISimParamApplier() = default;

    virtual bool applyMotorParams(const davinci_arm::models::MotorVelocityParams& p) = 0;
    virtual bool applyPhysicsParams(const davinci_arm::models::ArmPhysicsParams& p) = 0;
    virtual bool reloadSimulation() = 0;
    [[nodiscard]] virtual std::optional<std::string> lastError() const noexcept = 0;
};

class ICalibrationCommander {
public:
    virtual ~ICalibrationCommander() = default;

    virtual bool runMotorSequence(
        const std::vector<double>& pwm_steps,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st,
        bool use_real,
        bool use_sim) = 0;

    virtual bool runPhysicsSequence(
        const std::vector<double>& ref_steps_rad,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st,
        bool use_real,
        bool use_sim) = 0;

    virtual void stopAll() = 0;
};

class IRobotPoseProvider {
public:
    virtual ~IRobotPoseProvider() = default;

    [[nodiscard]] virtual std::optional<davinci_arm::models::CartesianPose>
    currentToolPose(davinci_arm::models::Domain domain) const = 0;
};

class IWorkspacePersistence {
public:
    virtual ~IWorkspacePersistence() = default;

    virtual bool saveWorkspace(const davinci_arm::models::DrawingWorkspaceCalibration& workspace) = 0;
    [[nodiscard]] virtual std::optional<davinci_arm::models::DrawingWorkspaceCalibration> loadWorkspace() const = 0;
    [[nodiscard]] virtual std::optional<std::string> lastError() const noexcept = 0;
};

} // namespace davinci_arm::services
