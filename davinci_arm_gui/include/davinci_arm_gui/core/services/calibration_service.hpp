#pragma once

#include "davinci_arm_gui/core/models/calibration_types.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/services/calibration_interfaces.hpp"
#include "davinci_arm_gui/infra/ros/calibration_command_sink.hpp"

#include <QObject>
#include <QString>

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace davinci_arm::models {
class TelemetryStore;
}

namespace davinci_arm::services {

class UrdfUpdater;

class CalibrationService final : public QObject {
    Q_OBJECT

public:
    explicit CalibrationService(QObject* parent = nullptr);
    explicit CalibrationService(davinci_arm::models::TelemetryStore* store,
                                std::shared_ptr<ICalibrationCommander> commander,
                                std::shared_ptr<ISimParamApplier> sim_param_applier = {},
                                std::shared_ptr<IRobotPoseProvider> pose_provider = {},
                                std::shared_ptr<IWorkspacePersistence> workspace_persistence = {},
                                QObject* parent = nullptr);
    ~CalibrationService() override;

    CalibrationService(const CalibrationService&) = delete;
    CalibrationService& operator=(const CalibrationService&) = delete;

    void setTelemetryStore(davinci_arm::models::TelemetryStore* store) noexcept;

    // New wiring API
    void setCommander(std::shared_ptr<ICalibrationCommander> commander) noexcept;
    void setSimParamApplier(std::shared_ptr<ISimParamApplier> sim_param_applier) noexcept;
    void setPoseProvider(std::shared_ptr<IRobotPoseProvider> pose_provider) noexcept;
    void setWorkspacePersistence(std::shared_ptr<IWorkspacePersistence> persistence) noexcept;

    // Backward-compatible wiring API used by existing app_context / ROS bridge code.
    void setCommandSink(ICalibrationCommandSink* sink) noexcept;
    void setUrdfUpdater(UrdfUpdater* updater) noexcept;
    void setSimReloadHook(ISimReloadHook* hook) noexcept;

    void startCalibration(const davinci_arm::models::CalibrationConfig& cfg);
    void stopCalibration();
    void applyCalibration();
    void resetToDefaults();

    void beginWorkspaceCalibration(const davinci_arm::models::PaperCalibrationConfig& cfg);
    bool captureWorkspaceOrigin();
    bool captureWorkspaceXAxisPoint();
    bool captureWorkspaceYAxisPoint();
    bool captureWorkspaceHoverPoint();
    bool solveWorkspaceCalibration();
    bool applyWorkspaceCalibration();
    void resetWorkspaceCalibration();

    [[nodiscard]] davinci_arm::models::CalibrationResult getLastResult() const;
    [[nodiscard]] davinci_arm::models::MotorVelocityParams getMotorParams() const;
    [[nodiscard]] davinci_arm::models::ArmPhysicsParams getPhysicsParams() const;
    [[nodiscard]] davinci_arm::models::DrawingWorkspaceCalibration getDrawingWorkspace() const;
    [[nodiscard]] davinci_arm::models::PaperCalibrationConfig getPaperCalibrationConfig() const;
    [[nodiscard]] std::string getWorkspaceCaptureStage() const;

signals:
    void statusChanged(davinci_arm::models::CalibrationStatus status);
    void progressUpdated(double progress01);
    void calibrationCompleted(davinci_arm::models::CalibrationResult result);
    void metricsUpdated(davinci_arm::models::CalibrationMetrics metrics);
    void parametersChanged();
    void workspaceCalibrationUpdated(davinci_arm::models::DrawingWorkspaceCalibration workspace);
    void workspaceCaptureStageChanged(QString stage);
    void workspaceValidationMessage(QString message);

private:
    using TimePoint = std::chrono::steady_clock::time_point;

    void runWorker_(davinci_arm::models::CalibrationConfig cfg, std::stop_token st);
    [[nodiscard]] bool canRunMotorPhysics_() const noexcept;
    [[nodiscard]] bool canRunWorkspace_() const noexcept;

    [[nodiscard]] bool runMotorExperimentLegacy_(const davinci_arm::models::CalibrationConfig& cfg,
            std::stop_token st);
    [[nodiscard]] bool runPhysicsExperimentLegacy_(const davinci_arm::models::CalibrationConfig& cfg,
            std::stop_token st);

    [[nodiscard]] bool applyMotorParamsToSim_(const davinci_arm::models::MotorVelocityParams& p);
    [[nodiscard]] bool applyPhysicsParamsToSim_(const davinci_arm::models::ArmPhysicsParams& p);
    [[nodiscard]] bool reloadSimulation_();
    void stopAllOutputs_();

    double evaluateMotorCost_(const davinci_arm::models::CalibrationConfig& cfg,
                              const davinci_arm::models::MotorVelocityParams& p,
                              std::stop_token st,
                              davinci_arm::models::CalibrationMetrics* out_metrics);

    double evaluatePhysicsCost_(const davinci_arm::models::CalibrationConfig& cfg,
                                const davinci_arm::models::ArmPhysicsParams& p,
                                std::stop_token st,
                                davinci_arm::models::CalibrationMetrics* out_metrics);

    [[nodiscard]] std::vector<davinci_arm::models::TelemetrySample> collectWindow_(
        davinci_arm::models::Domain domain,
        TimePoint t0,
        TimePoint t1,
        std::size_t max_points) const;

    [[nodiscard]] static davinci_arm::models::CalibrationMetrics makeMetrics_(
        double rmse,
        double max_error,
        double mean_error,
        double corr,
        std::size_t n);

    void setStatus_(davinci_arm::models::CalibrationStatus s);
    void setProgress_(double p01);

    [[nodiscard]] std::optional<davinci_arm::models::CartesianPose> capturePose_(davinci_arm::models::Domain domain);
    [[nodiscard]] bool hasAllWorkspaceCaptures_() const noexcept;
    [[nodiscard]] davinci_arm::models::DrawingWorkspaceCalibration computeWorkspace_() const;

private:
    davinci_arm::models::TelemetryStore* store_{nullptr};

    std::shared_ptr<ICalibrationCommander> commander_;
    std::shared_ptr<ISimParamApplier> sim_param_applier_;
    std::shared_ptr<IRobotPoseProvider> pose_provider_;
    std::shared_ptr<IWorkspacePersistence> workspace_persistence_;

    // Legacy wiring used by older app_context code.
    ICalibrationCommandSink* command_sink_{nullptr};
    UrdfUpdater* urdf_updater_{nullptr};
    ISimReloadHook* sim_reload_hook_{nullptr};

    mutable std::mutex state_mtx_;

    davinci_arm::models::MotorVelocityParams motor_params_{};
    davinci_arm::models::ArmPhysicsParams physics_params_{};
    davinci_arm::models::CalibrationResult last_result_{};
    davinci_arm::models::CalibrationStatus status_{davinci_arm::models::CalibrationStatus::Idle};

    davinci_arm::models::PaperCalibrationConfig paper_cfg_{};
    davinci_arm::models::DrawingWorkspaceCalibration workspace_{};
    std::optional<davinci_arm::models::CartesianPose> captured_origin_;
    std::optional<davinci_arm::models::CartesianPose> captured_x_axis_;
    std::optional<davinci_arm::models::CartesianPose> captured_y_axis_;
    std::optional<davinci_arm::models::CartesianPose> captured_hover_;
    std::string workspace_stage_{"idle"};

    std::jthread worker_;
};

} // namespace davinci_arm::services
