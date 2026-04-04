#pragma once

#include "davinci_arm_gui/core/models/drawing_types.hpp"
#include "davinci_arm_gui/core/services/drawing_interfaces.hpp"

#include <QObject>

#include <memory>
#include <optional>
#include <string>

namespace davinci_arm::services {

class DrawingService final : public QObject {
    Q_OBJECT

public:
    explicit DrawingService(QObject* parent = nullptr);
    explicit DrawingService(std::shared_ptr<IDrawingPlannerClient> planner_client,
                            QObject* parent = nullptr);

    void setPlannerClient(std::shared_ptr<IDrawingPlannerClient> planner_client) noexcept;
    void setPlannerConfig(const davinci_arm::models::DrawingPlannerConfig& cfg);
    void setWorkspace(const davinci_arm::models::DrawingWorkspaceCalibration& workspace);
    void setDocument(const davinci_arm::models::DrawingDocument& document);
    void clearDocument();

    [[nodiscard]] davinci_arm::models::DrawingPlannerConfig plannerConfig() const;
    [[nodiscard]] davinci_arm::models::DrawingWorkspaceCalibration workspace() const;
    [[nodiscard]] davinci_arm::models::DrawingDocument document() const;
    [[nodiscard]] davinci_arm::models::DrawingPlanResult lastPlanResult() const;
    [[nodiscard]] davinci_arm::models::DrawingExecutionState state() const noexcept;
    [[nodiscard]] bool isPlannerAvailable() const;

    bool previewTrajectory();
    bool generateTrajectory();
    bool sendToPlanner();
    void cancelActive();
    bool exportPreviewWaypointsCsv(const std::string& filename) const;

signals:
    void plannerAvailabilityChanged(bool available);
    void stateChanged(davinci_arm::models::DrawingExecutionState state);
    void documentChanged();
    void workspaceChanged(davinci_arm::models::DrawingWorkspaceCalibration workspace);
    void planReady(davinci_arm::models::DrawingPlanResult result);
    void statusMessage(QString message);

private:
    [[nodiscard]] bool canBuildRequest_(QString* why_not) const;
    [[nodiscard]] davinci_arm::models::DrawingPlanRequest buildRequest_(bool preview_only) const;
    void setState_(davinci_arm::models::DrawingExecutionState state);

private:
    std::shared_ptr<IDrawingPlannerClient> planner_client_;
    davinci_arm::models::DrawingPlannerConfig planner_cfg_{};
    davinci_arm::models::DrawingWorkspaceCalibration workspace_{};
    davinci_arm::models::DrawingDocument document_{};
    davinci_arm::models::DrawingPlanResult last_plan_result_{};
    davinci_arm::models::DrawingExecutionState state_{davinci_arm::models::DrawingExecutionState::Idle};
};

} // namespace davinci_arm::services
