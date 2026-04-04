#include "davinci_arm_gui/core/services/drawing_service.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace davinci_arm::services {

DrawingService::DrawingService(QObject* parent)
    : QObject(parent) {}

DrawingService::DrawingService(std::shared_ptr<IDrawingPlannerClient> planner_client,
                               QObject* parent)
    : QObject(parent), planner_client_(std::move(planner_client)) {}

void DrawingService::setPlannerClient(std::shared_ptr<IDrawingPlannerClient> planner_client) noexcept {
    planner_client_ = std::move(planner_client);
    emit plannerAvailabilityChanged(isPlannerAvailable());
}

void DrawingService::setPlannerConfig(const davinci_arm::models::DrawingPlannerConfig& cfg) {
    planner_cfg_ = cfg;
}

void DrawingService::setWorkspace(const davinci_arm::models::DrawingWorkspaceCalibration& workspace) {
    workspace_ = workspace;
    emit workspaceChanged(workspace_);
    if (!document_.empty()) {
        setState_(workspace_.valid ? davinci_arm::models::DrawingExecutionState::Ready
                  : davinci_arm::models::DrawingExecutionState::Idle);
    }
}

void DrawingService::setDocument(const davinci_arm::models::DrawingDocument& document) {
    document_ = document;
    emit documentChanged();
    setState_((!document_.empty() && workspace_.valid)
              ? davinci_arm::models::DrawingExecutionState::Ready
              : davinci_arm::models::DrawingExecutionState::Idle);
}

void DrawingService::clearDocument() {
    document_ = davinci_arm::models::DrawingDocument{};
    last_plan_result_ = davinci_arm::models::DrawingPlanResult{};
    emit documentChanged();
    setState_(workspace_.valid ? davinci_arm::models::DrawingExecutionState::Ready
              : davinci_arm::models::DrawingExecutionState::Idle);
}

davinci_arm::models::DrawingPlannerConfig DrawingService::plannerConfig() const {
    return planner_cfg_;
}

davinci_arm::models::DrawingWorkspaceCalibration DrawingService::workspace() const {
    return workspace_;
}

davinci_arm::models::DrawingDocument DrawingService::document() const {
    return document_;
}

davinci_arm::models::DrawingPlanResult DrawingService::lastPlanResult() const {
    return last_plan_result_;
}

davinci_arm::models::DrawingExecutionState DrawingService::state() const noexcept {
    return state_;
}

bool DrawingService::isPlannerAvailable() const {
    return planner_client_ && planner_client_->ping();
}

bool DrawingService::previewTrajectory() {
    QString why_not;
    if (!canBuildRequest_(&why_not)) {
        emit statusMessage(why_not);
        setState_(davinci_arm::models::DrawingExecutionState::Failed);
        return false;
    }

    setState_(davinci_arm::models::DrawingExecutionState::Previewing);
    const auto result = planner_client_->preview(buildRequest_(true));
    last_plan_result_ = result;

    if (!result.success) {
        emit statusMessage(QString::fromStdString(result.message.empty() ? "Preview failed." : result.message));
        setState_(davinci_arm::models::DrawingExecutionState::Failed);
        return false;
    }

    emit planReady(result);
    emit statusMessage(QStringLiteral("Preview trajectory received from the external drawing planner node."));
    setState_(davinci_arm::models::DrawingExecutionState::Planned);
    return true;
}

bool DrawingService::generateTrajectory() {
    QString why_not;
    if (!canBuildRequest_(&why_not)) {
        emit statusMessage(why_not);
        setState_(davinci_arm::models::DrawingExecutionState::Failed);
        return false;
    }

    setState_(davinci_arm::models::DrawingExecutionState::Planning);
    const auto result = planner_client_->plan(buildRequest_(false));
    last_plan_result_ = result;

    if (!result.success) {
        emit statusMessage(QString::fromStdString(result.message.empty() ? "Planning failed." : result.message));
        setState_(davinci_arm::models::DrawingExecutionState::Failed);
        return false;
    }

    emit planReady(result);
    emit statusMessage(QStringLiteral("Drawing trajectory planned by the external MoveIt node."));
    setState_(davinci_arm::models::DrawingExecutionState::Planned);
    return true;
}

bool DrawingService::sendToPlanner() {
    if (!planner_client_) {
        emit statusMessage(QStringLiteral("Planner client is not wired."));
        return false;
    }

    if (last_plan_result_.external_job_id.empty()) {
        emit statusMessage(QStringLiteral("There is no planned external job ID to send yet."));
        return false;
    }

    setState_(davinci_arm::models::DrawingExecutionState::Sending);
    const bool ok = planner_client_->sendPlannedJob(last_plan_result_.external_job_id);
    if (!ok) {
        emit statusMessage(QStringLiteral("Failed to send the planned drawing job to the external planner node."));
        setState_(davinci_arm::models::DrawingExecutionState::Failed);
        return false;
    }

    emit statusMessage(QStringLiteral("Drawing job sent to the external planner node."));
    setState_(davinci_arm::models::DrawingExecutionState::Completed);
    return true;
}

void DrawingService::cancelActive() {
    if (planner_client_) planner_client_->cancelActiveRequest();
    setState_(davinci_arm::models::DrawingExecutionState::Cancelled);
    emit statusMessage(QStringLiteral("Active drawing request cancelled."));
}

bool DrawingService::exportPreviewWaypointsCsv(const std::string& filename) const {
    if (last_plan_result_.preview_waypoints.empty()) return false;

    std::filesystem::path p(filename);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;

    out << "x_m,y_m,z_m,qx,qy,qz,qw,pen_down\n";
    out << std::fixed << std::setprecision(6);

    for (const auto& wp : last_plan_result_.preview_waypoints) {
        out << wp.x_m << ','
            << wp.y_m << ','
            << wp.z_m << ','
            << wp.qx << ','
            << wp.qy << ','
            << wp.qz << ','
            << wp.qw << ','
            << (wp.pen_down ? 1 : 0) << '\n';
    }

    return out.good();
}

bool DrawingService::canBuildRequest_(QString* why_not) const {
    if (!planner_client_) {
        if (why_not) *why_not = QStringLiteral("Planner client is not wired.");
        return false;
    }
    if (!planner_client_->ping()) {
        if (why_not) *why_not = QStringLiteral("The external drawing planner node is not reachable.");
        return false;
    }
    if (document_.empty()) {
        if (why_not) *why_not = QStringLiteral("There is no drawing document loaded yet.");
        return false;
    }
    if (!workspace_.valid) {
        if (why_not) *why_not = QStringLiteral("The paper workspace has not been calibrated or is invalid.");
        return false;
    }
    return true;
}

davinci_arm::models::DrawingPlanRequest DrawingService::buildRequest_(bool preview_only) const {
    auto req = davinci_arm::models::DrawingPlanRequest{};
    req.document = document_;
    req.workspace = workspace_;
    req.planner = planner_cfg_;
    req.planner.preview_only = preview_only;
    return req;
}

void DrawingService::setState_(davinci_arm::models::DrawingExecutionState state) {
    if (state_ == state) return;
    state_ = state;
    emit stateChanged(state_);
}

} // namespace davinci_arm::services
