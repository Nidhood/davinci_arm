#include "davinci_arm_gui/core/services/calibration_service.hpp"

#include "davinci_arm_gui/core/models/telemetry_store.hpp"
#include "davinci_arm_gui/core/services/calibration_optimizer.hpp"
#include "davinci_arm_gui/core/services/signal_alignment.hpp"
#include "davinci_arm_gui/core/services/urdf_updater.hpp"
#include "davinci_arm_gui/core/services/signal_alignment.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <thread>

namespace davinci_arm::services {

using davinci_arm::models::CalibrationConfig;
using davinci_arm::models::CalibrationMetrics;
using davinci_arm::models::CalibrationResult;
using davinci_arm::models::CalibrationStatus;
using davinci_arm::models::CalibrationType;
using davinci_arm::models::CartesianPose;
using davinci_arm::models::Domain;
using davinci_arm::models::DrawingWorkspaceCalibration;
using davinci_arm::models::PaperCalibrationConfig;
using davinci_arm::models::TelemetrySignalType;

namespace {

constexpr double kHugeCost = 1.0e9;
constexpr std::size_t kHistoryPoints = 40000;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vec3 toVec3(const CartesianPose& p) {
    return {p.x_m, p.y_m, p.z_m};
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 mul(const Vec3& a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Vec3 normalized(const Vec3& a) {
    const double n = norm(a);
    if (n <= 1.0e-12) return {0.0, 0.0, 0.0};
    return mul(a, 1.0 / n);
}

double radToDeg(double rad) {
    return rad * 180.0 / std::numbers::pi;
}

} // namespace

CalibrationService::CalibrationService(QObject* parent)
    : QObject(parent)
{
    last_result_.status = CalibrationStatus::Idle;
}

CalibrationService::CalibrationService(davinci_arm::models::TelemetryStore* store,
                                       std::shared_ptr<ICalibrationCommander> commander,
                                       std::shared_ptr<ISimParamApplier> sim_param_applier,
                                       std::shared_ptr<IRobotPoseProvider> pose_provider,
                                       std::shared_ptr<IWorkspacePersistence> workspace_persistence,
                                       QObject* parent)
    : QObject(parent),
      store_(store),
      commander_(std::move(commander)),
      sim_param_applier_(std::move(sim_param_applier)),
      pose_provider_(std::move(pose_provider)),
      workspace_persistence_(std::move(workspace_persistence))
{
    last_result_.status = CalibrationStatus::Idle;

    if (workspace_persistence_) {
        if (const auto saved = workspace_persistence_->loadWorkspace(); saved.has_value()) {
            workspace_ = *saved;
            last_result_.drawing_workspace = workspace_;
        }
    }
}

CalibrationService::~CalibrationService() {
    stopCalibration();
}

void CalibrationService::setTelemetryStore(davinci_arm::models::TelemetryStore* store) noexcept {
    store_ = store;
}

void CalibrationService::setCommander(std::shared_ptr<ICalibrationCommander> commander) noexcept {
    commander_ = std::move(commander);
}

void CalibrationService::setSimParamApplier(std::shared_ptr<ISimParamApplier> sim_param_applier) noexcept {
    sim_param_applier_ = std::move(sim_param_applier);
}

void CalibrationService::setPoseProvider(std::shared_ptr<IRobotPoseProvider> pose_provider) noexcept {
    pose_provider_ = std::move(pose_provider);
}

void CalibrationService::setWorkspacePersistence(std::shared_ptr<IWorkspacePersistence> persistence) noexcept {
    workspace_persistence_ = std::move(persistence);
}

void CalibrationService::setCommandSink(ICalibrationCommandSink* sink) noexcept {
    command_sink_ = sink;
}

void CalibrationService::setUrdfUpdater(UrdfUpdater* updater) noexcept {
    urdf_updater_ = updater;
}

void CalibrationService::setSimReloadHook(ISimReloadHook* hook) noexcept {
    sim_reload_hook_ = hook;
}

void CalibrationService::startCalibration(const CalibrationConfig& cfg) {
    if (cfg.type == CalibrationType::DrawingWorkspace) {
        beginWorkspaceCalibration(cfg.paper_config);
        return;
    }

    if (!canRunMotorPhysics_()) {
        setStatus_(CalibrationStatus::Failed);
        CalibrationResult r;
        r.status = CalibrationStatus::Failed;
        r.type = cfg.type;
        r.error_message = "Calibration service is not wired: telemetry store and command path are missing.";
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            last_result_ = r;
        }
        emit calibrationCompleted(r);
        return;
    }

    stopCalibration();
    setStatus_(CalibrationStatus::Running);
    setProgress_(0.0);

    worker_ = std::jthread([this, cfg](std::stop_token st) {
        runWorker_(cfg, st);
    });
}

void CalibrationService::stopCalibration() {
    if (worker_.joinable()) {
        worker_.request_stop();
        stopAllOutputs_();
        worker_.join();
    }

    if (status_ == CalibrationStatus::Running || status_ == CalibrationStatus::Analyzing) {
        setStatus_(CalibrationStatus::Cancelled);
    }
    setProgress_(0.0);
}

void CalibrationService::applyCalibration() {
    davinci_arm::models::MotorVelocityParams motor;
    davinci_arm::models::ArmPhysicsParams phys;
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        motor = motor_params_;
        phys = physics_params_;
    }

    const bool ok_motor = applyMotorParamsToSim_(motor);
    const bool ok_phys = applyPhysicsParamsToSim_(phys);
    const bool ok_reload = reloadSimulation_();

    if (!ok_motor || !ok_phys || !ok_reload) {
        emit workspaceValidationMessage(QStringLiteral("Failed to apply simulator parameters."));
        return;
    }

    emit parametersChanged();
}

void CalibrationService::resetToDefaults() {
    stopCalibration();

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        motor_params_ = davinci_arm::models::MotorVelocityParams{};
        physics_params_ = davinci_arm::models::ArmPhysicsParams{};
        workspace_ = DrawingWorkspaceCalibration{};
        paper_cfg_ = PaperCalibrationConfig{};
        captured_origin_.reset();
        captured_x_axis_.reset();
        captured_y_axis_.reset();
        captured_hover_.reset();
        workspace_stage_ = "idle";
        last_result_ = CalibrationResult{};
        last_result_.status = CalibrationStatus::Idle;
        status_ = CalibrationStatus::Idle;
    }

    emit parametersChanged();
    emit metricsUpdated(last_result_.metrics);
    emit workspaceCalibrationUpdated(workspace_);
    emit workspaceCaptureStageChanged(QStringLiteral("idle"));
    emit statusChanged(CalibrationStatus::Idle);
    emit progressUpdated(0.0);
}

void CalibrationService::beginWorkspaceCalibration(const PaperCalibrationConfig& cfg) {
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        paper_cfg_ = cfg;
        workspace_ = DrawingWorkspaceCalibration{};
        captured_origin_.reset();
        captured_x_axis_.reset();
        captured_y_axis_.reset();
        captured_hover_.reset();
        workspace_stage_ = "capture_origin";
        last_result_.type = CalibrationType::DrawingWorkspace;
        last_result_.status = CalibrationStatus::Idle;
    }

    emit workspaceCaptureStageChanged(QStringLiteral("capture_origin"));
    emit workspaceValidationMessage(
        QStringLiteral("Workspace calibration started. Capture the paper origin on the real robot."));
}

bool CalibrationService::captureWorkspaceOrigin() {
    const auto pose = capturePose_(Domain::Real);
    if (!pose.has_value()) return false;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        captured_origin_ = *pose;
        workspace_stage_ = "capture_x_axis";
    }

    emit workspaceCaptureStageChanged(QStringLiteral("capture_x_axis"));
    emit workspaceValidationMessage(
        QStringLiteral("Origin captured. Now capture the point that defines the paper X axis."));
    return true;
}

bool CalibrationService::captureWorkspaceXAxisPoint() {
    const auto pose = capturePose_(Domain::Real);
    if (!pose.has_value()) return false;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        captured_x_axis_ = *pose;
        workspace_stage_ = "capture_y_axis";
    }

    emit workspaceCaptureStageChanged(QStringLiteral("capture_y_axis"));
    emit workspaceValidationMessage(
        QStringLiteral("X-axis point captured. Now capture the point that defines the paper Y axis."));
    return true;
}

bool CalibrationService::captureWorkspaceYAxisPoint() {
    const auto pose = capturePose_(Domain::Real);
    if (!pose.has_value()) return false;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        captured_y_axis_ = *pose;
        workspace_stage_ = "capture_hover";
    }

    emit workspaceCaptureStageChanged(QStringLiteral("capture_hover"));
    emit workspaceValidationMessage(
        QStringLiteral("Y-axis point captured. Now capture a safe hover point above the paper."));
    return true;
}

bool CalibrationService::captureWorkspaceHoverPoint() {
    const auto pose = capturePose_(Domain::Real);
    if (!pose.has_value()) return false;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        captured_hover_ = *pose;
        workspace_stage_ = "solve";
    }

    emit workspaceCaptureStageChanged(QStringLiteral("solve"));
    emit workspaceValidationMessage(
        QStringLiteral("Hover point captured. Solve the workspace to validate the paper plane and drawing altitude."));
    return true;
}

bool CalibrationService::solveWorkspaceCalibration() {
    if (!hasAllWorkspaceCaptures_()) {
        emit workspaceValidationMessage(
            QStringLiteral("Workspace calibration is incomplete. Capture origin, X, Y and hover poses first."));
        return false;
    }

    const auto solved = computeWorkspace_();
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        workspace_ = solved;
        last_result_.type = CalibrationType::DrawingWorkspace;
        last_result_.drawing_workspace = solved;
        last_result_.status = solved.valid ? CalibrationStatus::Completed : CalibrationStatus::Failed;
        workspace_stage_ = solved.valid ? "ready" : "invalid";
    }

    emit workspaceCalibrationUpdated(solved);
    emit workspaceCaptureStageChanged(QString::fromStdString(getWorkspaceCaptureStage()));
    emit calibrationCompleted(getLastResult());

    if (!solved.valid) {
        emit workspaceValidationMessage(QString::fromStdString(solved.notes));
        return false;
    }

    emit workspaceValidationMessage(
        QStringLiteral("Workspace solved successfully. The drawing plane, hover height and paper extents are ready."));
    return true;
}

bool CalibrationService::applyWorkspaceCalibration() {
    DrawingWorkspaceCalibration workspace_copy;
    std::shared_ptr<IWorkspacePersistence> persistence;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        workspace_copy = workspace_;
        persistence = workspace_persistence_;
    }

    if (!workspace_copy.valid) {
        emit workspaceValidationMessage(
            QStringLiteral("Cannot apply workspace calibration because it is not valid yet."));
        return false;
    }

    if (persistence && !persistence->saveWorkspace(workspace_copy)) {
        emit workspaceValidationMessage(QStringLiteral("Workspace persistence failed."));
        return false;
    }

    emit workspaceCalibrationUpdated(workspace_copy);
    return true;
}

void CalibrationService::resetWorkspaceCalibration() {
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        workspace_ = DrawingWorkspaceCalibration{};
        captured_origin_.reset();
        captured_x_axis_.reset();
        captured_y_axis_.reset();
        captured_hover_.reset();
        workspace_stage_ = "idle";
    }

    emit workspaceCalibrationUpdated(DrawingWorkspaceCalibration{});
    emit workspaceCaptureStageChanged(QStringLiteral("idle"));
}

CalibrationResult CalibrationService::getLastResult() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return last_result_;
}

davinci_arm::models::MotorVelocityParams CalibrationService::getMotorParams() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return motor_params_;
}

davinci_arm::models::ArmPhysicsParams CalibrationService::getPhysicsParams() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return physics_params_;
}

DrawingWorkspaceCalibration CalibrationService::getDrawingWorkspace() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return workspace_;
}

PaperCalibrationConfig CalibrationService::getPaperCalibrationConfig() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return paper_cfg_;
}

std::string CalibrationService::getWorkspaceCaptureStage() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return workspace_stage_;
}

void CalibrationService::runWorker_(CalibrationConfig cfg, std::stop_token st) {
    CalibrationResult local_result;
    local_result.type = cfg.type;
    local_result.status = CalibrationStatus::Running;

    setStatus_(CalibrationStatus::Running);
    setProgress_(0.0);

    OptimizerConfig opt_cfg;
    opt_cfg.max_iterations = std::max<std::size_t>(1, cfg.max_iterations);

    const double target_rmse = std::max(1.0e-6, cfg.convergence_threshold);

    if (cfg.type == CalibrationType::MotorVelocity) {
        const auto p0 = getMotorParams();
        std::vector<double> x0 = {p0.Kw, p0.tau_w, p0.L_w, p0.motor_cmd_scale};
        std::vector<ParamBound> bounds = {
            {0.01, 50.0},
            {0.01, 5.0},
            {0.0, 2.0},
            {0.1, 10.0},
        };

        auto cost_fn = [&](const std::vector<double>& x) -> double {
            if (st.stop_requested()) return kHugeCost;

            davinci_arm::models::MotorVelocityParams p;
            p.Kw = x[0];
            p.tau_w = x[1];
            p.L_w = x[2];
            p.motor_cmd_scale = x[3];

            CalibrationMetrics metrics;
            const double cost = evaluateMotorCost_(cfg, p, st, &metrics);

            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                motor_params_ = p;
                last_result_.type = cfg.type;
                last_result_.metrics = metrics;
                last_result_.motor_params = p;
                last_result_.status = status_;
            }
            emit parametersChanged();
            emit metricsUpdated(metrics);
            return cost;
        };

        const auto res = CoordinateDescentOptimizer::optimize(
                             x0,
                             bounds,
                             opt_cfg,
                             cost_fn,
        [&](double progress01, double /*best_cost*/) {
            setProgress_(progress01);
        });

        local_result.motor_params.Kw = res.best_x[0];
        local_result.motor_params.tau_w = res.best_x[1];
        local_result.motor_params.L_w = res.best_x[2];
        local_result.motor_params.motor_cmd_scale = res.best_x[3];

        CalibrationMetrics metrics;
        const double final_cost = evaluateMotorCost_(cfg, local_result.motor_params, st, &metrics);
        local_result.metrics = metrics;
        local_result.status = st.stop_requested() ? CalibrationStatus::Cancelled : CalibrationStatus::Completed;
        local_result.error_message = (final_cost <= target_rmse)
                                     ? std::string{}
                                     :
                                     std::string{"Calibration finished above target RMSE."};
    } else if (cfg.type == CalibrationType::ArmPhysics) {
        const auto p0 = getPhysicsParams();
        std::vector<double> x0 = {p0.mass, p0.inertia_yy, p0.damping, p0.friction};
        std::vector<ParamBound> bounds = {
            {0.05, 10.0},
            {1.0e-5, 1.0},
            {0.0, 1.0},
            {0.0, 1.0},
        };

        auto cost_fn = [&](const std::vector<double>& x) -> double {
            if (st.stop_requested()) return kHugeCost;

            davinci_arm::models::ArmPhysicsParams p;
            p.mass = x[0];
            p.inertia_yy = x[1];
            p.damping = x[2];
            p.friction = x[3];

            CalibrationMetrics metrics;
            const double cost = evaluatePhysicsCost_(cfg, p, st, &metrics);

            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                physics_params_ = p;
                last_result_.type = cfg.type;
                last_result_.metrics = metrics;
                last_result_.physics_params = p;
                last_result_.status = status_;
            }
            emit parametersChanged();
            emit metricsUpdated(metrics);
            return cost;
        };

        const auto res = CoordinateDescentOptimizer::optimize(
                             x0,
                             bounds,
                             opt_cfg,
                             cost_fn,
        [&](double progress01, double /*best_cost*/) {
            setProgress_(progress01);
        });

        local_result.physics_params.mass = res.best_x[0];
        local_result.physics_params.inertia_yy = res.best_x[1];
        local_result.physics_params.damping = res.best_x[2];
        local_result.physics_params.friction = res.best_x[3];

        CalibrationMetrics metrics;
        const double final_cost = evaluatePhysicsCost_(cfg, local_result.physics_params, st, &metrics);
        local_result.metrics = metrics;
        local_result.status = st.stop_requested() ? CalibrationStatus::Cancelled : CalibrationStatus::Completed;
        local_result.error_message = (final_cost <= target_rmse)
                                     ? std::string{}
                                     :
                                     std::string{"Calibration finished above target RMSE."};
    }

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_result_ = local_result;
        if (cfg.type == CalibrationType::MotorVelocity) {
            motor_params_ = local_result.motor_params;
        } else if (cfg.type == CalibrationType::ArmPhysics) {
            physics_params_ = local_result.physics_params;
        }
    }

    setStatus_(local_result.status);
    setProgress_(1.0);
    emit calibrationCompleted(local_result);
    emit metricsUpdated(local_result.metrics);

    if (cfg.auto_apply && local_result.status == CalibrationStatus::Completed) {
        applyCalibration();
    }
}

bool CalibrationService::canRunMotorPhysics_() const noexcept {
    return store_ != nullptr && (static_cast<bool>(commander_) || command_sink_ != nullptr);
}

bool CalibrationService::canRunWorkspace_() const noexcept {
    return static_cast<bool>(pose_provider_);
}

bool CalibrationService::runMotorExperimentLegacy_(const CalibrationConfig& cfg, std::stop_token st) {
    if (!command_sink_) return false;

    if (cfg.use_real) command_sink_->sendAutoMode(Domain::Real, false);
    if (cfg.use_sim) command_sink_->sendAutoMode(Domain::Sim, false);

    auto sleep_or_stop = [&](double sec) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::duration<double>(std::max(0.0, sec)));
        constexpr auto kTick = std::chrono::milliseconds(20);
        while (remaining.count() > 0) {
            if (st.stop_requested()) return false;
            const auto step = (remaining > kTick) ? kTick : remaining;
            std::this_thread::sleep_for(step);
            remaining -= step;
        }
        return true;
    };

    for (double in : cfg.test_inputs) {
        if (st.stop_requested()) {
            stopAllOutputs_();
            return false;
        }

        const auto pwm = static_cast<std::uint16_t>(std::clamp(in, 0.0, 65535.0));
        if (cfg.use_real) command_sink_->sendPwmUs(Domain::Real, pwm);
        if (cfg.use_sim) command_sink_->sendPwmUs(Domain::Sim, pwm);

        if (!sleep_or_stop(cfg.settling_time_sec) || !sleep_or_stop(cfg.duration_sec)) {
            stopAllOutputs_();
            return false;
        }
    }

    stopAllOutputs_();
    return !st.stop_requested();
}

bool CalibrationService::runPhysicsExperimentLegacy_(const CalibrationConfig& cfg, std::stop_token st) {
    if (!command_sink_) return false;

    if (cfg.use_real) command_sink_->sendAutoMode(Domain::Real, true);
    if (cfg.use_sim) command_sink_->sendAutoMode(Domain::Sim, true);

    auto sleep_or_stop = [&](double sec) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::duration<double>(std::max(0.0, sec)));
        constexpr auto kTick = std::chrono::milliseconds(20);
        while (remaining.count() > 0) {
            if (st.stop_requested()) return false;
            const auto step = (remaining > kTick) ? kTick : remaining;
            std::this_thread::sleep_for(step);
            remaining -= step;
        }
        return true;
    };

    for (double ref_rad : cfg.test_inputs) {
        if (st.stop_requested()) {
            stopAllOutputs_();
            return false;
        }

        if (cfg.use_real) command_sink_->sendRefAngleRad(Domain::Real, ref_rad);
        if (cfg.use_sim) command_sink_->sendRefAngleRad(Domain::Sim, ref_rad);

        if (!sleep_or_stop(cfg.settling_time_sec) || !sleep_or_stop(cfg.duration_sec)) {
            stopAllOutputs_();
            return false;
        }
    }

    stopAllOutputs_();
    return !st.stop_requested();
}

bool CalibrationService::applyMotorParamsToSim_(const davinci_arm::models::MotorVelocityParams& p) {
    if (sim_param_applier_) return sim_param_applier_->applyMotorParams(p);
    if (urdf_updater_) return urdf_updater_->updateMotorParameters(p);
    return true;
}

bool CalibrationService::applyPhysicsParamsToSim_(const davinci_arm::models::ArmPhysicsParams& p) {
    if (sim_param_applier_) return sim_param_applier_->applyPhysicsParams(p);
    if (urdf_updater_) return urdf_updater_->updatePhysicsParameters(p);
    return true;
}

bool CalibrationService::reloadSimulation_() {
    if (sim_param_applier_) return sim_param_applier_->reloadSimulation();
    if (sim_reload_hook_) return sim_reload_hook_->reloadSimulation();
    return true;
}

void CalibrationService::stopAllOutputs_() {
    if (commander_) {
        commander_->stopAll();
        return;
    }
    if (!command_sink_) return;
    command_sink_->sendStop(Domain::Real);
    command_sink_->sendStop(Domain::Sim);
}

double CalibrationService::evaluateMotorCost_(
    const CalibrationConfig& cfg,
    const davinci_arm::models::MotorVelocityParams& p,
    std::stop_token st,
    CalibrationMetrics* out_metrics)
{
    if (!cfg.use_real || !cfg.use_sim) return kHugeCost;

    if (!applyMotorParamsToSim_(p)) return kHugeCost;
    if (!reloadSimulation_()) return kHugeCost;

    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = commander_
                    ? commander_->runMotorSequence(cfg.test_inputs, cfg.duration_sec, cfg.settling_time_sec, st, cfg.use_real, cfg.use_sim)
                    : runMotorExperimentLegacy_(cfg, st);
    if (!ok) return kHugeCost;
    const auto t1 = std::chrono::steady_clock::now();

    const auto real = collectWindow_(Domain::Real, t0, t1, kHistoryPoints);
    const auto sim = collectWindow_(Domain::Sim, t0, t1, kHistoryPoints);
    const auto aligned = davinci_arm::core::services::SignalAlignment::alignAndResample(
                             real,
                             sim,
                             davinci_arm::models::PlotSignalType::MotorSpeedMetS,
                             0.01);
    if (!aligned) return kHugeCost;

    const auto e = davinci_arm::core::services::SignalAlignment::computeError(*aligned);
    if (out_metrics) {
        *out_metrics = makeMetrics_(e.rmse, e.max_abs, e.mean_abs, e.correlation, e.n);
    }
    return e.rmse;
}

double CalibrationService::evaluatePhysicsCost_(
    const CalibrationConfig& cfg,
    const davinci_arm::models::ArmPhysicsParams& p,
    std::stop_token st,
    CalibrationMetrics* out_metrics)
{
    if (!cfg.use_real || !cfg.use_sim) return kHugeCost;

    if (!applyPhysicsParamsToSim_(p)) return kHugeCost;
    if (!reloadSimulation_()) return kHugeCost;

    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = commander_
                    ? commander_->runPhysicsSequence(cfg.test_inputs, cfg.duration_sec, cfg.settling_time_sec, st, cfg.use_real, cfg.use_sim)
                    : runPhysicsExperimentLegacy_(cfg, st);
    if (!ok) return kHugeCost;
    const auto t1 = std::chrono::steady_clock::now();

    const auto real = collectWindow_(Domain::Real, t0, t1, kHistoryPoints);
    const auto sim = collectWindow_(Domain::Sim, t0, t1, kHistoryPoints);
    const auto aligned = davinci_arm::core::services::SignalAlignment::alignAndResample(
                             real,
                             sim,
                             davinci_arm::models::PlotSignalType::ArmAngleDeg,
                             0.01);
    if (!aligned) return kHugeCost;

    const auto e = davinci_arm::core::services::SignalAlignment::computeError(*aligned);
    if (out_metrics) {
        *out_metrics = makeMetrics_(e.rmse, e.max_abs, e.mean_abs, e.correlation, e.n);
    }
    return e.rmse;
}

std::vector<davinci_arm::models::TelemetrySample> CalibrationService::collectWindow_(
    Domain domain,
    TimePoint t0,
    TimePoint t1,
    std::size_t max_points) const
{
    if (!store_) return {};

    const auto hist = store_->history(domain, max_points);
    std::vector<davinci_arm::models::TelemetrySample> out;
    out.reserve(hist.size());

    for (const auto& s : hist) {
        if (!s.valid) continue;
        if (s.t < t0 || s.t > t1) continue;
        out.push_back(s);
    }

    return out;
}

CalibrationMetrics CalibrationService::makeMetrics_(
    double rmse,
    double max_error,
    double mean_error,
    double corr,
    std::size_t n)
{
    CalibrationMetrics m;
    m.rmse = rmse;
    m.max_error = max_error;
    m.mean_error = mean_error;
    m.correlation = corr;
    m.sample_count = n;
    return m;
}

void CalibrationService::setStatus_(CalibrationStatus s) {
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        status_ = s;
        last_result_.status = s;
    }
    emit statusChanged(s);
}

void CalibrationService::setProgress_(double p01) {
    emit progressUpdated(std::clamp(p01, 0.0, 1.0));
}

std::optional<CartesianPose> CalibrationService::capturePose_(Domain domain) {
    if (!canRunWorkspace_()) {
        emit workspaceValidationMessage(QStringLiteral("Pose provider is not wired."));
        return std::nullopt;
    }

    const auto pose = pose_provider_->currentToolPose(domain);
    if (!pose.has_value() || !pose->valid) {
        emit workspaceValidationMessage(QStringLiteral("Current tool pose is not available."));
        return std::nullopt;
    }
    return pose;
}

bool CalibrationService::hasAllWorkspaceCaptures_() const noexcept {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return captured_origin_.has_value() && captured_x_axis_.has_value() &&
           captured_y_axis_.has_value() && captured_hover_.has_value();
}

DrawingWorkspaceCalibration CalibrationService::computeWorkspace_() const {
    std::lock_guard<std::mutex> lk(state_mtx_);

    DrawingWorkspaceCalibration out;
    if (!(captured_origin_ && captured_x_axis_ && captured_y_axis_ && captured_hover_)) {
        out.notes = "Workspace captures are incomplete.";
        return out;
    }

    out.origin_pose = *captured_origin_;
    out.x_axis_pose = *captured_x_axis_;
    out.y_axis_pose = *captured_y_axis_;
    out.hover_pose = *captured_hover_;

    const Vec3 O = toVec3(*captured_origin_);
    const Vec3 X = toVec3(*captured_x_axis_);
    const Vec3 Y = toVec3(*captured_y_axis_);
    const Vec3 H = toVec3(*captured_hover_);

    const Vec3 vx = sub(X, O);
    const Vec3 vy_raw = sub(Y, O);

    const double width = norm(vx);
    if (width <= 1.0e-9) {
        out.notes = "The X-axis capture is too close to the origin.";
        return out;
    }

    const Vec3 ex = normalized(vx);
    const Vec3 vy_proj = sub(vy_raw, mul(ex, dot(vy_raw, ex)));
    const double height = norm(vy_proj);
    if (height <= 1.0e-9) {
        out.notes = "The Y-axis capture is colinear with the X-axis.";
        return out;
    }

    const Vec3 ey = normalized(vy_proj);
    const Vec3 ez = normalized(cross(ex, ey));
    if (norm(ez) <= 1.0e-9) {
        out.notes = "The paper plane normal could not be computed.";
        return out;
    }

    const double right_angle_cos = std::clamp(dot(normalized(vy_raw), ex), -1.0, 1.0);
    const double angle_between = std::acos(std::abs(right_angle_cos));
    const double rectangularity_error = std::abs(90.0 - radToDeg(angle_between));

    const double hover_height = dot(sub(H, O), ez);

    const double width_rel_error = std::abs(width - paper_cfg_.paper_width_m) /
                                   std::max(paper_cfg_.paper_width_m, 1.0e-9);
    const double height_rel_error = std::abs(height - paper_cfg_.paper_height_m) /
                                    std::max(paper_cfg_.paper_height_m, 1.0e-9);

    out.measured_width_m = width;
    out.measured_height_m = height;
    out.drawable_width_m = std::max(0.0, width - paper_cfg_.margin_left_m - paper_cfg_.margin_right_m);
    out.drawable_height_m = std::max(0.0, height - paper_cfg_.margin_top_m - paper_cfg_.margin_bottom_m);
    out.hover_height_m = (paper_cfg_.desired_hover_height_m > 0.0)
                         ? paper_cfg_.desired_hover_height_m
                         : hover_height;
    out.draw_height_m = paper_cfg_.desired_draw_height_offset_m;
    out.rectangularity_error_deg = rectangularity_error;
    out.width_rel_error = width_rel_error;
    out.height_rel_error = height_rel_error;
    out.ex = {ex.x, ex.y, ex.z};
    out.ey = {ey.x, ey.y, ey.z};
    out.ez = {ez.x, ez.y, ez.z};

    std::ostringstream notes;
    notes << "Measured paper: " << width << " m x " << height << " m. "
          << "Rectangularity error: " << rectangularity_error << " deg. "
          << "Hover height: " << hover_height << " m.";
    out.notes = notes.str();

    out.valid = rectangularity_error <= paper_cfg_.max_rectangularity_error_deg &&
                width_rel_error <= paper_cfg_.max_size_rel_error &&
                height_rel_error <= paper_cfg_.max_size_rel_error &&
                out.drawable_width_m > 0.0 &&
                out.drawable_height_m > 0.0 &&
                hover_height > 0.0;

    if (!out.valid) {
        out.notes += " Validation failed against the configured paper or hover constraints.";
    }

    return out;
}

} // namespace davinci_arm::services
