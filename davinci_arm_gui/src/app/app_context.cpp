#include "davinci_arm_gui/app/app_context.hpp"

#include <QObject>
#include <cstdlib>
#include <vector>

namespace davinci_arm::app {

namespace {

std::filesystem::path envPathOrEmpty(const char* key)
{
    const char* v = std::getenv(key);
    if (!v || !*v) {
        return {};
    }
    return std::filesystem::path(v);
}

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

std::filesystem::path AppContext::resolveWorkspacePath_()
{
    auto ws = envPathOrEmpty("DAVINCI_ARM_WS");
    if (!ws.empty()) {
        return ws;
    }

    const auto home = envPathOrEmpty("HOME");
    if (!home.empty()) {
        return home / "davinci_arm_ws";
    }

    return std::filesystem::current_path();
}

AppContext::AppContext()
    : node_(std::make_shared<rclcpp::Node>("davinci_arm_gui_node")),
      topic_registry_(std::make_shared<davinci_arm::infra::ros::TopicRegistry>(*node_)),
      limits_registry_(std::make_unique<davinci_arm::infra::ros::LimitsRegistry>(*node_)),
      telemetry_(5000),
      recorder_(),
      main_window_(nullptr),
      workspace_path_(resolveWorkspacePath_())
{
    ros_bridge_ = std::make_unique<davinci_arm::infra::ros::DavinciArmRosBridge>(
                      node_,
                      topic_registry_,
                      &main_window_);

    main_window_.setTelemetryStore(&telemetry_);
    main_window_.setLimitsRegistry(limits_registry_.get());
    main_window_.setRecorderService(&recorder_);

    urdf_updater_ = std::make_unique<davinci_arm::services::UrdfUpdater>(workspace_path_);
    calib_sink_ = std::make_unique<davinci_arm::infra::ros::DavinciArmRosBridgeCommandSink>(ros_bridge_.get());

    calibration_service_ = std::make_unique<davinci_arm::services::CalibrationService>(&main_window_);
    calibration_service_->setTelemetryStore(&telemetry_);
    calibration_service_->setCommandSink(calib_sink_.get());
    calibration_service_->setUrdfUpdater(urdf_updater_.get());

    main_window_.setCalibrationService(calibration_service_.get());

    QObject::connect(
        ros_bridge_.get(),
        &davinci_arm::infra::ros::DavinciArmRosBridge::telemetryUpdated,
        &main_window_,
    [this](davinci_arm::models::TelemetrySample sample) {
        telemetry_.push(sample);
        recorder_.onSample(sample);
        main_window_.onTelemetry(sample);
    },
    Qt::QueuedConnection);

    QObject::connect(
        ros_bridge_.get(),
        &davinci_arm::infra::ros::DavinciArmRosBridge::connectionChanged,
        &main_window_,
        &davinci_arm::app::MainWindow::setConnected,
        Qt::QueuedConnection);

    QObject::connect(
        ros_bridge_.get(),
        &davinci_arm::infra::ros::DavinciArmRosBridge::streamLiveChanged,
        &main_window_,
        &davinci_arm::app::MainWindow::setStreamLive,
        Qt::QueuedConnection);

    QObject::connect(
        &main_window_,
        &davinci_arm::app::MainWindow::refAngleChanged,
        ros_bridge_.get(),
        &davinci_arm::infra::ros::DavinciArmRosBridge::sendRefAngle,
        Qt::QueuedConnection);

    QObject::connect(
        &main_window_,
        &davinci_arm::app::MainWindow::stopRequested,
        ros_bridge_.get(),
        &davinci_arm::infra::ros::DavinciArmRosBridge::sendStop,
        Qt::QueuedConnection);

    QObject::connect(
        &main_window_,
        &davinci_arm::app::MainWindow::jointBatchCommandRequested,
        ros_bridge_.get(),
    [this](const QVector<double>& joints_deg) {
        std::vector<double> joints_rad;
        joints_rad.reserve(static_cast<std::size_t>(joints_deg.size()));

        for (const double deg_ui : joints_deg) {
            joints_rad.push_back((deg_ui - 180.0) * kDegToRad);
        }
        ros_bridge_->sendJointTrajectory(davinci_arm::models::Domain::Sim, joints_rad);
        ros_bridge_->sendJointTrajectory(davinci_arm::models::Domain::Real, joints_rad);
    },
    Qt::QueuedConnection);
}

AppContext::~AppContext()
{
    if (calibration_service_) {
        calibration_service_->stopCalibration();
    }

    ros_bridge_.reset();
    calibration_service_.reset();
    calib_sink_.reset();
    urdf_updater_.reset();
    limits_registry_.reset();
    topic_registry_.reset();
    node_.reset();
}

MainWindow& AppContext::mainWindow() noexcept
{
    return main_window_;
}

rclcpp::Node::SharedPtr AppContext::node() const noexcept
{
    return node_;
}

}  // namespace davinci_arm::app