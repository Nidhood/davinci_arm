#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace davinci_arm::models {

enum class CalibrationType : std::uint8_t {
    MotorVelocity,
    ArmPhysics,
    DrawingWorkspace
};

enum class CalibrationStatus : std::uint8_t {
    Idle,
    Running,
    Analyzing,
    Completed,
    Failed,
    Cancelled
};

struct MotorVelocityParams {
    double Kw{1.4016};
    double tau_w{0.564};
    double L_w{0.294};
    double motor_cmd_scale{1.162};

    bool operator==(const MotorVelocityParams&) const noexcept = default;
};

struct ArmPhysicsParams {
    double mass{0.4067};
    double inertia_yy{0.01086};
    double damping{0.0326233};
    double friction{0.0};

    bool operator==(const ArmPhysicsParams&) const noexcept = default;
};

struct CalibrationMetrics {
    double rmse{0.0};
    double max_error{0.0};
    double mean_error{0.0};
    double correlation{0.0};
    std::size_t sample_count{0};

    bool operator==(const CalibrationMetrics&) const noexcept = default;
};

struct CartesianPose {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
    bool valid{false};

    bool operator==(const CartesianPose&) const noexcept = default;
};

struct PaperCalibrationConfig {
    double paper_width_m{0.297};
    double paper_height_m{0.210};
    double margin_left_m{0.010};
    double margin_right_m{0.010};
    double margin_top_m{0.010};
    double margin_bottom_m{0.010};
    double desired_draw_height_offset_m{0.000};
    double desired_hover_height_m{0.010};
    double max_rectangularity_error_deg{5.0};
    double max_size_rel_error{0.15};
    bool auto_apply_to_drawing{true};

    bool operator==(const PaperCalibrationConfig&) const noexcept = default;
};

struct DrawingWorkspaceCalibration {
    CartesianPose origin_pose;
    CartesianPose x_axis_pose;
    CartesianPose y_axis_pose;
    CartesianPose hover_pose;

    double measured_width_m{0.0};
    double measured_height_m{0.0};
    double drawable_width_m{0.0};
    double drawable_height_m{0.0};
    double hover_height_m{0.0};
    double draw_height_m{0.0};
    double rectangularity_error_deg{0.0};
    double width_rel_error{0.0};
    double height_rel_error{0.0};

    std::array<double, 3> ex{{1.0, 0.0, 0.0}};
    std::array<double, 3> ey{{0.0, 1.0, 0.0}};
    std::array<double, 3> ez{{0.0, 0.0, 1.0}};

    bool valid{false};
    std::string notes;

    bool operator==(const DrawingWorkspaceCalibration&) const noexcept = default;
};

struct CalibrationResult {
    CalibrationStatus status{CalibrationStatus::Idle};
    CalibrationType type{CalibrationType::MotorVelocity};
    CalibrationMetrics metrics;
    MotorVelocityParams motor_params;
    ArmPhysicsParams physics_params;
    DrawingWorkspaceCalibration drawing_workspace;
    std::string error_message;

    bool operator==(const CalibrationResult&) const noexcept = default;
};

struct CalibrationConfig {
    CalibrationType type{CalibrationType::MotorVelocity};
    std::size_t selected_joint_index{0};
    double duration_sec{30.0};
    double settling_time_sec{2.0};
    std::vector<double> test_inputs;
    bool auto_apply{false};
    bool use_real{true};
    bool use_sim{true};
    double convergence_threshold{0.01};
    std::size_t max_iterations{20};
    PaperCalibrationConfig paper_config{};

    bool operator==(const CalibrationConfig&) const noexcept = default;
};

} // namespace davinci_arm::models
