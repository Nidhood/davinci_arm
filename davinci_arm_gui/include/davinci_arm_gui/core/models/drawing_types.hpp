#pragma once

#include "davinci_arm_gui/core/models/calibration_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace davinci_arm::models {

enum class DrawingExecutionState : std::uint8_t {
    Idle,
    Ready,
    Previewing,
    Planning,
    Planned,
    Sending,
    Completed,
    Failed,
    Cancelled
};

struct StrokePoint2D {
    double x{0.0};
    double y{0.0};
    bool pen_down{true};

    bool operator==(const StrokePoint2D&) const noexcept = default;
};

struct DrawingPath2D {
    std::string name;
    std::vector<StrokePoint2D> points;
    bool closed{false};

    bool operator==(const DrawingPath2D&) const noexcept = default;
};

struct DrawingDocument {
    std::string source_name;
    double width_units{0.0};
    double height_units{0.0};
    std::vector<DrawingPath2D> paths;

    [[nodiscard]] bool empty() const noexcept {
        return paths.empty();
    }

    bool operator==(const DrawingDocument&) const noexcept = default;
};

struct DrawingPlannerConfig {
    std::string planning_group{"arm"};
    std::string end_effector_link{"tool0"};
    std::string base_frame{"base_link"};
    std::string workspace_frame{"paper_frame"};
    double sample_step_m{0.0025};
    double velocity_scale{0.25};
    double acceleration_scale{0.20};
    bool preview_only{false};
    bool collision_check{true};
    bool execute_after_plan{false};

    bool operator==(const DrawingPlannerConfig&) const noexcept = default;
};

struct Waypoint3D {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
    bool pen_down{true};

    bool operator==(const Waypoint3D&) const noexcept = default;
};

struct DrawingPlanRequest {
    DrawingDocument document;
    DrawingWorkspaceCalibration workspace;
    DrawingPlannerConfig planner;

    bool operator==(const DrawingPlanRequest&) const noexcept = default;
};

struct DrawingPlanResult {
    bool success{false};
    std::string message;
    std::string external_job_id;
    double estimated_duration_sec{0.0};
    std::vector<Waypoint3D> preview_waypoints;

    bool operator==(const DrawingPlanResult&) const noexcept = default;
};

} // namespace davinci_arm::models
