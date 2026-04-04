#pragma once

#include "davinci_arm_gui/core/models/drawing_types.hpp"

#include <optional>
#include <string>

namespace davinci_arm::services {

class IDrawingPlannerClient {
public:
    virtual ~IDrawingPlannerClient() = default;

    [[nodiscard]] virtual bool ping() = 0;
    [[nodiscard]] virtual davinci_arm::models::DrawingPlanResult preview(const davinci_arm::models::DrawingPlanRequest& request) = 0;
    [[nodiscard]] virtual davinci_arm::models::DrawingPlanResult plan(const davinci_arm::models::DrawingPlanRequest& request) = 0;
    [[nodiscard]] virtual bool sendPlannedJob(const std::string& external_job_id) = 0;
    virtual void cancelActiveRequest() = 0;
    [[nodiscard]] virtual std::optional<std::string> lastError() const noexcept = 0;
};

} // namespace davinci_arm::services
