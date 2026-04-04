#pragma once

#include <optional>
#include <vector>

#include "davinci_arm_gui/core/models/telemetry_sample.hpp"

namespace davinci_arm::models {

struct DomainBuffer {
    std::optional<TelemetrySample> latest;
    std::vector<TelemetrySample> history;
};

} // namespace davinci_arm::models