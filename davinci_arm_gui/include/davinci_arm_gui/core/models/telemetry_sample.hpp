#pragma once

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/telemetry_signal_type.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace davinci_arm::models {

struct TelemetrySample {
    Domain domain{Domain::Real};

    // Required by the new GUI routing/filtering
    TelemetrySignalType signal{TelemetrySignalType::Angle};
    std::string joint_name{};

    // Timestamp used by plots
    std::chrono::steady_clock::time_point t{};

    // Payload
    double arm_angle_rad{0.0};
    double motor_speed_rad_s{0.0};
    double ref_angle_rad{0.0};
    std::uint16_t pwm_us{0};

    bool valid{false};

    bool operator==(const TelemetrySample&) const noexcept = default;
};

} // namespace davinci_arm::models