#pragma once

namespace davinci_arm::models {

enum class TelemetrySignalType {
    Angle,
    MotorSpeed,
    PwmFeedback,
    AngleRef
};

} // namespace davinci_arm::models