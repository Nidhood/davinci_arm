#pragma once

#include "davinci_arm_gui/core/models/domain.hpp"

#include <cstdint>

namespace davinci_arm::services {

class ICalibrationCommandSink {
public:
    virtual ~ICalibrationCommandSink() = default;

    virtual void sendPwmUs(davinci_arm::models::Domain domain, std::uint16_t pwm_us) = 0;
    virtual void sendRefAngleRad(davinci_arm::models::Domain domain, double ref_angle_rad) = 0;
    virtual void sendAutoMode(davinci_arm::models::Domain domain, bool enabled) = 0;
    virtual void sendStop(davinci_arm::models::Domain domain) = 0;
};

} // namespace davinci_arm::services

namespace davinci_arm::infra::ros {
using ICalibrationCommandSink = davinci_arm::services::ICalibrationCommandSink;
} // namespace davinci_arm::infra::ros
