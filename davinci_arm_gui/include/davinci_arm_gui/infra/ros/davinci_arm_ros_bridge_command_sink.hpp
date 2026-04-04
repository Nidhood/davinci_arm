#pragma once

#include "davinci_arm_gui/core/services/calibration_service.hpp"
#include "davinci_arm_gui/core/models/domain.hpp"

#include <cstdint>

namespace davinci_arm::infra::ros {

class DavinciArmRosBridge;

class DavinciArmRosBridgeCommandSink final : public davinci_arm::services::ICalibrationCommandSink {
public:
    explicit DavinciArmRosBridgeCommandSink(DavinciArmRosBridge* bridge) noexcept;

    void sendPwmUs(davinci_arm::models::Domain domain, std::uint16_t pwm_us) override;
    void sendRefAngleRad(davinci_arm::models::Domain domain, double ref_angle_rad) override;
    void sendAutoMode(davinci_arm::models::Domain domain, bool enabled) override;
    void sendStop(davinci_arm::models::Domain domain) override;

private:
    DavinciArmRosBridge* bridge_{nullptr};
};

}  // namespace davinci_arm::infra::ros
