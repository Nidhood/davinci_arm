#pragma once

#include "davinci_arm_gui/core/services/calibration_service.hpp" // ICalibrationCommandSink
#include "davinci_arm_gui/core/models/domain.hpp"

#include <cstdint>

namespace prop_arm::infra::ros {

class DavinciArmRosBridge;

class DavinciArmRosBridgeCommandSink final : public prop_arm::services::ICalibrationCommandSink {
public:
    explicit DavinciArmRosBridgeCommandSink(DavinciArmRosBridge* bridge) noexcept;

    void sendPwmUs(prop_arm::models::Domain domain, std::uint16_t pwm_us) override;
    void sendRefAngleRad(prop_arm::models::Domain domain, double ref_angle_rad) override;
    void sendAutoMode(prop_arm::models::Domain domain, bool enabled) override;
    void sendStop(prop_arm::models::Domain domain) override;

private:
    DavinciArmRosBridge* bridge_{nullptr}; // non-owning
};

}  // namespace prop_arm::infra::ros
