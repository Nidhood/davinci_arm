#pragma once

#include <cmath>
#include "davinci_arm_gui/core/models/domain.hpp"

namespace davinci_arm::core::charts {

class ValueFilter {
public:
    void setTau(double seconds) {
        tau_ = seconds;
    }
    double filter(davinci_arm::models::Domain d, double y, double dt);
    void reset();

private:
    double tau_{0.035};

    struct State {
        bool has{false};
        double y{0.0};
    };

    State real_{};
    State sim_{};
};

} // namespace davinci_arm::core::charts
