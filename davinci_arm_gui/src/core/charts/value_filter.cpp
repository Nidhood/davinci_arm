#include "davinci_arm_gui/core/charts/value_filter.hpp"

namespace davinci_arm::core::charts {

double ValueFilter::filter(davinci_arm::models::Domain d, double y, double dt) {
    if (!std::isfinite(y)) return y;
    if (!(tau_ > 0.0) || !(dt > 0.0) || !std::isfinite(dt)) return y;

    auto& st = (d == davinci_arm::models::Domain::Real) ? real_ : sim_;

    if (!st.has) {
        st.has = true;
        st.y = y;
        return y;
    }

    const double a = 1.0 - std::exp(-dt / tau_);
    st.y = st.y + a * (y - st.y);
    return st.y;
}

void ValueFilter::reset() {
    real_ = {};
    sim_ = {};
}

} // namespace davinci_arm::core::charts
