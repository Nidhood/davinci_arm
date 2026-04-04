#pragma once

#include <deque>
#include <optional>
#include <QPointF>

namespace davinci_arm::core::charts {

class HoverInterpolator {
public:
    static std::optional<QPointF> yAt(const std::deque<QPointF>& buf, double t);
};

} // namespace davinci_arm::core::charts
