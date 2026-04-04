#pragma once

#include <QColor>
#include <QSize>
#include <QSvgRenderer>
#include <QWidget>

#include <memory>

#include "davinci_arm_gui/ui/style/theme.hpp"

namespace davinci_arm::ui::widgets {

class ArmVisualizer final : public QWidget {
    Q_OBJECT

public:
    explicit ArmVisualizer(QWidget* parent = nullptr);

    void setRefAngle(double rad);
    void setRealAngle(double rad);
    void setSimAngle(double rad);

    void setShowRef(bool show);
    void setShowReal(bool show);
    void setShowSim(bool show);

    void applyTheme(const davinci_arm::ui::style::ThemeSpec& spec);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawBase_(QPainter& painter, const QPointF& pivot);
    void drawArm_(QPainter& painter,
                  const QPointF& pivot,
                  double angle_rad,
                  const QColor& color,
                  bool shadow,
                  double opacity);
    bool loadSvgModels_();

private:
    std::unique_ptr<QSvgRenderer> arm_renderer_;
    std::unique_ptr<QSvgRenderer> base_renderer_;
    QSize arm_svg_size_;
    QSize base_svg_size_;
    double ref_angle_{0.0};
    double real_angle_{0.0};
    double sim_angle_{0.0};
    bool show_ref_{false};
    bool show_real_{false};
    bool show_sim_{false};
    QColor ref_color_{100, 255, 100};
    QColor real_color_{255, 100, 100};
    QColor sim_color_{100, 150, 255};
    QColor base_color_{150, 150, 150};
    QColor bg_color_{30, 30, 35};
    double base_width_{60.0};
    double base_height_{40.0};
    double arm_length_{120.0};
    double arm_width_{25.0};
    double arm_scale_factor_{1.0};
    double base_scale_factor_{1.0};
    double pivot_y_offset_{0.6};
    double arm_center_x_{0.0};
    double arm_center_y_{0.5};
    double base_center_x_{0.5};
    double base_center_y_{0.5};
    double arm_angle_offset_{0.0};
    double ref_opacity_{1.0};
    double real_opacity_{0.85};
    double sim_opacity_{0.85};
};

}  // namespace davinci_arm::ui::widgets
