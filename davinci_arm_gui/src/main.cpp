#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "davinci_arm_gui/app/app_context.hpp"
#include "davinci_arm_gui/app/ros_qt_bridge.hpp"
#include "davinci_arm_gui/core/models/telemetry_sample.hpp"
#include "davinci_arm_gui/core/models/theme_id.hpp"
#include "davinci_arm_gui/ui/style/theme_manager.hpp"

int main(int argc, char* argv[]) {

    // 1. Qt first (safe argv ownership & GUI prerequisites):+
    QApplication app(argc, argv);
    qRegisterMetaType<davinci_arm::models::TelemetrySample>("davinci_arm::models::TelemetrySample");

    // 2. ROS after Qt:
    rclcpp::init(argc, argv);

    // 3. Theme:
    davinci_arm::ui::style::ThemeManager::instance()
    .apply(davinci_arm::models::ThemeId::TronAres);

    // 4. App context:
    davinci_arm::app::AppContext context;
    context.mainWindow().show();

    // 5. Bridge ROS <-> Qt lifetime:
    davinci_arm::app::RosQtBridge bridge(app, context.node());

    // 6. Qt loop:
    return app.exec();
}
