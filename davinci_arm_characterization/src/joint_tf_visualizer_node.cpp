#include "davinci_arm_characterization/joint_tf_visualizer.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<davinci_arm_characterization::JointTfVisualizer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
