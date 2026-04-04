#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace davinci_arm_characterization
{

class JointTfVisualizer final : public rclcpp::Node
{
public:
    explicit JointTfVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    struct Color final
    {
        float r {1.0F};
        float g {1.0F};
        float b {1.0F};
        float a {1.0F};
    };

    struct VisualOptions final
    {
        std::string fixed_frame {"world"};
        std::string marker_topic {"/davinci_arm/joint_tf_visualization"};
        std::string robot_description_node {"robot_state_publisher"};

        double update_rate_hz {10.0};
        double connection_width {0.007};
        double axis_length {0.09};
        double axis_shaft_diameter {0.010};
        double axis_head_diameter {0.018};
        double axis_head_length {0.020};
        double revolute_radius {0.045};
        double revolute_arrow_length {0.020};
        double prismatic_half_length {0.050};
        double text_size {0.038};
        double text_lift {0.035};

        int revolute_samples {40};

        bool show_fixed_joints {false};
        bool show_joint_names {true};
        bool show_joint_types {true};
        bool show_distances {true};
        bool show_limits {false};
    };

    struct JointDescriptor final
    {
        std::string name;
        std::string type;
        std::string parent_link;
        std::string child_link;

        tf2::Vector3 axis {1.0, 0.0, 0.0};
        tf2::Vector3 origin_xyz {0.0, 0.0, 0.0};
        tf2::Quaternion origin_quaternion {0.0, 0.0, 0.0, 1.0};

        bool has_limits {false};
        double lower_limit {0.0};
        double upper_limit {0.0};
    };

    struct JointColors final
    {
        Color connection;
        Color axis;
        Color accent;
        Color text;
    };

    void declareParameters();
    void loadParameters();

    void timerCallback();
    void requestRobotDescription();
    void handleRobotDescription(
        const std::shared_future<std::vector<rclcpp::Parameter>> & future);

    bool parseRobotDescription(const std::string & robot_description_xml);
    void collectJointsRecursive(
        const urdf::LinkConstSharedPtr & link,
        std::vector<JointDescriptor> & ordered_joints) const;

    static std::string jointTypeToString(int joint_type);
    static bool isFixedJoint(const JointDescriptor & joint);
    static bool isRevoluteJoint(const JointDescriptor & joint);
    static bool isPrismaticJoint(const JointDescriptor & joint);

    [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
        const std::string & target_frame,
        const std::string & source_frame) const;

    [[nodiscard]] tf2::Transform toTf2(const geometry_msgs::msg::Transform & transform_msg) const;
    [[nodiscard]] geometry_msgs::msg::Point toPoint(const tf2::Vector3 & vector) const;
    [[nodiscard]] tf2::Vector3 makePerpendicularUnit(const tf2::Vector3 & axis) const;
    [[nodiscard]] tf2::Vector3 computeTextOffset(
        const tf2::Vector3 & parent,
        const tf2::Vector3 & child,
        const tf2::Vector3 & axis) const;

    [[nodiscard]] JointColors colorsForJoint(const JointDescriptor & joint) const;
    [[nodiscard]] std::string makeJointLabel(
        const JointDescriptor & joint,
        double distance_meters) const;

    void appendDeleteAllMarker(visualization_msgs::msg::MarkerArray & marker_array) const;
    void appendConnectionMarker(
        visualization_msgs::msg::MarkerArray & marker_array,
        std::int32_t & marker_id,
        const JointDescriptor & joint,
        const tf2::Vector3 & parent_position,
        const tf2::Vector3 & child_position,
        const JointColors & colors) const;
    void appendAxisMarker(
        visualization_msgs::msg::MarkerArray & marker_array,
        std::int32_t & marker_id,
        const JointDescriptor & joint,
        const tf2::Vector3 & joint_origin,
        const tf2::Vector3 & joint_axis_world,
        const JointColors & colors) const;
    void appendLabelMarker(
        visualization_msgs::msg::MarkerArray & marker_array,
        std::int32_t & marker_id,
        const JointDescriptor & joint,
        const tf2::Vector3 & parent_position,
        const tf2::Vector3 & child_position,
        const tf2::Vector3 & joint_axis_world,
        double distance_meters,
        const JointColors & colors) const;
    void appendRevoluteMarkers(
        visualization_msgs::msg::MarkerArray & marker_array,
        std::int32_t & marker_id,
        const JointDescriptor & joint,
        const tf2::Vector3 & joint_origin,
        const tf2::Vector3 & joint_axis_world,
        const JointColors & colors) const;
    void appendPrismaticMarkers(
        visualization_msgs::msg::MarkerArray & marker_array,
        std::int32_t & marker_id,
        const JointDescriptor & joint,
        const tf2::Vector3 & joint_origin,
        const tf2::Vector3 & joint_axis_world,
        const JointColors & colors) const;

    VisualOptions options_ {};

    urdf::Model urdf_model_ {};
    std::vector<JointDescriptor> ordered_joints_ {};

    bool robot_description_loaded_ {false};
    bool parameter_request_in_flight_ {false};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_ {};
    std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_ {};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {};
    rclcpp::TimerBase::SharedPtr timer_ {};
};

}  // namespace davinci_arm_characterization
