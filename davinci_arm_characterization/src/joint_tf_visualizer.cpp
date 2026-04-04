#include "davinci_arm_characterization/joint_tf_visualizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace davinci_arm_characterization
{

namespace
{
constexpr double kVectorEpsilon = 1.0e-9;
constexpr double kLookupTimeoutSeconds = 0.05;

using Segment2D = std::array<double, 4>;

[[nodiscard]] tf2::Vector3 normalizeOrFallback(
    const tf2::Vector3 & vector,
    const tf2::Vector3 & fallback)
{
    if (vector.length2() < kVectorEpsilon) {
        return fallback;
    }

    tf2::Vector3 normalized = vector;
    normalized.normalize();
    return normalized;
}

[[nodiscard]] std::string formatDouble(const double value, const int precision = 4)
{
    std::ostringstream stream;
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(precision);
    stream << value;
    return stream.str();
}

[[nodiscard]] tf2::Quaternion safeNormalize(const tf2::Quaternion & quaternion)
{
    if (quaternion.length2() < kVectorEpsilon) {
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }

    tf2::Quaternion normalized = quaternion;
    normalized.normalize();
    return normalized;
}

[[nodiscard]] geometry_msgs::msg::Point makePoint(const tf2::Vector3 & vector)
{
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

[[nodiscard]] const std::vector<Segment2D> & glyphSegments(const char character)
{
    static const std::vector<Segment2D> glyph_0 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.60, 1.00, 0.60, 0.00},
        {0.60, 0.00, 0.10, 0.00},
        {0.10, 0.00, 0.10, 1.00},
    };

    static const std::vector<Segment2D> glyph_1 = {
        {0.35, 1.00, 0.35, 0.00},
        {0.20, 0.82, 0.35, 1.00},
        {0.20, 0.00, 0.50, 0.00},
    };

    static const std::vector<Segment2D> glyph_2 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.60, 1.00, 0.60, 0.55},
        {0.10, 0.55, 0.60, 0.55},
        {0.10, 0.55, 0.10, 0.00},
        {0.10, 0.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_3 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.60, 1.00, 0.60, 0.00},
        {0.10, 0.55, 0.60, 0.55},
        {0.10, 0.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_4 = {
        {0.10, 1.00, 0.10, 0.55},
        {0.10, 0.55, 0.60, 0.55},
        {0.60, 1.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_5 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.10, 1.00, 0.10, 0.55},
        {0.10, 0.55, 0.60, 0.55},
        {0.60, 0.55, 0.60, 0.00},
        {0.10, 0.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_6 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.10, 1.00, 0.10, 0.00},
        {0.10, 0.55, 0.60, 0.55},
        {0.60, 0.55, 0.60, 0.00},
        {0.10, 0.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_7 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.60, 1.00, 0.35, 0.00},
    };

    static const std::vector<Segment2D> glyph_8 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.60, 1.00, 0.60, 0.00},
        {0.60, 0.00, 0.10, 0.00},
        {0.10, 0.00, 0.10, 1.00},
        {0.10, 0.55, 0.60, 0.55},
    };

    static const std::vector<Segment2D> glyph_9 = {
        {0.10, 1.00, 0.60, 1.00},
        {0.10, 1.00, 0.10, 0.55},
        {0.10, 0.55, 0.60, 0.55},
        {0.60, 1.00, 0.60, 0.00},
        {0.10, 0.00, 0.60, 0.00},
    };

    static const std::vector<Segment2D> glyph_dot = {
        {0.05, 0.05, 0.15, 0.05},
        {0.10, 0.00, 0.10, 0.10},
    };

    static const std::vector<Segment2D> glyph_m = {
        {0.00, 0.00, 0.00, 0.78},
        {0.00, 0.78, 0.18, 0.58},
        {0.18, 0.58, 0.34, 0.78},
        {0.34, 0.78, 0.34, 0.00},
        {0.34, 0.78, 0.52, 0.58},
        {0.52, 0.58, 0.68, 0.78},
        {0.68, 0.78, 0.68, 0.00},
    };

    static const std::vector<Segment2D> empty_glyph = {};

    switch (character) {
    case '0':
        return glyph_0;
    case '1':
        return glyph_1;
    case '2':
        return glyph_2;
    case '3':
        return glyph_3;
    case '4':
        return glyph_4;
    case '5':
        return glyph_5;
    case '6':
        return glyph_6;
    case '7':
        return glyph_7;
    case '8':
        return glyph_8;
    case '9':
        return glyph_9;
    case '.':
        return glyph_dot;
    case 'm':
        return glyph_m;
    case ' ':
    default:
        return empty_glyph;
    }
}

[[nodiscard]] double glyphAdvance(const char character)
{
    switch (character) {
    case '1':
        return 0.45;
    case '.':
        return 0.22;
    case 'm':
        return 0.78;
    case ' ':
        return 0.30;
    default:
        return 0.72;
    }
}

void appendVectorTextToMarker(
    visualization_msgs::msg::Marker & marker,
    const std::string & text,
    const tf2::Vector3 & origin,
    const tf2::Vector3 & baseline_direction,
    const tf2::Vector3 & up_direction,
    const double glyph_height)
{
    const tf2::Vector3 baseline_unit = normalizeOrFallback(
                                           baseline_direction,
                                           tf2::Vector3(1.0, 0.0, 0.0));
    const tf2::Vector3 up_unit = normalizeOrFallback(
                                     up_direction,
                                     tf2::Vector3(0.0, 0.0, 1.0));

    constexpr double character_spacing = 0.18;
    double cursor = 0.0;

    for (const char character : text) {
        const auto & segments = glyphSegments(character);

        for (const auto & segment : segments) {
            const tf2::Vector3 start =
                origin +
                baseline_unit * ((cursor + segment[0]) * glyph_height) +
                up_unit * (segment[1] * glyph_height);

            const tf2::Vector3 end =
                origin +
                baseline_unit * ((cursor + segment[2]) * glyph_height) +
                up_unit * (segment[3] * glyph_height);

            marker.points.push_back(makePoint(start));
            marker.points.push_back(makePoint(end));
        }

        cursor += glyphAdvance(character) + character_spacing;
    }
}

}  // namespace

JointTfVisualizer::JointTfVisualizer(const rclcpp::NodeOptions & options)
    : rclcpp::Node("joint_tf_visualizer", options)
{
    declareParameters();
    loadParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                            options_.marker_topic,
                            rclcpp::QoS(10).transient_local());

    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
                            this,
                            options_.robot_description_node);

    const auto period = std::chrono::duration<double>(1.0 / std::max(options_.update_rate_hz, 1.0));
    timer_ = this->create_wall_timer(
                 std::chrono::duration_cast<std::chrono::milliseconds>(period),
                 std::bind(&JointTfVisualizer::timerCallback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "joint_tf_visualizer started | fixed_frame='%s' | marker_topic='%s' | "
        "robot_description_node='%s'",
        options_.fixed_frame.c_str(),
        options_.marker_topic.c_str(),
        options_.robot_description_node.c_str());
}

void JointTfVisualizer::declareParameters()
{
    this->declare_parameter<std::string>("fixed_frame", options_.fixed_frame);
    this->declare_parameter<std::string>("marker_topic", options_.marker_topic);
    this->declare_parameter<std::string>("robot_description_node", options_.robot_description_node);

    this->declare_parameter<double>("update_rate_hz", options_.update_rate_hz);
    this->declare_parameter<double>("connection_width", options_.connection_width);
    this->declare_parameter<double>("axis_length", options_.axis_length);
    this->declare_parameter<double>("axis_shaft_diameter", options_.axis_shaft_diameter);
    this->declare_parameter<double>("axis_head_diameter", options_.axis_head_diameter);
    this->declare_parameter<double>("axis_head_length", options_.axis_head_length);
    this->declare_parameter<double>("revolute_radius", options_.revolute_radius);
    this->declare_parameter<double>("revolute_arrow_length", options_.revolute_arrow_length);
    this->declare_parameter<double>("prismatic_half_length", options_.prismatic_half_length);
    this->declare_parameter<double>("text_size", options_.text_size);
    this->declare_parameter<double>("text_lift", options_.text_lift);

    this->declare_parameter<int>("revolute_samples", options_.revolute_samples);

    this->declare_parameter<bool>("show_fixed_joints", options_.show_fixed_joints);
    this->declare_parameter<bool>("show_joint_names", options_.show_joint_names);
    this->declare_parameter<bool>("show_joint_types", options_.show_joint_types);
    this->declare_parameter<bool>("show_distances", options_.show_distances);
    this->declare_parameter<bool>("show_limits", options_.show_limits);
}

void JointTfVisualizer::loadParameters()
{
    options_.fixed_frame = this->get_parameter("fixed_frame").as_string();
    options_.marker_topic = this->get_parameter("marker_topic").as_string();
    options_.robot_description_node = this->get_parameter("robot_description_node").as_string();

    options_.update_rate_hz = this->get_parameter("update_rate_hz").as_double();
    options_.connection_width = this->get_parameter("connection_width").as_double();
    options_.axis_length = this->get_parameter("axis_length").as_double();
    options_.axis_shaft_diameter = this->get_parameter("axis_shaft_diameter").as_double();
    options_.axis_head_diameter = this->get_parameter("axis_head_diameter").as_double();
    options_.axis_head_length = this->get_parameter("axis_head_length").as_double();
    options_.revolute_radius = this->get_parameter("revolute_radius").as_double();
    options_.revolute_arrow_length = this->get_parameter("revolute_arrow_length").as_double();
    options_.prismatic_half_length = this->get_parameter("prismatic_half_length").as_double();
    options_.text_size = this->get_parameter("text_size").as_double();
    options_.text_lift = this->get_parameter("text_lift").as_double();

    options_.revolute_samples = static_cast<int>(this->get_parameter("revolute_samples").as_int());

    options_.show_fixed_joints = this->get_parameter("show_fixed_joints").as_bool();
    options_.show_joint_names = this->get_parameter("show_joint_names").as_bool();
    options_.show_joint_types = this->get_parameter("show_joint_types").as_bool();
    options_.show_distances = this->get_parameter("show_distances").as_bool();
    options_.show_limits = this->get_parameter("show_limits").as_bool();
}

void JointTfVisualizer::timerCallback()
{
    if (!robot_description_loaded_) {
        requestRobotDescription();
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(1 + static_cast<std::size_t>(ordered_joints_.size()) * 12U);
    appendDeleteAllMarker(marker_array);

    std::int32_t marker_id = 0;

    for (const JointDescriptor & joint : ordered_joints_) {
        if (isFixedJoint(joint) && !options_.show_fixed_joints) {
            continue;
        }

        const auto parent_world = lookupTransform(options_.fixed_frame, joint.parent_link);
        const auto child_world = lookupTransform(options_.fixed_frame, joint.child_link);
        const auto parent_child = lookupTransform(joint.parent_link, joint.child_link);

        if (!parent_world.has_value() || !child_world.has_value() || !parent_child.has_value()) {
            continue;
        }

        const tf2::Transform tf_parent_world = toTf2(parent_world->transform);
        const tf2::Transform tf_child_world = toTf2(child_world->transform);
        const tf2::Transform tf_parent_child = toTf2(parent_child->transform);

        const tf2::Transform tf_parent_joint(
            safeNormalize(joint.origin_quaternion),
            joint.origin_xyz);

        const tf2::Transform tf_world_joint = tf_parent_world * tf_parent_joint;

        const tf2::Vector3 parent_position = tf_parent_world.getOrigin();
        const tf2::Vector3 child_position = tf_child_world.getOrigin();
        const tf2::Vector3 joint_origin = tf_world_joint.getOrigin();
        const tf2::Vector3 joint_axis_world = normalizeOrFallback(
                tf2::quatRotate(tf_world_joint.getRotation(), joint.axis),
                tf2::Vector3(0.0, 0.0, 1.0));
        const double distance_meters = tf_parent_child.getOrigin().length();

        const JointColors colors = colorsForJoint(joint);

        appendConnectionMarker(marker_array, marker_id, joint, parent_position, child_position, colors);
        appendAxisMarker(marker_array, marker_id, joint, joint_origin, joint_axis_world, colors);
        appendLabelMarker(
            marker_array,
            marker_id,
            joint,
            parent_position,
            child_position,
            joint_axis_world,
            distance_meters,
            colors);

        if (isRevoluteJoint(joint)) {
            appendRevoluteMarkers(marker_array, marker_id, joint, joint_origin, joint_axis_world, colors);
        } else if (isPrismaticJoint(joint)) {
            appendPrismaticMarkers(marker_array, marker_id, joint, joint_origin, joint_axis_world, colors);
        }
    }

    marker_publisher_->publish(marker_array);
}

void JointTfVisualizer::requestRobotDescription()
{
    if (parameter_request_in_flight_) {
        return;
    }

    if (!parameter_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Waiting for parameter service on node '%s'...",
            options_.robot_description_node.c_str());
        return;
    }

    parameter_request_in_flight_ = true;

    parameter_client_->get_parameters(
    {"robot_description"},
    std::bind(
        &JointTfVisualizer::handleRobotDescription,
        this,
        std::placeholders::_1));
}

void JointTfVisualizer::handleRobotDescription(
    const std::shared_future<std::vector<rclcpp::Parameter>> & future)
{
    parameter_request_in_flight_ = false;

    try {
        const auto parameters = future.get();
        if (parameters.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' was not returned.");
            return;
        }

        const std::string robot_description_xml = parameters.front().as_string();
        if (robot_description_xml.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is empty.");
            return;
        }

        if (!parseRobotDescription(robot_description_xml)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse the robot_description URDF.");
            return;
        }

        robot_description_loaded_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu joints from runtime robot_description.",
            ordered_joints_.size());
    } catch (const std::exception & exception) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not retrieve robot_description: %s",
            exception.what());
    }
}

bool JointTfVisualizer::parseRobotDescription(const std::string & robot_description_xml)
{
    urdf_model_ = urdf::Model();
    ordered_joints_.clear();

    if (!urdf_model_.initString(robot_description_xml)) {
        return false;
    }

    const urdf::LinkConstSharedPtr root_link = urdf_model_.getRoot();
    if (!root_link) {
        RCLCPP_ERROR(this->get_logger(), "URDF root link is null.");
        return false;
    }

    collectJointsRecursive(root_link, ordered_joints_);
    return !ordered_joints_.empty();
}

void JointTfVisualizer::collectJointsRecursive(
    const urdf::LinkConstSharedPtr & link,
    std::vector<JointDescriptor> & ordered_joints) const
{
    if (!link) {
        return;
    }

    for (const urdf::JointSharedPtr & joint : link->child_joints) {
        if (!joint) {
            continue;
        }

        JointDescriptor descriptor;
        descriptor.name = joint->name;
        descriptor.type = jointTypeToString(joint->type);
        descriptor.parent_link = joint->parent_link_name;
        descriptor.child_link = joint->child_link_name;

        descriptor.axis = tf2::Vector3(joint->axis.x, joint->axis.y, joint->axis.z);
        descriptor.axis = normalizeOrFallback(descriptor.axis, tf2::Vector3(1.0, 0.0, 0.0));

        descriptor.origin_xyz = tf2::Vector3(
                                    joint->parent_to_joint_origin_transform.position.x,
                                    joint->parent_to_joint_origin_transform.position.y,
                                    joint->parent_to_joint_origin_transform.position.z);

        descriptor.origin_quaternion = safeNormalize(
                                           tf2::Quaternion(
                                               joint->parent_to_joint_origin_transform.rotation.x,
                                               joint->parent_to_joint_origin_transform.rotation.y,
                                               joint->parent_to_joint_origin_transform.rotation.z,
                                               joint->parent_to_joint_origin_transform.rotation.w));

        if (joint->limits) {
            descriptor.has_limits = true;
            descriptor.lower_limit = joint->limits->lower;
            descriptor.upper_limit = joint->limits->upper;
        }

        ordered_joints.push_back(std::move(descriptor));

        const urdf::LinkConstSharedPtr child_link = urdf_model_.getLink(joint->child_link_name);
        collectJointsRecursive(child_link, ordered_joints);
    }
}

std::string JointTfVisualizer::jointTypeToString(const int joint_type)
{
    switch (joint_type) {
    case urdf::Joint::REVOLUTE:
        return "revolute";
    case urdf::Joint::CONTINUOUS:
        return "continuous";
    case urdf::Joint::PRISMATIC:
        return "prismatic";
    case urdf::Joint::FLOATING:
        return "floating";
    case urdf::Joint::PLANAR:
        return "planar";
    case urdf::Joint::FIXED:
    default:
        return "fixed";
    }
}

bool JointTfVisualizer::isFixedJoint(const JointDescriptor & joint)
{
    return joint.type == "fixed";
}

bool JointTfVisualizer::isRevoluteJoint(const JointDescriptor & joint)
{
    return joint.type == "revolute" || joint.type == "continuous";
}

bool JointTfVisualizer::isPrismaticJoint(const JointDescriptor & joint)
{
    return joint.type == "prismatic";
}

std::optional<geometry_msgs::msg::TransformStamped> JointTfVisualizer::lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame) const
{
    try {
        return tf_buffer_->lookupTransform(
                   target_frame,
                   source_frame,
                   tf2::TimePointZero,
                   tf2::durationFromSec(kLookupTimeoutSeconds));
    } catch (const tf2::TransformException & exception) {
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "TF lookup failed: %s <- %s | %s",
            target_frame.c_str(),
            source_frame.c_str(),
            exception.what());
        return std::nullopt;
    }
}

tf2::Transform JointTfVisualizer::toTf2(const geometry_msgs::msg::Transform & transform_msg) const
{
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
                            transform_msg.translation.x,
                            transform_msg.translation.y,
                            transform_msg.translation.z));
    transform.setRotation(safeNormalize(tf2::Quaternion(
                                            transform_msg.rotation.x,
                                            transform_msg.rotation.y,
                                            transform_msg.rotation.z,
                                            transform_msg.rotation.w)));
    return transform;
}

geometry_msgs::msg::Point JointTfVisualizer::toPoint(const tf2::Vector3 & vector) const
{
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

tf2::Vector3 JointTfVisualizer::makePerpendicularUnit(const tf2::Vector3 & axis) const
{
    const tf2::Vector3 axis_unit = normalizeOrFallback(axis, tf2::Vector3(0.0, 0.0, 1.0));

    const std::array<tf2::Vector3, 3> candidates = {
        tf2::Vector3(1.0, 0.0, 0.0),
        tf2::Vector3(0.0, 1.0, 0.0),
        tf2::Vector3(0.0, 0.0, 1.0)
    };

    for (const auto & candidate : candidates) {
        const tf2::Vector3 perpendicular = axis_unit.cross(candidate);
        if (perpendicular.length2() > kVectorEpsilon) {
            return normalizeOrFallback(perpendicular, tf2::Vector3(1.0, 0.0, 0.0));
        }
    }

    return tf2::Vector3(1.0, 0.0, 0.0);
}

tf2::Vector3 JointTfVisualizer::computeTextOffset(
    const tf2::Vector3 & parent,
    const tf2::Vector3 & child,
    const tf2::Vector3 & axis) const
{
    const tf2::Vector3 segment = child - parent;
    const tf2::Vector3 segment_unit = normalizeOrFallback(segment, tf2::Vector3(1.0, 0.0, 0.0));

    tf2::Vector3 lateral = segment_unit.cross(axis);
    if (lateral.length2() < kVectorEpsilon) {
        lateral = axis.cross(tf2::Vector3(0.0, 0.0, 1.0));
    }
    if (lateral.length2() < kVectorEpsilon) {
        lateral = axis.cross(tf2::Vector3(0.0, 1.0, 0.0));
    }

    lateral = normalizeOrFallback(lateral, tf2::Vector3(1.0, 0.0, 0.0));
    return (lateral * (options_.revolute_radius * 0.55)) +
           tf2::Vector3(0.0, 0.0, options_.text_lift * 0.65);
}

JointTfVisualizer::JointColors JointTfVisualizer::colorsForJoint(const JointDescriptor & joint) const
{
    JointColors colors;

    if (isRevoluteJoint(joint)) {
        colors.connection = {0.55F, 1.00F, 0.15F, 0.98F};
        colors.axis = {0.10F, 0.92F, 1.00F, 1.00F};
        colors.accent = {1.00F, 0.12F, 0.86F, 1.00F};
        colors.text = {1.00F, 0.82F, 0.20F, 1.00F};
        return colors;
    }

    if (isPrismaticJoint(joint)) {
        colors.connection = {0.25F, 1.00F, 0.35F, 0.96F};
        colors.axis = {0.10F, 0.92F, 1.00F, 1.00F};
        colors.accent = {0.92F, 0.24F, 1.00F, 1.00F};
        colors.text = {1.00F, 0.86F, 0.35F, 1.00F};
        return colors;
    }

    colors.connection = {0.60F, 0.70F, 0.68F, 0.72F};
    colors.axis = {0.72F, 0.92F, 0.98F, 0.84F};
    colors.accent = {0.86F, 0.86F, 0.90F, 0.82F};
    colors.text = {0.96F, 0.92F, 0.84F, 0.92F};
    return colors;
}

std::string JointTfVisualizer::makeJointLabel(
    const JointDescriptor & joint,
    const double distance_meters) const
{
    std::ostringstream stream;

    if (options_.show_joint_names) {
        stream << joint.name;
    }

    if (options_.show_joint_types) {
        if (!stream.str().empty()) {
            stream << '\n';
        }
        stream << joint.type;
    }

    if (options_.show_distances) {
        if (!stream.str().empty()) {
            stream << '\n';
        }
        stream << formatDouble(distance_meters, 4) << " m";
    }

    if (options_.show_limits && joint.has_limits) {
        if (!stream.str().empty()) {
            stream << '\n';
        }

        const bool angle_like = isRevoluteJoint(joint);
        stream << "[" << formatDouble(joint.lower_limit, 2) << ", "
               << formatDouble(joint.upper_limit, 2) << "] "
               << (angle_like ? "rad" : "m");
    }

    return stream.str();
}

void JointTfVisualizer::appendDeleteAllMarker(
    visualization_msgs::msg::MarkerArray & marker_array) const
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = options_.fixed_frame;
    marker.header.stamp = this->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(std::move(marker));
}

void JointTfVisualizer::appendConnectionMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    std::int32_t & marker_id,
    const JointDescriptor &,
    const tf2::Vector3 & parent_position,
    const tf2::Vector3 & child_position,
    const JointColors & colors) const
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = options_.fixed_frame;
    marker.header.stamp = this->now();
    marker.ns = "joint_connection";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = options_.connection_width;
    marker.color.r = colors.connection.r;
    marker.color.g = colors.connection.g;
    marker.color.b = colors.connection.b;
    marker.color.a = colors.connection.a;
    marker.pose.orientation.w = 1.0;
    marker.points.push_back(toPoint(parent_position));
    marker.points.push_back(toPoint(child_position));
    marker.frame_locked = false;
    marker_array.markers.push_back(std::move(marker));
}

void JointTfVisualizer::appendAxisMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    std::int32_t & marker_id,
    const JointDescriptor &,
    const tf2::Vector3 & joint_origin,
    const tf2::Vector3 & joint_axis_world,
    const JointColors & colors) const
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = options_.fixed_frame;
    marker.header.stamp = this->now();
    marker.ns = "joint_axis";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = options_.axis_shaft_diameter;
    marker.scale.y = options_.axis_head_diameter;
    marker.scale.z = options_.axis_head_length;
    marker.color.r = colors.axis.r;
    marker.color.g = colors.axis.g;
    marker.color.b = colors.axis.b;
    marker.color.a = colors.axis.a;
    marker.pose.orientation.w = 1.0;
    marker.points.push_back(toPoint(joint_origin));
    marker.points.push_back(toPoint(joint_origin + joint_axis_world * options_.axis_length));
    marker.frame_locked = false;
    marker_array.markers.push_back(std::move(marker));
}

void JointTfVisualizer::appendLabelMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    std::int32_t & marker_id,
    const JointDescriptor & joint,
    const tf2::Vector3 & parent_position,
    const tf2::Vector3 & child_position,
    const tf2::Vector3 & joint_axis_world,
    const double distance_meters,
    const JointColors & colors) const
{
    const tf2::Vector3 midpoint = (parent_position + child_position) * 0.5;
    const tf2::Vector3 world_up(0.0, 0.0, 1.0);

    tf2::Vector3 side_direction = computeTextOffset(parent_position, child_position, joint_axis_world);
    side_direction.setZ(0.0);

    if (side_direction.length2() < kVectorEpsilon) {
        side_direction = makePerpendicularUnit(child_position - parent_position);
        side_direction.setZ(0.0);
    }

    side_direction = normalizeOrFallback(side_direction, tf2::Vector3(1.0, 0.0, 0.0));

    // Force all distance labels to the +X side of the fixed frame.
    if (side_direction.x() < 0.0) {
        side_direction = -side_direction;
    }

    const tf2::Vector3 leader_anchor =
        midpoint +
        side_direction * (options_.revolute_radius * 0.78) +
        world_up * (options_.text_lift * 0.15);

    if (options_.show_distances) {
        const std::string distance_label = formatDouble(distance_meters, 4) + " m";
        const double glyph_height = options_.text_size * 0.72;
        const double gap = options_.text_size * 0.12;
        const double vector_text_line_width =
            std::max(options_.connection_width * 0.20, glyph_height * 0.035);

        const tf2::Vector3 distance_text_origin =
            leader_anchor +
            side_direction * gap -
            world_up * (glyph_height * 0.36);

        visualization_msgs::msg::Marker midpoint_marker;
        midpoint_marker.header.frame_id = options_.fixed_frame;
        midpoint_marker.header.stamp = this->now();
        midpoint_marker.ns = "joint_distance_midpoint";
        midpoint_marker.id = marker_id++;
        midpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        midpoint_marker.action = visualization_msgs::msg::Marker::ADD;
        midpoint_marker.pose.position = toPoint(midpoint);
        midpoint_marker.pose.orientation.w = 1.0;
        midpoint_marker.scale.x = options_.connection_width * 2.2;
        midpoint_marker.scale.y = options_.connection_width * 2.2;
        midpoint_marker.scale.z = options_.connection_width * 2.2;
        midpoint_marker.color.r = 1.0F;
        midpoint_marker.color.g = 1.0F;
        midpoint_marker.color.b = 1.0F;
        midpoint_marker.color.a = 0.95F;
        midpoint_marker.frame_locked = false;
        marker_array.markers.push_back(std::move(midpoint_marker));

        visualization_msgs::msg::Marker leader_marker;
        leader_marker.header.frame_id = options_.fixed_frame;
        leader_marker.header.stamp = this->now();
        leader_marker.ns = "joint_distance_leader";
        leader_marker.id = marker_id++;
        leader_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        leader_marker.action = visualization_msgs::msg::Marker::ADD;
        leader_marker.scale.x = options_.connection_width * 0.45;
        leader_marker.color.r = 0.95F;
        leader_marker.color.g = 0.95F;
        leader_marker.color.b = 0.95F;
        leader_marker.color.a = 0.72F;
        leader_marker.pose.orientation.w = 1.0;
        leader_marker.points.push_back(toPoint(midpoint));
        leader_marker.points.push_back(toPoint(leader_anchor));
        leader_marker.frame_locked = false;
        marker_array.markers.push_back(std::move(leader_marker));

        visualization_msgs::msg::Marker anchor_marker;
        anchor_marker.header.frame_id = options_.fixed_frame;
        anchor_marker.header.stamp = this->now();
        anchor_marker.ns = "joint_distance_anchor";
        anchor_marker.id = marker_id++;
        anchor_marker.type = visualization_msgs::msg::Marker::SPHERE;
        anchor_marker.action = visualization_msgs::msg::Marker::ADD;
        anchor_marker.pose.position = toPoint(leader_anchor);
        anchor_marker.pose.orientation.w = 1.0;
        anchor_marker.scale.x = options_.connection_width * 1.35;
        anchor_marker.scale.y = options_.connection_width * 1.35;
        anchor_marker.scale.z = options_.connection_width * 1.35;
        anchor_marker.color.r = 1.0F;
        anchor_marker.color.g = 1.0F;
        anchor_marker.color.b = 1.0F;
        anchor_marker.color.a = 0.95F;
        anchor_marker.frame_locked = false;
        marker_array.markers.push_back(std::move(anchor_marker));

        visualization_msgs::msg::Marker distance_vector_marker;
        distance_vector_marker.header.frame_id = options_.fixed_frame;
        distance_vector_marker.header.stamp = this->now();
        distance_vector_marker.ns = "joint_distance_vector_text";
        distance_vector_marker.id = marker_id++;
        distance_vector_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        distance_vector_marker.action = visualization_msgs::msg::Marker::ADD;
        distance_vector_marker.scale.x = vector_text_line_width;
        distance_vector_marker.color.r = 1.0F;
        distance_vector_marker.color.g = 1.0F;
        distance_vector_marker.color.b = 1.0F;
        distance_vector_marker.color.a = 0.98F;
        distance_vector_marker.pose.orientation.w = 1.0;
        distance_vector_marker.frame_locked = false;

        appendVectorTextToMarker(
            distance_vector_marker,
            distance_label,
            distance_text_origin,
            side_direction,
            world_up,
            glyph_height);

        marker_array.markers.push_back(std::move(distance_vector_marker));
    }

    std::ostringstream info_stream;
    if (options_.show_joint_names) {
        info_stream << joint.name;
    }

    if (options_.show_joint_types) {
        if (!info_stream.str().empty()) {
            info_stream << '\n';
        }
        info_stream << joint.type;
    }

    if (options_.show_limits && joint.has_limits) {
        if (!info_stream.str().empty()) {
            info_stream << '\n';
        }

        const bool angle_like = isRevoluteJoint(joint);
        info_stream << "[" << formatDouble(joint.lower_limit, 2) << ", "
                    << formatDouble(joint.upper_limit, 2) << "] "
                    << (angle_like ? "rad" : "m");
    }

    const std::string info_label = info_stream.str();
    if (info_label.empty()) {
        return;
    }

    const tf2::Vector3 axis_unit =
        normalizeOrFallback(joint_axis_world, tf2::Vector3(0.0, 0.0, 1.0));
    const tf2::Vector3 label_lateral =
        makePerpendicularUnit(axis_unit) * (options_.revolute_radius * 0.55);

    const tf2::Vector3 joint_label_position =
        child_position +
        axis_unit * (options_.axis_length * 0.55) +
        label_lateral +
        world_up * (options_.text_lift * 0.30);

    visualization_msgs::msg::Marker info_marker;
    info_marker.header.frame_id = options_.fixed_frame;
    info_marker.header.stamp = this->now();
    info_marker.ns = "joint_info_label";
    info_marker.id = marker_id++;
    info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    info_marker.action = visualization_msgs::msg::Marker::ADD;
    info_marker.pose.position = toPoint(joint_label_position);
    info_marker.pose.orientation.w = 1.0;
    info_marker.scale.z = options_.text_size * 0.92;
    info_marker.color.r = colors.text.r;
    info_marker.color.g = colors.text.g;
    info_marker.color.b = colors.text.b;
    info_marker.color.a = colors.text.a;
    info_marker.text = info_label;
    info_marker.frame_locked = false;
    marker_array.markers.push_back(std::move(info_marker));
}

void JointTfVisualizer::appendRevoluteMarkers(
    visualization_msgs::msg::MarkerArray & marker_array,
    std::int32_t & marker_id,
    const JointDescriptor &,
    const tf2::Vector3 & joint_origin,
    const tf2::Vector3 & joint_axis_world,
    const JointColors & colors) const
{
    const tf2::Vector3 u = makePerpendicularUnit(joint_axis_world);
    const tf2::Vector3 v =
        normalizeOrFallback(joint_axis_world.cross(u), tf2::Vector3(0.0, 1.0, 0.0));

    constexpr double kPi = 3.14159265358979323846;
    constexpr double start_angle = -0.50 * kPi;
    constexpr double end_angle = 1.05 * kPi;
    const int sample_count = std::max(options_.revolute_samples, 12);

    visualization_msgs::msg::Marker arc_marker;
    arc_marker.header.frame_id = options_.fixed_frame;
    arc_marker.header.stamp = this->now();
    arc_marker.ns = "joint_revolute_arc";
    arc_marker.id = marker_id++;
    arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    arc_marker.action = visualization_msgs::msg::Marker::ADD;
    arc_marker.scale.x = options_.connection_width * 0.80;
    arc_marker.color.r = colors.accent.r;
    arc_marker.color.g = colors.accent.g;
    arc_marker.color.b = colors.accent.b;
    arc_marker.color.a = colors.accent.a;
    arc_marker.pose.orientation.w = 1.0;
    arc_marker.points.reserve(static_cast<std::size_t>(sample_count));

    for (int index = 0; index < sample_count; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(sample_count - 1);
        const double angle = start_angle + ratio * (end_angle - start_angle);
        const tf2::Vector3 point = joint_origin +
                                   (u * std::cos(angle) + v * std::sin(angle)) * options_.revolute_radius;
        arc_marker.points.push_back(toPoint(point));
    }

    marker_array.markers.push_back(std::move(arc_marker));

    const double arrow_angle = end_angle;
    const tf2::Vector3 arc_end = joint_origin +
                                 (u * std::cos(arrow_angle) + v * std::sin(arrow_angle)) * options_.revolute_radius;
    const tf2::Vector3 tangent = normalizeOrFallback(
                                     (-u * std::sin(arrow_angle)) + (v * std::cos(arrow_angle)),
                                     u);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = options_.fixed_frame;
    arrow_marker.header.stamp = this->now();
    arrow_marker.ns = "joint_revolute_arrow";
    arrow_marker.id = marker_id++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.scale.x = options_.axis_shaft_diameter * 0.75;
    arrow_marker.scale.y = options_.axis_head_diameter * 0.85;
    arrow_marker.scale.z = options_.axis_head_length * 0.85;
    arrow_marker.color.r = colors.accent.r;
    arrow_marker.color.g = colors.accent.g;
    arrow_marker.color.b = colors.accent.b;
    arrow_marker.color.a = colors.accent.a;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.points.push_back(toPoint(arc_end - tangent * options_.revolute_arrow_length));
    arrow_marker.points.push_back(toPoint(arc_end));
    marker_array.markers.push_back(std::move(arrow_marker));
}

void JointTfVisualizer::appendPrismaticMarkers(
    visualization_msgs::msg::MarkerArray & marker_array,
    std::int32_t & marker_id,
    const JointDescriptor &,
    const tf2::Vector3 & joint_origin,
    const tf2::Vector3 & joint_axis_world,
    const JointColors & colors) const
{
    const tf2::Vector3 negative_tip = joint_origin - joint_axis_world * options_.prismatic_half_length;
    const tf2::Vector3 positive_tip = joint_origin + joint_axis_world * options_.prismatic_half_length;

    visualization_msgs::msg::Marker rail_marker;
    rail_marker.header.frame_id = options_.fixed_frame;
    rail_marker.header.stamp = this->now();
    rail_marker.ns = "joint_prismatic_rail";
    rail_marker.id = marker_id++;
    rail_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    rail_marker.action = visualization_msgs::msg::Marker::ADD;
    rail_marker.scale.x = options_.connection_width * 0.85;
    rail_marker.color.r = colors.accent.r;
    rail_marker.color.g = colors.accent.g;
    rail_marker.color.b = colors.accent.b;
    rail_marker.color.a = colors.accent.a;
    rail_marker.pose.orientation.w = 1.0;
    rail_marker.points.push_back(toPoint(negative_tip));
    rail_marker.points.push_back(toPoint(positive_tip));
    marker_array.markers.push_back(std::move(rail_marker));

    visualization_msgs::msg::Marker positive_arrow;
    positive_arrow.header.frame_id = options_.fixed_frame;
    positive_arrow.header.stamp = this->now();
    positive_arrow.ns = "joint_prismatic_positive";
    positive_arrow.id = marker_id++;
    positive_arrow.type = visualization_msgs::msg::Marker::ARROW;
    positive_arrow.action = visualization_msgs::msg::Marker::ADD;
    positive_arrow.scale.x = options_.axis_shaft_diameter * 0.85;
    positive_arrow.scale.y = options_.axis_head_diameter;
    positive_arrow.scale.z = options_.axis_head_length;
    positive_arrow.color.r = colors.axis.r;
    positive_arrow.color.g = colors.axis.g;
    positive_arrow.color.b = colors.axis.b;
    positive_arrow.color.a = colors.axis.a;
    positive_arrow.pose.orientation.w = 1.0;
    positive_arrow.points.push_back(toPoint(joint_origin));
    positive_arrow.points.push_back(toPoint(positive_tip));
    marker_array.markers.push_back(std::move(positive_arrow));

    visualization_msgs::msg::Marker negative_arrow;
    negative_arrow.header.frame_id = options_.fixed_frame;
    negative_arrow.header.stamp = this->now();
    negative_arrow.ns = "joint_prismatic_negative";
    negative_arrow.id = marker_id++;
    negative_arrow.type = visualization_msgs::msg::Marker::ARROW;
    negative_arrow.action = visualization_msgs::msg::Marker::ADD;
    negative_arrow.scale.x = options_.axis_shaft_diameter * 0.55;
    negative_arrow.scale.y = options_.axis_head_diameter * 0.75;
    negative_arrow.scale.z = options_.axis_head_length * 0.75;
    negative_arrow.color.r = colors.connection.r;
    negative_arrow.color.g = colors.connection.g;
    negative_arrow.color.b = colors.connection.b;
    negative_arrow.color.a = colors.connection.a * 0.75F;
    negative_arrow.pose.orientation.w = 1.0;
    negative_arrow.points.push_back(toPoint(joint_origin));
    negative_arrow.points.push_back(toPoint(negative_tip));
    marker_array.markers.push_back(std::move(negative_arrow));
}

}  // namespace davinci_arm_characterization