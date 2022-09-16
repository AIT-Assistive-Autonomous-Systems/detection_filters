#include <memory>
#include <string>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace detection_filters
{

using std::placeholders::_1;
using geometry_msgs::msg::Pose;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::Detection3DArray;
using rclcpp::Duration;

class PoseRegionFilter : public rclcpp::Node
{
private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<Detection3DArray>::SharedPtr sub_detections_;
  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

  void detections_callback(const Detection3DArray::SharedPtr msg)
  {
    Detection3DArray result;
    result.header = msg->header;
    for (auto & detection : msg->detections) {
      if (is_pose_valid(msg->header, detection.results[0].pose.pose)) {
        result.detections.push_back(detection);
      }
    }
    pub_detections_->publish(result);
  }

public:
  explicit PoseRegionFilter(const rclcpp::NodeOptions & options)
  : Node("pose_region_filter", options)
  {
    declare_parameter("frame_id", "base_link");
    declare_parameter("tf_timeout", 0.1);

    for (const std::string & dim : {"x", "y", "z", "roll", "pitch", "yaw"}) {
      declare_parameter(dim + "_min", -std::numeric_limits<double>::infinity());
      declare_parameter(dim + "_max", std::numeric_limits<double>::infinity());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_detections_ =
      create_subscription<Detection3DArray>(
      "detections", 1,
      std::bind(&PoseRegionFilter::detections_callback, this, _1));
    pub_detections_ = create_publisher<Detection3DArray>("detections_out", 1);
  }

  bool is_dim_within_bounds(const std::string & dim, double value)
  {
    auto value_min = get_parameter(dim + "_min").as_double();
    if (value < value_min) {
      RCLCPP_DEBUG(
        get_logger(), "detection filtered: %s=%.2f<%.2f=%s_min",
        dim.c_str(), value, value_min, dim.c_str());
      return false;
    }
    auto value_max = get_parameter(dim + "_max").as_double();
    if (value > value_max) {
      RCLCPP_DEBUG(
        get_logger(), "detection filtered: %s=%.2f>%.2f=%s_max",
        dim.c_str(), value, value_max, dim.c_str());
      return false;
    }
    return true;
  }

  bool is_pose_valid(const std_msgs::msg::Header & header, const Pose & pose_msg)
  {
    tf2::Transform transform;
    try {
      rclcpp::Time t = header.stamp;
      auto timeout = Duration::from_seconds(get_parameter("tf_timeout").as_double());
      auto transform_msg = tf_buffer_->lookupTransform(
        get_parameter("frame_id").as_string(), header.frame_id, t, timeout);
      tf2::fromMsg(transform_msg.transform, transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "failed to lookup transform to ground: %s", e.what());
      return false;
    }

    tf2::Transform pose;
    tf2::fromMsg(pose_msg, pose);
    pose = transform * pose;

    if (!is_dim_within_bounds("x", pose.getOrigin().x())) {
      return false;
    }
    if (!is_dim_within_bounds("y", pose.getOrigin().y())) {
      return false;
    }
    if (!is_dim_within_bounds("z", pose.getOrigin().z())) {
      return false;
    }

    double yaw, pitch, roll;
    pose.getBasis().getEulerYPR(yaw, pitch, roll);

    if (!is_dim_within_bounds("roll", roll)) {
      return false;
    }
    if (!is_dim_within_bounds("pitch", pitch)) {
      return false;
    }
    if (!is_dim_within_bounds("yaw", yaw)) {
      return false;
    }
    return true;
  }
};

} // namespace detection_filters

#include <rclcpp_components/register_node_macro.hpp> // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(detection_filters::PoseRegionFilter)
