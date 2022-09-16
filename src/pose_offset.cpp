#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace detection_filters
{

using std::placeholders::_1;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::Detection3DArray;


class PoseOffset : public rclcpp::Node
{
private:
  rclcpp::Subscription<Detection3DArray>::SharedPtr sub_detections_;
  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

public:
  explicit PoseOffset(const rclcpp::NodeOptions & options)
  : Node("pose_offset", options)
  {
    declare_parameter("offset.x", 0.0);
    declare_parameter("offset.y", 0.0);
    declare_parameter("offset.z", 0.0);
    declare_parameter("offset.roll", 0.0);
    declare_parameter("offset.pitch", 0.0);
    declare_parameter("offset.yaw", 0.0);

    sub_detections_ =
      create_subscription<Detection3DArray>(
      "detections", 1,
      std::bind(&PoseOffset::detections_callback, this, _1));
    pub_detections_ = create_publisher<Detection3DArray>("detections_out", 1);
  }

  tf2::Transform get_offset_parameter()
  {
    auto x = get_parameter("offset.x").as_double();
    auto y = get_parameter("offset.y").as_double();
    auto z = get_parameter("offset.z").as_double();
    auto roll = get_parameter("offset.roll").as_double();
    auto pitch = get_parameter("offset.pitch").as_double();
    auto yaw = get_parameter("offset.yaw").as_double();
    tf2::Vector3 translation(x, y, z);
    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    return tf2::Transform(rotation, translation);
  }

  void detections_callback(const Detection3DArray::SharedPtr msg)
  {
    auto transform = get_offset_parameter();
    for (auto & object : msg->detections) {
      tf2::Transform in_pose;
      tf2::fromMsg(object.results[0].pose.pose, in_pose);
      auto out_pose = in_pose * transform;
      tf2::toMsg(out_pose, object.results[0].pose.pose);
    }
    pub_detections_->publish(*msg);
  }

};

} // namespace detection_filters

#include <rclcpp_components/register_node_macro.hpp> // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(detection_filters::PoseOffset)
