#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace detection_filters
{

using std::placeholders::_1;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::Detection3DArray;

bool any_result_score_lt(double min_score, const Detection3D & detection)
{
  for (const auto & result : detection.results) {
    if (result.hypothesis.score < min_score) {
      return true;
    }
  }
  return false;
}

class ScoreFilter : public rclcpp::Node
{
private:
  rclcpp::Subscription<Detection3DArray>::SharedPtr sub_detections_;
  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

public:
  explicit ScoreFilter(const rclcpp::NodeOptions & options)
  : Node("score_filter", options)
  {
    declare_parameter("min_score", 1.0);
    sub_detections_ =
      create_subscription<Detection3DArray>(
      "detections", 1,
      std::bind(&ScoreFilter::detections_callback, this, _1));
    pub_detections_ = create_publisher<Detection3DArray>("detections_out", 1);
  }

private:
  void detections_callback(const Detection3DArray::SharedPtr msg)
  {
    auto min_score = get_parameter("min_score").as_double();
    auto it_remove =
      std::remove_if(
      msg->detections.begin(), msg->detections.end(),
      std::bind(&any_result_score_lt, min_score, _1));
    msg->detections.erase(it_remove, msg->detections.end());
    pub_detections_->publish(*msg);
  }
};

} // namespace detection_filters

#include <rclcpp_components/register_node_macro.hpp> // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(detection_filters::ScoreFilter)
