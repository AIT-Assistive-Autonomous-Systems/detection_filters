Filter and manipulate detection results in [vision_msgs](https://index.ros.org/p/vision_msgs)

Provided components are

* `detection_filters::PoseOffset`: republish `vision_msgs/msg/Detection3DArray` with transformed poses
* `detection_filters::PoseRegionFilter`: republish only those detections in `vision_msgs/msg/Detection3DArray` with poses within a certain xyz region, and match a roll/pitch/yaw tolerance
* `detection_filters::ScoreFilter`: republish only those detections in `vision_msgs/msg/Detection3DArray` which exceed a score threshold