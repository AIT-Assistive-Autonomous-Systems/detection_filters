cmake_minimum_required(VERSION 3.8)
project(detection_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_offset.cpp
  src/pose_region_filter.cpp
  src/score_filter.cpp)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "detection_filters::PoseOffset" EXECUTABLE pose_offset_node)
set_target_properties(pose_offset_node PROPERTIES OUTPUT_NAME "pose_offset")

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "detection_filters::PoseRegionFilter" EXECUTABLE pose_region_filter_node)
set_target_properties(pose_region_filter_node PROPERTIES OUTPUT_NAME "pose_region_filter")

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "detection_filters::ScoreFilter" EXECUTABLE score_filter_node)
set_target_properties(score_filter_node PROPERTIES OUTPUT_NAME "score_filter")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
