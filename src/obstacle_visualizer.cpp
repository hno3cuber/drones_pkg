// obstacle_visualizer.cpp
//
// 静态障碍物可视化节点：在 RViz 里画出若干圆柱形障碍物。
//
// 设计要点:
//   1. 纯粹的可视化节点，不发布任何控制话题、不订阅任何飞行数据，
//      可以在飞行过程中随时启动/退出，不影响控制链路。
//   2. 障碍物是静态的：只在启动时发一次 MarkerArray，随后速度很低地
//      周期性重发（可选），publisher 设为 transient_local，晚启动的
//      RViz 也能立刻拿到完整障碍物列表。
//   3. 坐标系默认 "map"，与 trajectory_visualizer 保持一致；位置按
//      ENU (x=East, y=North, z=Up) 解释，圆柱底面贴地。
//   4. 圆柱用 visualization_msgs/Cylinder marker 表示：scale.x/y 是直径，
//      scale.z 是高度，而 marker 的 pose.position 是几何中心，因此
//      中心 z = obstacle_height / 2 才能让底面落在 z=0。

#include "drones_pkg/obstacle_visualizer.hpp"

#include <algorithm>
#include <cmath>

namespace drones_pkg
{

namespace
{
constexpr char kMarkerNamespace[] = "obstacles";
}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------
ObstacleVisualizer::ObstacleVisualizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("obstacle_visualizer", options)
{
  declareParameters();

  // transient_local + KeepLast(1)：RViz 后启动也能拿到最近一次的完整列表，
  // 因此不需要高频率重发。
  pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/markers", rclcpp::QoS(1).transient_local());

  buildMarkers();

  // 周期性重发一遍，兜住「RViz 启动早于本节点」或中途 rviz 配置重载的情况。
  // 默认 1 Hz，开销可以忽略。
  const double period = 1.0 / std::clamp(publish_rate_hz_, 0.1, 10.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    [this]() {pub_->publish(markers_);});

  // 立即发一次，不必等第一个定时器周期
  pub_->publish(markers_);

  RCLCPP_INFO(
    get_logger(), "obstacle_visualizer started: %zu obstacle(s), frame=%s, "
    "radius=%.2fm, height=%.2fm",
    obstacles_.size(), frame_id_.c_str(), radius_, height_);
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    RCLCPP_INFO(
      get_logger(), "  obstacle %zu at (%.2f, %.2f), cylinder z=[0, %.2f]",
      i, obstacles_[i].first, obstacles_[i].second, height_);
  }
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void ObstacleVisualizer::declareParameters()
{
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

  // 圆柱几何：半径 [m]、高度 [m]，高度是「底面到顶面」的净高度
  radius_ = declare_parameter<double>("obstacle_radius", 0.5);
  height_ = declare_parameter<double>("obstacle_height", 5.0);

  // 障碍物圆心位置，扁平数组 [x0, y0, x1, y1, ...]（ENU 平面，z 由 height 决定）
  // 默认为三根圆柱：(4,4) (7,7) (15,4)
  obstacles_flat_ = declare_parameter<std::vector<double>>(
    "obstacles", std::vector<double>{4.0, 4.0, 7.0, 7.0, 15.0, 4.0});

  // 颜色与透明度
  color_r_ = declare_parameter<double>("color_r", 0.85);
  color_g_ = declare_parameter<double>("color_g", 0.20);
  color_b_ = declare_parameter<double>("color_b", 0.20);
  alpha_ = declare_parameter<double>("alpha", 0.65);

  // 防御：负值或 0 会让 marker 在 RViz 里消失或翻转
  radius_ = std::max(radius_, 1e-3);
  height_ = std::max(height_, 1e-3);
  alpha_ = std::clamp(alpha_, 0.0, 1.0);

  // 凑不齐成对的坐标就直接报错退出，避免画出半个障碍物
  if (obstacles_flat_.size() % 2 != 0) {
    throw std::runtime_error(
      "参数 obstacles 必须是偶数个元素 [x0, y0, x1, y1, ...]");
  }
  obstacles_.clear();
  obstacles_.reserve(obstacles_flat_.size() / 2);
  for (size_t i = 0; i + 1 < obstacles_flat_.size(); i += 2) {
    obstacles_.emplace_back(obstacles_flat_[i], obstacles_flat_[i + 1]);
  }
}

// ---------------------------------------------------------------------------
// 构建 marker 数组
// ---------------------------------------------------------------------------
void ObstacleVisualizer::buildMarkers()
{
  markers_.markers.clear();
  markers_.markers.reserve(obstacles_.size());

  const auto stamp = now();

  for (size_t i = 0; i < obstacles_.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.ns = kMarkerNamespace;
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    // 几何中心：底面贴地 => 中心在 height/2
    m.pose.position.x = obstacles_[i].first;
    m.pose.position.y = obstacles_[i].second;
    m.pose.position.z = height_ * 0.5;
    m.pose.orientation.w = 1.0;

    // Cylinder 的 scale：x/y = 直径，z = 高度
    m.scale.x = radius_ * 2.0;
    m.scale.y = radius_ * 2.0;
    m.scale.z = height_;

    m.color.r = static_cast<float>(color_r_);
    m.color.g = static_cast<float>(color_g_);
    m.color.b = static_cast<float>(color_b_);
    m.color.a = static_cast<float>(alpha_);

    // 静态障碍物：给一个很长的生命周期，避免 RViz 里超时消失
    m.lifetime = rclcpp::Duration::from_seconds(0.0);

    markers_.markers.push_back(std::move(m));
  }
}

}  // namespace drones_pkg

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drones_pkg::ObstacleVisualizer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
