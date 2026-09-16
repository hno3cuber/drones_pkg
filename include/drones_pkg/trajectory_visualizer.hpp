// trajectory_visualizer.hpp
//
// Pure visualization companion for `trajectory_follower`.
//
// It never touches the control chain: no PX4 topics, no commands. It only
// listens and republishes what is already happening on the wire, so it can be
// started or killed at any time during flight without affecting the vehicle.
//
// Publishes:
//   ~/flown_path      nav_msgs/Path               无人机实飞三维航迹（增量追加）
//
// 只画无人机自己飞过的三维轨迹，不画任何参考轨迹/进度 marker。
// 点的 z 直接取 PX4 local position 的真实相对高度（NED 的 -z），
// 因此在 RViz 里能直接看到爬升与下降。
//
// 该节点不读轨迹文件、不参与控制：它只订阅实飞位置并画出轨迹。
//
// 参数:
//   frame_id, publish_rate_hz, path_max_points

#ifndef DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
#define DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/path.hpp>

#include <string>
#include <utility>

namespace drones_pkg
{

class TrajectoryVisualizer : public rclcpp::Node
{
public:
  explicit TrajectoryVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();

  // ---- 坐标换算 ----
  /// NED -> RViz 可视坐标 (East, North)，返回 {x, y}
  std::pair<double, double> toRvizFrame(double ned_x, double ned_y) const;

  // ---- 定时器回调 ----
  void onTimer();

  // ---- 参数 ----
  std::string frame_id_{"map"};
  double publish_rate_hz_{10.0};
  int path_max_points_{5000};

  // ---- 发布状态 ----
  // 跳变检测用的上一帧位置（RViz 平面坐标），用于在起飞点重置/数据中断时
  // 另起一段航迹，避免拉出横贯全图的假线。
  double last_vehicle_x_{0.0};
  double last_vehicle_y_{0.0};
  bool has_last_vehicle_{false};

  // ---- ROS 实体 ----
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_flown_path_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path flown_path_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
