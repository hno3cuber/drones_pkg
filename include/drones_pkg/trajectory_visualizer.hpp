// trajectory_visualizer.hpp
//
// Pure visualization companion for `trajectory_follower`.
//
// It never touches the control chain: no PX4 topics, no commands. It only
// listens and republishes what is already happening on the wire, so it can be
// started or killed at any time during flight without affecting the vehicle.
//
// Publishes:
//   ~/flown_path      nav_msgs/Path               无人机实飞航迹（增量追加）
//   ~/markers         visualization_msgs/MarkerArray
//       - progress        已飞过的部分
//       - current_ref     当前 reference setpoint（球）
//
// 坐标系约定与 follower 保持一致:
//   csv 默认 ENU, 内部转 NED (x=North, y=East, z=Down);
//   RViz 里通过 frame_id 映射到可视坐标系（默认 map, x=East, y=North）。
//
// 参数（与 trajectory_follower.yaml 共用，同名同义）:
//   trajectory_file, dt, frame_is_enu, flight_altitude,
//   origin_offset_ned_x, origin_offset_ned_y
// 可视化专属参数:
//   frame_id, loop_trajectory, publish_rate_hz, rviz_real_height,
//   path_max_points

#ifndef DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
#define DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace drones_pkg
{

class TrajectoryVisualizer : public rclcpp::Node
{
public:
  explicit TrajectoryVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();

  // ---- 轨迹加载（与 trajectory_follower 同一套解析规则）----
  bool loadTrajectory();

  // ---- 采样与坐标换算 ----
  /// 按已飞行秒数采样参考轨迹（线性插值），返回 RViz 平面坐标 {x, y}
  std::pair<double, double> sampleReference(double t_sec) const;
  /// NED -> RViz 可视坐标 (East, North)，返回 {x, y}
  std::pair<double, double> toRvizFrame(double ned_x, double ned_y) const;
  /// 参考轨迹在 RViz 中的高度：贴地显示，或按真实飞行高度
  double rvizZ() const;

  // ---- 定时器回调 ----
  void onTimer();

  // ---- 发布辅助 ----
  void buildProgressMarker(
    double t_sec, std::vector<visualization_msgs::msg::Marker> & out) const;

  // ---- 参数 ----
  std::string trajectory_file_;
  double dt_{0.1};
  bool frame_is_enu_{true};
  double flight_altitude_{5.0};
  double origin_offset_x_{0.0};
  double origin_offset_y_{0.0};

  std::string frame_id_{"map"};
  bool loop_trajectory_{false};
  double publish_rate_hz_{10.0};
  bool rviz_real_height_{false};
  int path_max_points_{5000};

  // ---- 轨迹数据 (NED, 已转换) ----
  std::vector<float> traj_x_;   // North
  std::vector<float> traj_y_;   // East
  // 可视坐标缓存 (East, North)，在 loadTrajectory() 内建好，
  // 避免"缓存未填充就被使用"导致的段错误
  std::vector<float> vis_x_;
  std::vector<float> vis_y_;
  double traj_duration_{0.0};

  // ---- 发布状态 ----
  // 说明：参考轨迹的"播放进度"不是从 follower 订阅来的，而是本节点用自己的
  // 定时器按 dt 推进的一个纯显示用估算值。它不会、也无法影响飞行，只是让
  // 参考线随飞行进度逐步变绿并在末端放一个提示球。
  double track_elapsed_{0.0};
  bool finished_{false};
  double last_tracked_time_{-1.0};
  double last_vehicle_x_{0.0};
  double last_vehicle_y_{0.0};
  bool has_last_vehicle_{false};

  // ---- ROS 实体 ----
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_flown_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path flown_path_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
