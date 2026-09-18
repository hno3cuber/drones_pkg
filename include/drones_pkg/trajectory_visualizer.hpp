// trajectory_visualizer.hpp
//
// Pure visualization companion for `trajectory_follower`.
//
// It never touches the control chain: no PX4 topics, no commands. It only
// listens and republishes what is already happening on the wire, so it can be
// started or killed at any time during flight without affecting the vehicle.
//
// Publishes (每个 uav 一条独立 topic，便于在 RViz 中单独配色):
//   ~/<uav>/flown_path   nav_msgs/Path   该架无人机的实飞三维航迹（增量追加）
//
// 只画无人机自己飞过的三维轨迹，不画任何参考轨迹/进度 marker。
// 点的 z 直接取 PX4 local position 的真实相对高度（NED 的 -z），
// 因此在 RViz 里能直接看到爬升与下降。
//
// 该节点不读轨迹文件、不参与控制：它只订阅实飞位置并画出轨迹。
//
// 参数:
//   uav_namespaces    string 数组，每项为一个 PX4 实例命名空间
//                     （如 ["/fmu", "/px4_1/fmu", "/px4_2/fmu", "/px4_3/fmu"]）
//   frame_id, publish_rate_hz, path_max_points

#ifndef DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
#define DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/path.hpp>

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
  /// 单架无人机的可视化状态：订阅、航迹、跳变检测与发布全在这里。
  struct UavTrack
  {
    std::string ns;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub;
    nav_msgs::msg::Path path;
    double last_x{0.0};
    double last_y{0.0};
    bool has_last{false};
    // 有新点写入后置位，由定时器按 publish_rate_hz 统一发布，避免
    // 高频订阅下每个点都重发整条 Path。
    bool dirty{false};
  };

  void declareParameters();

  // 语法糖：命名空间 "/fmu" -> 话题后缀需拼 "/out/..."，而用于 topic 名的
  // uav 标识则是去掉前导 '/' 并把内部 '/' 换成 '_'，例如 "/px4_1/fmu"
  // -> "px4_1_fmu"。
  static std::string sanitizeName(const std::string & ns);

  // 向单架航迹追加一个点（含分段与容量裁剪），由各自的订阅回调调用
  void appendPoint(
    UavTrack & track, const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  // ---- 坐标换算 ----
  /// NED -> RViz 可视坐标 (East, North)，返回 {x, y}
  std::pair<double, double> toRvizFrame(double ned_x, double ned_y) const;

  // ---- 定时器回调 ----
  // 按 publish_rate_hz 检查，有新增点的航迹才重发整条 Path。
  void onTimer();

  // ---- 参数 ----
  std::vector<std::string> uav_namespaces_{"/fmu"};
  std::string frame_id_{"map"};
  double publish_rate_hz_{2.0};
  int path_max_points_{500000};

  // ---- 状态 ----
  std::vector<UavTrack> tracks_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_VISUALIZER_HPP_
