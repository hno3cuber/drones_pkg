// trajectory_visualizer.cpp
//
// Pure visualization companion for `trajectory_follower`.
//
// 设计要点:
//   1. 不参与控制，不占用任何 PX4 输入话题；只订阅实飞位置并画出航迹。
//   2. 内部运算是 NED（同 follower），只在写入 RViz 消息时映射到可视坐标系。
//   3. 每架无人机一条独立 topic（~/<uav>/flown_path），便于在 RViz 中单独
//      配色区分。只画无人机自己飞过的三维航迹，点的 z 取真实相对高度。
//      不画参考轨迹、不画进度 marker。
//   4. 实飞航迹只能被动观测：在 RViz 坐标系下判断"是否已在动"，不猜测
//      follower 的状态机阶段。

#include "drones_pkg/trajectory_visualizer.hpp"

#include <algorithm>
#include <cmath>

namespace drones_pkg
{

namespace
{
// 与 trajectory_follower.cpp 共用同一话题后缀与 QoS 约定
constexpr char kLocalPositionSuffix[] = "/out/vehicle_local_position_v1";

// 位置跳变超过该值视为新的一段航迹起点 [m]
constexpr double kTrackResetJump = 5.0;

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------
TrajectoryVisualizer::TrajectoryVisualizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_visualizer", options)
{
  declareParameters();

  // 实飞航迹只订阅，不发任何指令；QoS 与 px4_ros_com 一致
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();

  // 先按数量建好，之后再不改动容器，订阅回调里持有的引用才安全
  tracks_.reserve(uav_namespaces_.size());
  for (const auto & ns : uav_namespaces_) {
    UavTrack track;
    track.ns = ns;
    track.pub = create_publisher<nav_msgs::msg::Path>(
      "~/" + sanitizeName(ns) + "/flown_path",
      rclcpp::QoS(1).transient_local());
    tracks_.push_back(std::move(track));
  }

  for (size_t i = 0; i < tracks_.size(); ++i) {
    const std::string topic = tracks_[i].ns + kLocalPositionSuffix;
    tracks_[i].sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic, qos,
      [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        appendPoint(tracks_[i], msg);
      });
  }

  // 有新增点时才重发整条航迹：Path 是完整快照，无法只发增量，因此用
  // “收点标脏 → 定时器按频率发布”的方式，避免高频订阅下每个点都发一次。
  // 悬停不动时不再空转重发；publisher 为 transient_local，晚启动的 RViz
  // 也能拿到最后一条完整航迹。
  const double period = 1.0 / std::clamp(publish_rate_hz_, 1.0, 50.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    std::bind(&TrajectoryVisualizer::onTimer, this));

  RCLCPP_INFO(
    get_logger(), "trajectory_visualizer started: %zu uav(s), 3D flown path only, frame=%s",
    tracks_.size(), frame_id_.c_str());
  for (const auto & track : tracks_) {
    RCLCPP_INFO(
      get_logger(), "  RViz add: Path[~/%s/flown_path] (3D)",
      sanitizeName(track.ns).c_str());
  }
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::declareParameters()
{
  // frame_id 默认值与 yaml 保持一致，避免带/不带参数文件时行为不同
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
  path_max_points_ = declare_parameter<int>("path_max_points", 500000);

  // 默认只画单机 /fmu；多机时由 launch 传入完整命名空间列表
  uav_namespaces_ = declare_parameter<std::vector<std::string>>(
    "uav_namespaces", std::vector<std::string>{"/fmu"});

  if (path_max_points_ < 10) {
    path_max_points_ = 10;
  }
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
std::string TrajectoryVisualizer::sanitizeName(const std::string & ns)
{
  // "/px4_1/fmu" -> "px4_1_fmu"：去掉前导 '/'，其余 '/' 换成 '_'
  std::string out;
  out.reserve(ns.size());
  for (char c : ns) {
    if (c == '/') {
      if (out.empty()) {continue;}   // 跳过前导 '/'
      out.push_back('_');
    } else {
      out.push_back(c);
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// 坐标
// ---------------------------------------------------------------------------
std::pair<double, double> TrajectoryVisualizer::toRvizFrame(
  double ned_x, double ned_y) const
{
  // NED (North, East) -> ENU (East, North)。
  // 若 RViz 里最好用 NED 视觉（x=North, y=East），把 frame_id 改成一个
  // 绕 z 轴转 -90° 的坐标系即可，算法不需要改。
  return {ned_y, ned_x};
}

// ---------------------------------------------------------------------------
// 航迹追加：单架的一份逻辑，被各自订阅回调调用
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::appendPoint(
  UavTrack & track, const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  if (!msg->xy_valid) {return;}

  const auto veh_xy = toRvizFrame(msg->x, msg->y);
  const double vx = veh_xy.first;
  const double vy = veh_xy.second;

  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now();
  ps.header.frame_id = frame_id_;
  // 三维航迹：z 直接取真实相对高度。PX4 local position 的 z 是 NED
  // （向下为正），映射到 RViz 的 ENU 可视系后取反为向上为正。
  ps.pose.position = makePoint(vx, vy, -static_cast<double>(msg->z));
  // RViz 的 Axes marker 会读取 pose 朝向，这里顺便把 heading 带上。
  // NED 的 heading 绕 z 轴顺时针为正，映射到 ENU 可视坐标系后取反。
  double yaw = 0.0;
  if (msg->heading_good_for_control) {
    yaw = -static_cast<double>(msg->heading);
  }
  ps.pose.orientation.z = std::sin(yaw * 0.5);
  ps.pose.orientation.w = std::cos(yaw * 0.5);

  // 跳变（起飞点重置 / 数据中断）时另起一段，避免拉出横贯全图的长线
  if (track.has_last &&
    std::hypot(vx - track.last_x, vy - track.last_y) > kTrackResetJump)
  {
    track.path.poses.clear();
  }
  track.last_x = vx;
  track.last_y = vy;
  track.has_last = true;

  track.path.header = ps.header;
  track.path.poses.push_back(ps);
  const size_t cap = static_cast<size_t>(std::max(10, path_max_points_));
  if (track.path.poses.size() > cap) {
    track.path.poses.erase(
      track.path.poses.begin(), track.path.poses.begin() +
      static_cast<long>(track.path.poses.size() - cap));
  }

  // 只标脏，真正的发布统一交给定时器，避免 30~50Hz 的订阅频率
  // 变成整条 Path 的重发频率。
  track.dirty = true;
}

// ---------------------------------------------------------------------------
// 定时器：只重发有新点的航迹
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::onTimer()
{
  for (auto & track : tracks_) {
    if (!track.dirty) {continue;}
    track.pub->publish(track.path);
    track.dirty = false;
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
    rclcpp::spin(std::make_shared<drones_pkg::TrajectoryVisualizer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
