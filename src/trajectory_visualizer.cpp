// trajectory_visualizer.cpp
//
// Pure visualization companion for `trajectory_follower`.
//
// 设计要点:
//   1. 不参与控制，不占用任何 PX4 输入话题；只订阅实飞位置并画出航迹。
//   2. 内部运算是 NED（同 follower），只在写入 RViz 消息时映射到可视坐标系。
//   3. 只画无人机自己飞过的三维航迹：订阅 /fmu/out/vehicle_local_position_v1，
//      把每个点追加到 nav_msgs/Path，点的 z 取真实相对高度（-z）。
//      不画参考轨迹、不画进度 marker。
//   4. 实飞航迹只能被动观测：在 RViz 坐标系下判断"是否已在动"，不猜测
//      follower 的状态机阶段。

#include "drones_pkg/trajectory_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace drones_pkg
{

namespace
{
// 与 trajectory_follower.cpp 共用同一话题与 QoS 约定
constexpr char kLocalPositionTopic[] = "/fmu/out/vehicle_local_position_v1";

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
  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    kLocalPositionTopic, qos,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
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
      if (has_last_vehicle_ &&
        std::hypot(vx - last_vehicle_x_, vy - last_vehicle_y_) > kTrackResetJump)
      {
        flown_path_.poses.clear();
      }
      last_vehicle_x_ = vx;
      last_vehicle_y_ = vy;
      has_last_vehicle_ = true;

      flown_path_.header = ps.header;
      flown_path_.poses.push_back(ps);
      const size_t cap = static_cast<size_t>(std::max(10, path_max_points_));
      if (flown_path_.poses.size() > cap) {
        flown_path_.poses.erase(
          flown_path_.poses.begin(), flown_path_.poses.begin() +
          static_cast<long>(flown_path_.poses.size() - cap));
      }
    });

  pub_flown_path_ = create_publisher<nav_msgs::msg::Path>("~/flown_path", 1);

  // 以固定频率重发当前航迹，保证 RViz 订阅后即使无人机悬停不动也能立即
  // 看到完整航迹（Path 内容由订阅回调增量维护）。
  const double period = 1.0 / std::clamp(publish_rate_hz_, 1.0, 50.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    std::bind(&TrajectoryVisualizer::onTimer, this));

  RCLCPP_INFO(
    get_logger(), "trajectory_visualizer started: 3D flown path only, frame=%s",
    frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "RViz add: Path[~/flown_path] (3D)");
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::declareParameters()
{
  // frame_id 默认值与 yaml 保持一致，避免带/不带参数文件时行为不同
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  path_max_points_ = declare_parameter<int>("path_max_points", 5000);

  if (path_max_points_ < 10) {
    path_max_points_ = 10;
  }
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
// 定时器：周期性重发实飞航迹
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::onTimer()
{
  pub_flown_path_->publish(flown_path_);
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
