// trajectory_visualizer.cpp
//
// Pure visualization companion for `trajectory_follower`.
//
// 设计要点:
//   1. 与 follower 共享同一份参数 (+同名同义)，因此画出来的参考轨迹与
//      无人机实际跟踪的目标严格一致；没有额外依赖，也不会占用 /fmu 话题。
//   2. 内部运算是 NED（同 follower），只在写入 RViz 消息时映射到可视坐标系。
//   3. 实飞航迹只能被动观测：在 RViz 坐标系下判断"是否已在动"，不猜测
//      follower 的状态机阶段。

#include "drones_pkg/trajectory_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace drones_pkg
{

namespace
{
// 与 trajectory_follower.cpp 共用同一话题与 QoS 约定
constexpr char kLocalPositionTopic[] = "/fmu/out/vehicle_local_position_v1";

// 位置跳变超过该值视为新的一段航迹起点 [m]
constexpr double kTrackResetJump = 5.0;

// marker ns
constexpr char kNsProgress[] = "progress";
constexpr char kNsCurrent[] = "current_ref";

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// 由相邻点求 yaw（左手法则绕 z，即 ROS 约定：x 前, y 左, yaw 逆时针）
[[maybe_unused]] double yawBetween(double x0, double y0, double x1, double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}
}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------
TrajectoryVisualizer::TrajectoryVisualizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_visualizer", options)
{
  declareParameters();

  if (!loadTrajectory()) {
    throw std::runtime_error("Failed to load reference trajectory");
  }

  // 坐标系变换在 loadTrajectory() 末尾完成（vis_x_ / vis_y_ 缓存），
  // 保证任何使用缓存的代码都在填充之后执行。

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
      // 平面显示模式下 pz 取 rvizZ()，需要真实高度时由 rviz_real_height 切换
      ps.pose.position = makePoint(
        vx, vy, rviz_real_height_ ? -static_cast<double>(msg->z) : rvizZ());
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
  pub_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);

  const double period = 1.0 / std::clamp(publish_rate_hz_, 1.0, 50.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    std::bind(&TrajectoryVisualizer::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "trajectory_visualizer started: %zu points, %.1fs, ref=%s, frame=%s",
    traj_x_.size(), traj_duration_, frame_is_enu_ ? "ENU" : "NED", frame_id_.c_str());
  RCLCPP_INFO(
    get_logger(), "RViz add: Path[~/flown_path], MarkerArray[~/markers]");
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::declareParameters()
{
  // --- 与 follower 同名同义：同一份 yaml 可同时喂给两个节点 ---
  trajectory_file_ = declare_parameter<std::string>("trajectory_file", "");
  dt_ = declare_parameter<double>("dt", 0.1);
  frame_is_enu_ = declare_parameter<bool>("frame_is_enu", true);
  flight_altitude_ = declare_parameter<double>("flight_altitude", 5.0);
  origin_offset_x_ = declare_parameter<double>("origin_offset_ned_x", 0.0);
  origin_offset_y_ = declare_parameter<double>("origin_offset_ned_y", 0.0);

  // --- 可视化专属 ---
  // frame_id 默认值与 yaml 保持一致，避免带/不带参数文件时行为不同
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  loop_trajectory_ = declare_parameter<bool>("loop_trajectory", false);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  rviz_real_height_ = declare_parameter<bool>("rviz_real_height", false);
  path_max_points_ = declare_parameter<int>("path_max_points", 5000);

  if (dt_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "dt=%.4f invalid, fallback to 0.1", dt_);
    dt_ = 0.1;
  }
  if (path_max_points_ < 10) {
    path_max_points_ = 10;
  }
}

// ---------------------------------------------------------------------------
// 轨迹加载（与 trajectory_follower::loadTrajectory 同一套规则）
// ---------------------------------------------------------------------------
bool TrajectoryVisualizer::loadTrajectory()
{
  std::string path = trajectory_file_;
  if (path.empty()) {
    path = std::string(PKG_SRC_DIR) + "/uav0.csv";
  }

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(get_logger(), "cannot open trajectory file: %s", path.c_str());
    return false;
  }

  std::vector<std::vector<double>> raw;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {continue;}
    std::vector<double> row;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      if (cell.empty()) {continue;}
      try {
        row.push_back(std::stod(cell));
      } catch (const std::exception &) {
        // 跳过表头等无法解析的单元格
      }
    }
    if (!row.empty()) {raw.push_back(row);}
  }

  if (raw.size() < 2) {
    RCLCPP_ERROR(get_logger(), "trajectory needs >=2 rows, got %zu", raw.size());
    return false;
  }

  const auto & row_x = raw[0];
  const auto & row_y = raw[1];
  const size_t n = std::min(row_x.size(), row_y.size());
  if (n < 2) {
    RCLCPP_ERROR(get_logger(), "trajectory needs >=2 points, got %zu", n);
    return false;
  }

  traj_x_.clear();
  traj_y_.clear();
  traj_x_.reserve(n);
  traj_y_.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double cx = row_x[i];
    const double cy = row_y[i];
    if (frame_is_enu_) {
      traj_x_.push_back(static_cast<float>(cy + origin_offset_x_));
      traj_y_.push_back(static_cast<float>(cx + origin_offset_y_));
    } else {
      traj_x_.push_back(static_cast<float>(cx + origin_offset_x_));
      traj_y_.push_back(static_cast<float>(cy + origin_offset_y_));
    }
  }
  traj_duration_ = static_cast<double>(traj_x_.size() - 1) * dt_;

  // 构建可视坐标缓存（NED -> RViz）。后续采样与进度绘制都依赖它，
  // 因此必须在 loadTrajectory 内完成，而不是留给构造函数 ——
  // 否则任何在缓存填充前使用它的代码都会在空 vector 上越界。
  vis_x_.resize(traj_x_.size());
  vis_y_.resize(traj_x_.size());
  for (size_t i = 0; i < traj_x_.size(); ++i) {
    const auto xy = toRvizFrame(traj_x_[i], traj_y_[i]);
    vis_x_[i] = static_cast<float>(xy.first);
    vis_y_[i] = static_cast<float>(xy.second);
  }

  RCLCPP_INFO(
    get_logger(),
    "loaded %zu points from %s (frame=%s, dt=%.3fs, duration=%.1fs)",
    traj_x_.size(), path.c_str(), frame_is_enu_ ? "ENU" : "NED", dt_, traj_duration_);
  return true;
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

double TrajectoryVisualizer::rvizZ() const
{
  // 平面参考轨迹贴地显示（默认），或悬在真实飞行高度
  return rviz_real_height_ ? flight_altitude_ : 0.0;
}

std::pair<double, double> TrajectoryVisualizer::sampleReference(double t_sec) const
{
  if (traj_x_.size() < 2 || traj_duration_ <= 0.0) {
    return {vis_x_.empty() ? 0.0 : vis_x_.front(), vis_y_.empty() ? 0.0 : vis_y_.front()};
  }

  double t = std::clamp(t_sec, 0.0, traj_duration_);
  if (loop_trajectory_) {
    t = std::fmod(t, traj_duration_);
  }

  const double index_f = dt_ > 0.0 ? t / dt_ : 0.0;
  const size_t i0 = std::min(static_cast<size_t>(std::floor(index_f)), vis_x_.size() - 1);
  const size_t i1 = std::min(i0 + 1, vis_x_.size() - 1);
  const double alpha = index_f - static_cast<double>(i0);

  const double x = vis_x_[i0] + alpha * (vis_x_[i1] - vis_x_[i0]);
  const double y = vis_y_[i0] + alpha * (vis_y_[i1] - vis_y_[i0]);
  return {x, y};
}

// ---------------------------------------------------------------------------
// 定时器：推进进度 + 发布
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::onTimer()
{
  const double nominal_dt = 1.0 / std::clamp(publish_rate_hz_, 1.0, 50.0);

  if (!finished_) {
    if (last_tracked_time_ < 0.0) {
      // 首帧只锚定时间基线，不推进，避免把 dt 算进进度里
      last_tracked_time_ = 0.0;
    } else {
      track_elapsed_ += nominal_dt;
    }
  }

  if (track_elapsed_ >= traj_duration_) {
    track_elapsed_ = traj_duration_;
    finished_ = true;
  }

  // 1) 已飞过的部分
  std::vector<visualization_msgs::msg::Marker> markers;
  buildProgressMarker(track_elapsed_, markers);

  // 2) 当前 reference setpoint（只保留球）
  if (!finished_) {
    const auto p = sampleReference(track_elapsed_);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = now();
    m.ns = kNsCurrent;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = makePoint(p.first, p.second, rvizZ());
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.45;
    m.color = makeColor(1.0f, 0.85f, 0.1f, 0.9f);
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    markers.push_back(m);
  }

  // 3) 发布
  visualization_msgs::msg::MarkerArray arr;
  arr.markers = std::move(markers);
  pub_markers_->publish(arr);

  pub_flown_path_->publish(flown_path_);
}

// ---------------------------------------------------------------------------
// Marker 构造
// ---------------------------------------------------------------------------
void TrajectoryVisualizer::buildProgressMarker(
  double t_sec, std::vector<visualization_msgs::msg::Marker> & out) const
{
  if (vis_x_.size() < 2) {return;}

  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = now();
  m.ns = kNsProgress;
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.22;
  m.color = makeColor(0.15f, 1.0f, 0.45f, 1.0f);
  m.lifetime = rclcpp::Duration::from_seconds(0.0);
  m.frame_locked = true;

  // 按索引切片：进度时间 -> 已行进到的轨迹点下标，直接取前缀
  const double clamped = std::clamp(t_sec, 0.0, traj_duration_);
  const double index_f = dt_ > 0.0 ? clamped / dt_ : 0.0;
  const size_t last = std::min(
    static_cast<size_t>(std::floor(index_f)), vis_x_.size() - 1);

  const double z = rvizZ() + 0.01;  // 抬高一丝，避免与参考线 z-fighting
  const size_t end = std::min(last + 1, vis_x_.size());
  m.points.reserve(end);
  for (size_t i = 0; i < end; ++i) {
    m.points.push_back(makePoint(vis_x_[i], vis_y_[i], z));
  }

  if (m.points.size() < 2) {
    // 只有一个点时 LINE_STRIP 不可见，复制一份即可显示一小段
    m.points.push_back(m.points.front());
  }
  out.push_back(std::move(m));
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
