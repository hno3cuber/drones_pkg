// trajectory_follower.cpp
//
// 让 PX4 无人机按 csv 中的参考轨迹飞行（OFFBOARD 模式）。
//
// 状态机:
//   WAIT_SYNC      等待 EKF 位置有效
//   SEND_SETPOINT  先持续发一段 setpoint，让 PX4 接受 offboard 心跳
//   OFFBOARD       切换 OFFBOARD 模式
//   ARM            发送解锁指令
//   APPROACH       从当前位置飞向轨迹起点（含爬升到目标高度）
//   TRACK          逐点播放参考轨迹
//   HOLD           轨迹播放完毕，保持在终点
//   RTL            返航
//
// 坐标系约定:
//   输入 csv 默认按 ENU (x=East, y=North) 解释,
//   内部转换为 PX4 使用的 NED (x=North, y=East, z=Down)。

#include "drones_pkg/trajectory_follower.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace drones_pkg
{

namespace
{
// 状态机各阶段进入时的超时保护 [s]
constexpr double kSyncTimeout = 60.0;
constexpr double kOffboardTimeout = 10.0;

// 发送 setpoint 的最短预热时间 [s]，超过后再切 offboard
constexpr double kSetpointPreheat = 1.0;

// 到达轨迹起点后，等待多少时间才开始计时播放轨迹 [s]
constexpr double kApproachSettle = 0.5;

// PX4 指令
constexpr uint32_t kCmdArmDisarm =
  px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
constexpr uint32_t kCmdDoSetMode =
  px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
constexpr uint32_t kCmdNavRtl =
  px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;

// PX4 自定义主模式：OFFBOARD
constexpr float kPx4CustomMainModeOffboard = 6.0f;
}  // namespace

TrajectoryFollower::TrajectoryFollower(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_follower", options)
{
  declareParameters();
  loadParameters();

  // ---- QoS: 与 px4_ros_com 保持一致（BEST_EFFORT，深度 1）----
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();

  const std::string ns = node_namespace_;
  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    ns + "/out/vehicle_local_position_v1", qos,
    std::bind(&TrajectoryFollower::onLocalPosition, this, std::placeholders::_1));
  sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    ns + "/out/vehicle_status_v1", qos,
    std::bind(&TrajectoryFollower::onVehicleStatus, this, std::placeholders::_1));

  pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    ns + "/in/offboard_control_mode", qos);
  pub_setpoint_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    ns + "/in/trajectory_setpoint", qos);
  pub_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    ns + "/in/vehicle_command", qos);

  // 完成信号用 transient_local：即使绘图节点在飞行末尾才启动，
  // 订阅后也会立刻收到那一帧 latched 的 true。
  rclcpp::QoS finished_qos(rclcpp::KeepLast(1));
  finished_qos.transient_local();
  pub_finished_ = create_publisher<std_msgs::msg::Bool>(
    node_namespace_ + "/trajectory_finished", finished_qos);

  if (!loadTrajectory()) {
    throw std::runtime_error("Failed to load reference trajectory");
  }

  // 注意：不能依赖 setState 来初始化计时器。
  // state_ 初值就是 kWaitSync，setState(kWaitSync) 会因相等而提前返回，
  // 导致 state_enter_time_ 停留在 epoch，elapsed 立即巨量超时。
  state_ = State::kWaitSync;
  state_enter_time_ = std::chrono::steady_clock::now();

  const auto period =
    std::chrono::duration<double>(1.0 / std::max(1.0, setpoint_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TrajectoryFollower::stepStateMachine, this));

  RCLCPP_INFO(
    get_logger(), "trajectory_follower started: %zu points, %.1fs, altitude %.2fm",
    traj_x_.size(), traj_duration_, flight_altitude_);
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void TrajectoryFollower::declareParameters()
{
  node_namespace_ = declare_parameter<std::string>("node_namespace", "/fmu");
  target_system_ = declare_parameter<int>("target_system", 1);
  trajectory_file_ = declare_parameter<std::string>("trajectory_file", "");
  dt_ = declare_parameter<double>("dt", 0.1);
  frame_is_enu_ = declare_parameter<bool>("frame_is_enu", true);
  flight_altitude_ = declare_parameter<double>("flight_altitude", 5.0);
  origin_offset_x_ = declare_parameter<double>("origin_offset_ned_x", 0.0);
  origin_offset_y_ = declare_parameter<double>("origin_offset_ned_y", 0.0);
  approach_speed_ = declare_parameter<double>("approach_speed", 2.0);
  setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
  approach_tolerance_ = declare_parameter<double>("approach_tolerance", 0.5);
  loop_trajectory_ = declare_parameter<bool>("loop_trajectory", false);
  end_action_ = declare_parameter<std::string>("end_action", "hold");

  avoidance_enable_ = declare_parameter<bool>("avoidance_enable", true);
  avoidance_obstacles_ = declare_parameter<std::vector<double>>(
    "avoidance_obstacles", std::vector<double>{});
  avoidance_radius_ = declare_parameter<double>("avoidance_radius", 1.5);
  export_avoided_csv_ = declare_parameter<std::string>("export_avoided_csv", "");
  arc_side_ = declare_parameter<std::string>("avoidance_arc_side", "short");
  arc_side_by_obstacle_ = declare_parameter<std::vector<std::string>>(
    "avoidance_arc_side_by_obstacle", std::vector<std::string>{});
}

void TrajectoryFollower::loadParameters()
{
  if (dt_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "dt=%.4f invalid, fallback to 0.1", dt_);
    dt_ = 0.1;
  }
  if (setpoint_rate_hz_ < 2.0) {
    RCLCPP_WARN(
      get_logger(), "setpoint_rate_hz=%.1f too low for PX4, use 20.0", setpoint_rate_hz_);
    setpoint_rate_hz_ = 20.0;
  }
  setpoint_rate_hz_ = std::min(setpoint_rate_hz_, 200.0);

  if (avoidance_radius_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "avoidance_radius=%.3f invalid, avoidance disabled", avoidance_radius_);
    avoidance_enable_ = false;
  }
  if (avoidance_obstacles_.size() % 2 != 0) {
    RCLCPP_WARN(
      get_logger(), "avoidance_obstacles has odd size %zu, last value ignored",
      avoidance_obstacles_.size());
    avoidance_obstacles_.pop_back();
  }
  RCLCPP_INFO(
    get_logger(), "avoidance: %s, %zu obstacle(s), safety radius %.2fm",
    avoidance_enable_ ? "on" : "off", avoidance_obstacles_.size() / 2, avoidance_radius_);
}

// ---------------------------------------------------------------------------
// 轨迹加载
// ---------------------------------------------------------------------------
bool TrajectoryFollower::loadTrajectory()
{
  std::string path = trajectory_file_;
  if (path.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "trajectory_file is empty; set it explicitly (e.g. via launch/params)");
    return false;
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
        // 跳过无法解析的单元格（例如表头）
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
    float nx, ny;
    if (frame_is_enu_) {
      // ENU -> NED: North = y_enu, East = x_enu
      nx = static_cast<float>(cy + origin_offset_x_);
      ny = static_cast<float>(cx + origin_offset_y_);
    } else {
      nx = static_cast<float>(cx + origin_offset_x_);
      ny = static_cast<float>(cy + origin_offset_y_);
    }
    traj_x_.push_back(nx);
    traj_y_.push_back(ny);
  }

  traj_duration_ = static_cast<double>(traj_x_.size() - 1) * dt_;

  RCLCPP_INFO(
    get_logger(), "loaded %zu points from %s (frame=%s, dt=%.3fs, duration=%.1fs)",
    traj_x_.size(), path.c_str(), frame_is_enu_ ? "ENU" : "NED", dt_, traj_duration_);

  // 避障：把落入障碍安全圆的轨迹段替换成沿圆周的弧（一次性预处理）
  const size_t avoided = applyObstacleAvoidance();
  if (avoided > 0) {
    traj_duration_ = static_cast<double>(traj_x_.size() - 1) * dt_;
    RCLCPP_INFO(
      get_logger(), "avoidance applied: %zu segment(s), now %zu points, %.1fs",
      avoided, traj_x_.size(), traj_duration_);
  }

  // 可选：把"平移 + 绕行"后的结果导出成新 csv（离线固化用）
  if (!export_avoided_csv_.empty() && !exportAvoidedCsv(export_avoided_csv_)) {
    RCLCPP_WARN(
      get_logger(), "failed to export avoided trajectory to %s",
      export_avoided_csv_.c_str());
  }
  return true;
}

bool TrajectoryFollower::exportAvoidedCsv(const std::string & path) const
{
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }

  // 轨迹点已含平移（在加载时加过 origin_offset），此处只把 NED 转回 ENU：
  // North = y_enu, East = x_enu -> csv 第一行为 x_enu，第二行为 y_enu。
  // 高精度输出，避免边缘点因截断漂回安全圆内。
  ofs << std::setprecision(12);
  for (size_t i = 0; i < traj_x_.size(); ++i) {
    if (i > 0) {ofs << ',';}
    ofs << (frame_is_enu_ ? traj_y_[i] : traj_x_[i]);
  }
  ofs << '\n';
  for (size_t i = 0; i < traj_y_.size(); ++i) {
    if (i > 0) {ofs << ',';}
    ofs << (frame_is_enu_ ? traj_x_[i] : traj_y_[i]);
  }
  ofs << '\n';

  RCLCPP_INFO(
    get_logger(), "exported %zu points to %s (frame=%s)",
    traj_x_.size(), path.c_str(), frame_is_enu_ ? "ENU" : "NED");
  return true;
}

size_t TrajectoryFollower::applyObstacleAvoidance()
{
  if (!avoidance_enable_ || avoidance_obstacles_.size() < 2 || avoidance_radius_ <= 0.0) {
    return 0;
  }

  // 障碍圆心参数按 ENU 解释（与 obstacle_visualizer 一致），
  // 轨迹点已在 NED：North = y_enu, East = x_enu。
  const double R = avoidance_radius_;
  // 判据容差：轨迹点是 float，绕行弧上的点到圆心的距离理论上正好是 R，
  // 但经过 float 存储、csv 往返、坐标转换后会有 ~1e-7 量级的误差。
  // 若用严格的 d < R，这些贴边点会被判成"仍在圆内"，导致对已绕行的
  // 轨迹反复二次绕行（csv 导出后再加载时尤其明显）。把判据收紧到
  // d < R - kEps，使贴边点不再触发，避障对自身输出幂等。
  constexpr double kEps = 1e-3;
  const double R_in = R - kEps;

  size_t total_segments = 0;
  const size_t num_obs = avoidance_obstacles_.size() / 2;

  // 障碍之间可能相距较近，逐个障碍依次重算；
  // 每次重算都基于当前（可能已被前一个障碍改过的）点序列。
  for (size_t oi = 0; oi < num_obs; ++oi) {
    const double ox = avoidance_obstacles_[oi * 2 + 1];  // North = y_enu
    const double oy = avoidance_obstacles_[oi * 2];      // East  = x_enu
    // 该障碍的弧向：按障碍下标覆盖，缺省用全局值
    const std::string side = (oi < arc_side_by_obstacle_.size())
      ? arc_side_by_obstacle_[oi] : arc_side_;

    // 标记哪些点在安全圆内
    const size_t n = traj_x_.size();
    std::vector<bool> inside(n, false);
    for (size_t i = 0; i < n; ++i) {
      const double dx = traj_x_[i] - ox;
      const double dy = traj_y_[i] - oy;
      inside[i] = (dx * dx + dy * dy) < R_in * R_in;
    }

    // 从后往前处理连续段，避免索引失效
    size_t i = n;
    while (i > 0) {
      --i;
      if (!inside[i]) {
        continue;
      }
      // 找到连续段的起点（含）与终点（不含）
      size_t seg_end = i + 1;
      size_t seg_begin = i;
      while (seg_begin > 0 && inside[seg_begin - 1]) {
        --seg_begin;
      }

      const size_t i_before = (seg_begin == 0) ? 0 : seg_begin - 1;
      const size_t i_after = (seg_end >= n) ? n - 1 : seg_end;

      // 用圆外相邻点与圆内点线性插值，求与安全圆的交点
      auto boundaryPoint = [&](size_t i_out, size_t i_in) -> std::pair<double, double> {
        const double ax = traj_x_[i_out], ay = traj_y_[i_out];
        const double bx = traj_x_[i_in], by = traj_y_[i_in];
        const double vx = bx - ax, vy = by - ay;
        // 解 |A + s*v - C| = R，取 [0,1] 内最接近圆的那一个根
        const double fx = ax - ox, fy = ay - oy;
        const double a = vx * vx + vy * vy;
        const double b = 2.0 * (fx * vx + fy * vy);
        const double c = fx * fx + fy * fy - R * R;
        double s = 0.0;
        if (a > 1e-12) {
          const double disc = b * b - 4.0 * a * c;
          if (disc >= 0.0) {
            const double sq = std::sqrt(disc);
            const double s1 = (-b - sq) / (2.0 * a);
            const double s2 = (-b + sq) / (2.0 * a);
            // 选落在 [0,1] 内、且更靠近圆内的那个根
            if (s1 >= 0.0 && s1 <= 1.0) {s = s1;} else if (s2 >= 0.0 && s2 <= 1.0) {s = s2;}
          }
        }
        return {ax + s * vx, ay + s * vy};
      };

      const auto entry = boundaryPoint(i_before, seg_begin);
      const auto exit = boundaryPoint(i_after, seg_end - 1);

      // 进出点在圆周上的方位角（标准 atan2）
      double a_entry = std::atan2(entry.second - oy, entry.first - ox);
      double a_exit = std::atan2(exit.second - oy, exit.first - ox);

      // 选弧方向：
      //   short（默认）：跨度归一化到 (-pi, pi]，走角度差较小的一侧
      //   long ：走另一侧（角度差较大）
      double delta = a_exit - a_entry;
      while (delta > M_PI) {delta -= 2.0 * M_PI;}
      while (delta <= -M_PI) {delta += 2.0 * M_PI;}

      if (side == "long") {
        delta = (delta > 0.0) ? delta - 2.0 * M_PI : delta + 2.0 * M_PI;
      }

      // 弧长与铺点数：等角度铺，点数按弧长 / 原轨迹步长决定，
      // 使弧上的单点位移与原轨迹同量级。
      const double arc_len = std::abs(delta) * R;
      // 参考步长取圆外相邻两点的间距（轨迹进入圆前的真实步长），
      // 不能用跨圆内的直线距离，那与局部步长无关。
      const size_t i_ref_a = (seg_begin == 0) ? 0 : seg_begin - 1;
      const size_t i_ref_b = (i_ref_a == 0 && seg_begin > 0) ? seg_begin : (i_ref_a + 1);
      const double dxb = traj_x_[i_ref_b] - traj_x_[i_ref_a];
      const double dyb = traj_y_[i_ref_b] - traj_y_[i_ref_a];
      double ref_step = std::sqrt(dxb * dxb + dyb * dyb);
      if (ref_step < 1e-6) {
        // 退化情况：用整条轨迹的平均步长兜底
        ref_step = traj_duration_ > 0.0 && traj_x_.size() > 1 ?
          (std::hypot(traj_x_.back() - traj_x_.front(), traj_y_.back() - traj_y_.front()) /
          static_cast<double>(traj_x_.size() - 1)) : 0.05;
      }
      ref_step = std::max(ref_step, 1e-4);
      size_t num_arc_pts = static_cast<size_t>(std::ceil(arc_len / ref_step));
      num_arc_pts = std::max<size_t>(num_arc_pts, 4);   // 至少几个点，保证弧看得见
      num_arc_pts = std::min<size_t>(num_arc_pts, 20000);

      std::vector<std::pair<double, double>> arc_pts;
      arc_pts.reserve(num_arc_pts + 1);
      arc_pts.emplace_back(entry.first, entry.second);
      for (size_t k = 1; k < num_arc_pts; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(num_arc_pts);
        const double ang = a_entry + delta * t;
        arc_pts.emplace_back(ox + R * std::cos(ang), oy + R * std::sin(ang));
      }
      arc_pts.emplace_back(exit.first, exit.second);

      // 用弧上的点替换 [seg_begin, seg_end) 这一段
      traj_x_.erase(
        traj_x_.begin() + static_cast<long>(seg_begin),
        traj_x_.begin() + static_cast<long>(seg_end));
      traj_y_.erase(
        traj_y_.begin() + static_cast<long>(seg_begin),
        traj_y_.begin() + static_cast<long>(seg_end));

      std::vector<float> ax_ins, ay_ins;
      ax_ins.reserve(arc_pts.size());
      ay_ins.reserve(arc_pts.size());
      for (const auto & q : arc_pts) {
        ax_ins.push_back(static_cast<float>(q.first));
        ay_ins.push_back(static_cast<float>(q.second));
      }
      traj_x_.insert(traj_x_.begin() + static_cast<long>(seg_begin), ax_ins.begin(), ax_ins.end());
      traj_y_.insert(traj_y_.begin() + static_cast<long>(seg_begin), ay_ins.begin(), ay_ins.end());

      ++total_segments;
      i = seg_begin;  // 继续向前扫描
    }
  }

  return total_segments;
}

std::array<float, 3> TrajectoryFollower::sampleTrajectory(double t_sec) const
{
  const float z = static_cast<float>(-flight_altitude_);
  if (traj_x_.size() < 2 || traj_duration_ <= 0.0) {
    const float x0 = traj_x_.empty() ? 0.0f : traj_x_.front();
    const float y0 = traj_y_.empty() ? 0.0f : traj_y_.front();
    return {x0, y0, z};
  }

  if (t_sec <= 0.0) {
    return {traj_x_.front(), traj_y_.front(), z};
  }

  double t = t_sec;
  if (loop_trajectory_) {
    t = std::fmod(t, traj_duration_);
  } else if (t >= traj_duration_) {
    return {traj_x_.back(), traj_y_.back(), z};
  }

  const double index_f = t / dt_;
  const size_t i0 = static_cast<size_t>(std::floor(index_f));
  const size_t i1 = std::min(i0 + 1, traj_x_.size() - 1);
  const double alpha = index_f - static_cast<double>(i0);

  const float x = static_cast<float>(traj_x_[i0] + alpha * (traj_x_[i1] - traj_x_[i0]));
  const float y = static_cast<float>(traj_y_[i0] + alpha * (traj_y_[i1] - traj_y_[i0]));
  return {x, y, z};
}

// ---------------------------------------------------------------------------
// 发布
// ---------------------------------------------------------------------------
void TrajectoryFollower::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = nowMicros();
  msg.position = true;
  pub_offboard_mode_->publish(msg);
}

void TrajectoryFollower::publishTrajectorySetpoint(float x, float y, float z, float yaw)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = nowMicros();
  msg.position = {x, y, z};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = yaw;
  msg.yawspeed = nan;
  pub_setpoint_->publish(msg);
}

void TrajectoryFollower::publishVehicleCommand(uint32_t command, float p1, float p2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = nowMicros();
  msg.command = command;
  msg.param1 = p1;
  msg.param2 = p2;
  msg.target_system = static_cast<uint8_t>(target_system_);
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  pub_command_->publish(msg);
}

void TrajectoryFollower::arm()
{
  publishVehicleCommand(kCmdArmDisarm, 1.0f, 0.0f);
  RCLCPP_INFO(get_logger(), "-> ARM");
}

void TrajectoryFollower::engageOffboardMode()
{
  publishVehicleCommand(kCmdDoSetMode, 1.0f, kPx4CustomMainModeOffboard);
  RCLCPP_INFO(get_logger(), "-> OFFBOARD");
}

void TrajectoryFollower::sendRtl()
{
  publishVehicleCommand(kCmdNavRtl, 0.0f, 0.0f);
  RCLCPP_INFO(get_logger(), "-> RTL");
}

void TrajectoryFollower::publishFinished()
{
  if (finished_published_) {return;}
  finished_published_ = true;

  std_msgs::msg::Bool msg;
  msg.data = true;
  pub_finished_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "-> trajectory finished signal published on '%s'",
    (node_namespace_ + "/trajectory_finished").c_str());
}

// ---------------------------------------------------------------------------
// 回调
// ---------------------------------------------------------------------------
void TrajectoryFollower::onLocalPosition(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_pos_ = msg;
}

void TrajectoryFollower::onVehicleStatus(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  status_ = msg;
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
uint64_t TrajectoryFollower::nowMicros()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

float TrajectoryFollower::distance2d(float x1, float y1, float x2, float y2) const
{
  const float dx = x1 - x2;
  const float dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

const char * TrajectoryFollower::stateName(State s) const
{
  switch (s) {
    case State::kWaitSync: return "WAIT_SYNC";
    case State::kSendSetpoint: return "SEND_SETPOINT";
    case State::kOffboard: return "OFFBOARD";
    case State::kApproach: return "APPROACH";
    case State::kTrack: return "TRACK";
    case State::kHold: return "HOLD";
    case State::kRtl: return "RTL";
    case State::kFailsafe: return "FAILSAFE";
  }
  return "UNKNOWN";
}

void TrajectoryFollower::setState(State s)
{
  if (s == state_) {return;}
  RCLCPP_INFO(get_logger(), "state: %s -> %s", stateName(state_), stateName(s));
  state_ = s;
  state_enter_time_ = std::chrono::steady_clock::now();
  arm_sent_ = false;
  rtl_sent_ = false;
  hold_sent_ = false;
}

/// 当前状态已持续的秒数
static double elapsedSeconds(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// ---------------------------------------------------------------------------
// 状态机
// ---------------------------------------------------------------------------
void TrajectoryFollower::stepStateMachine()
{
  const double elapsed = elapsedSeconds(state_enter_time_);
  const float x0 = traj_x_.empty() ? 0.0f : traj_x_.front();
  const float y0 = traj_y_.empty() ? 0.0f : traj_y_.front();
  const float z_target = static_cast<float>(-flight_altitude_);

  // offboard 心跳：除 WAIT_SYNC 之外全程发布，否则 PX4 会掉出 offboard
  if (state_ != State::kWaitSync && state_ != State::kFailsafe) {
    publishOffboardControlMode();
  }

  switch (state_) {
    case State::kWaitSync: {
      if (!local_pos_) {
        if (elapsed > kSyncTimeout) {
          RCLCPP_ERROR(get_logger(), "timeout waiting for vehicle_local_position");
          setState(State::kFailsafe);
        }
        return;
      }
      if (!local_pos_->xy_valid || !local_pos_->z_valid) {
        return;
      }
      RCLCPP_INFO(
        get_logger(), "EKF ready, start pos (%.2f, %.2f, %.2f)",
        local_pos_->x, local_pos_->y, local_pos_->z);
      setState(State::kSendSetpoint);
      return;
    }

    case State::kSendSetpoint: {
      // 先原地发一段 setpoint（当前位置 + 目标高度），建立 offboard 心跳
      if (local_pos_) {
        publishTrajectorySetpoint(local_pos_->x, local_pos_->y, z_target, NAN);
      } else {
        publishTrajectorySetpoint(0.0f, 0.0f, z_target, NAN);
      }
      if (elapsed >= kSetpointPreheat) {
        setState(State::kOffboard);
      }
      return;
    }

    case State::kOffboard: {
      if (local_pos_) {
        publishTrajectorySetpoint(local_pos_->x, local_pos_->y, z_target, NAN);
      }
      if (!arm_sent_) {
        arm();
        arm_sent_ = true;
      }
      // 等 PX4 真正进入 offboard 且已解锁
      bool offboard = status_ && status_->nav_state ==
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
      bool armed = status_ && status_->arming_state ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
      if (offboard && armed) {
        setState(State::kApproach);
      } else if (elapsed > kOffboardTimeout) {
        RCLCPP_WARN(get_logger(), "offboard/arm not confirmed, retrying commands");
        arm();
        engageOffboardMode();
        state_enter_time_ = std::chrono::steady_clock::now();
      }
      return;
    }

    case State::kApproach: {
      // 从当前位置飞向轨迹起点(x0, y0, alt)
      float cx = 0.0f, cy = 0.0f;
      if (local_pos_) {
        cx = local_pos_->x;
        cy = local_pos_->y;
      }
      const float dist = distance2d(cx, cy, x0, y0);
      // 限速：每周期最多靠近 approach_speed/rate 的距离
      const float step =
        static_cast<float>(approach_speed_ * (1.0 / std::max(1.0, setpoint_rate_hz_)));
      float tx = x0, ty = y0;
      if (dist > step && dist > 1e-3f) {
        tx = cx + (x0 - cx) / dist * step;
        ty = cy + (y0 - cy) / dist * step;
      }
      publishTrajectorySetpoint(tx, ty, z_target, NAN);

      const float dz = std::abs(local_pos_ ? (local_pos_->z + flight_altitude_) : 0.0f);
      if (dist < approach_tolerance_ && dz < approach_tolerance_) {
        RCLCPP_INFO(get_logger(), "reached trajectory start, begin tracking");
        track_elapsed_ = 0.0;
        setState(State::kTrack);
      }
      return;
    }

    case State::kTrack: {
      track_elapsed_ += 1.0 / std::max(1.0, setpoint_rate_hz_);
      const auto p = sampleTrajectory(track_elapsed_);
      publishTrajectorySetpoint(p[0], p[1], p[2], NAN);

      if (!loop_trajectory_ && track_elapsed_ >= traj_duration_) {
        RCLCPP_INFO(get_logger(), "trajectory finished (%.1fs)", track_elapsed_);
        if (end_action_ == "rtl") {
          setState(State::kRtl);
        } else {
          setState(State::kHold);
        }
      }
      return;
    }

    case State::kHold: {
      // 保持在轨迹终点
      if (!hold_sent_) {
        RCLCPP_INFO(get_logger(), "holding at trajectory end point");
        hold_sent_ = true;
        publishFinished();
      }
      const auto p = sampleTrajectory(traj_duration_);
      publishTrajectorySetpoint(p[0], p[1], p[2], NAN);
      return;
    }

    case State::kRtl: {
      if (!rtl_sent_) {
        sendRtl();
        rtl_sent_ = true;
      }
      // 交给 PX4 自动驾驶，仍持续发心跳避免意外
      return;
    }

    case State::kFailsafe: {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "FAILSAFE: node is not controlling the vehicle");
      return;
    }
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
    rclcpp::spin(std::make_shared<drones_pkg::TrajectoryFollower>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_follower"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
