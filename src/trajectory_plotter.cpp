// trajectory_plotter.cpp
//
// 跟踪任务结束后弹一张三条距离曲线 + 安全距离虚线的图。
//
// 流程:
//   1. 定时器按 compute_rate_hz 采样无人机到每根障碍物中轴线的距离，累积成序列
//   2. 订阅 <node_namespace>/trajectory_finished；收到 true 就在独立线程里绘图
//      （matplotlibcpp 的 show() 是阻塞的，不能占住 ROS 执行器）
//   3. 图中三条曲线共用一张图，横轴为任务时间，另加 y=safe_distance 红色虚线
//
// 与 obstacle_distance_monitor 的关系：两者用同一套「到中轴线距离」定义，但这
// 个节点不发话题，只在内存里累积、最后画出来，属于纯离线观测。

#include "drones_pkg/trajectory_plotter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "drones_pkg/matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace drones_pkg
{

namespace
{
constexpr char kObstacleNamespace[] = "obstacles";
}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------
TrajectoryPlotter::TrajectoryPlotter(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_plotter", options)
{
  declareParameters();

  const std::string ns = node_namespace_;

  // 位置：与 trajectory_follower / trajectory_visualizer 一致，BEST_EFFORT
  rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
  px4_qos.best_effort().durability_volatile();
  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    ns + "/out/vehicle_local_position_v1", px4_qos,
    std::bind(&TrajectoryPlotter::onLocalPosition, this, std::placeholders::_1));

  // 障碍物几何：优先从 obstacle_visualizer 的 MarkerArray 读取
  rclcpp::QoS marker_qos(rclcpp::KeepLast(1));
  marker_qos.transient_local();
  sub_markers_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    obstacles_topic_, marker_qos,
    std::bind(&TrajectoryPlotter::onMarkers, this, std::placeholders::_1));

  // 到点信号：follower 用 transient_local 发布，晚启动也能收到
  rclcpp::QoS finished_qos(rclcpp::KeepLast(1));
  finished_qos.transient_local();
  sub_finished_ = create_subscription<std_msgs::msg::Bool>(
    ns + "/trajectory_finished", finished_qos,
    std::bind(&TrajectoryPlotter::onFinished, this, std::placeholders::_1));

  // 兜底几何建好累计序列，MarkerArray 一到再按实际数量重建
  axes_ = fallback_axes_;
  dist_.resize(axes_.size());

  const double period = 1.0 / std::clamp(compute_rate_hz_, 1.0, 200.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    [this]() {
      if (!has_position_) {return;}

      const rclcpp::Time now_t = now();
      if (!t0_set_) {
        t0_ = now_t.seconds();
        t0_set_ = true;
      }

      std::lock_guard<std::mutex> lock(mtx_);
      t_.push_back(now_t.seconds() - t0_);
      for (size_t i = 0; i < axes_.size(); ++i) {
        if (i >= dist_.size()) {dist_.resize(axes_.size());}
        const Axis & a = axes_[i];
        const double d_xy = std::hypot(pos_x_ - a.x, pos_y_ - a.y);
        double v_out = 0.0;
        if (pos_z_ < a.z_min) {
          v_out = a.z_min - pos_z_;
        } else if (pos_z_ > a.z_max) {
          v_out = pos_z_ - a.z_max;
        }
        dist_[i].push_back(std::hypot(d_xy, v_out));
      }
    });

  RCLCPP_INFO(
    get_logger(), "trajectory_plotter started: uav ns=%s, obstacles=%zu, "
    "safe_distance=%.2fm, waiting for '%s/trajectory_finished'",
    ns.c_str(), axes_.size(), safe_distance_, ns.c_str());
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void TrajectoryPlotter::declareParameters()
{
  node_namespace_ = declare_parameter<std::string>("node_namespace", "/fmu");
  obstacles_topic_ = declare_parameter<std::string>(
    "obstacles_topic", "/obstacle_visualizer/markers");
  output_png_ = declare_parameter<std::string>("output_png", "");
  safe_distance_ = declare_parameter<double>("safe_distance", 0.75);
  compute_rate_hz_ = declare_parameter<double>("compute_rate_hz", 20.0);
  drone_z_valid_ = declare_parameter<bool>("drone_z_valid", true);
  backend_ = declare_parameter<std::string>("backend", "TkAgg");
  fallback_flat_ = declare_parameter<std::vector<double>>(
    "fallback_obstacles", std::vector<double>{4.0, 4.0, 7.0, 7.0, 15.0, 4.0});

  loadFallbackAxes();
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
std::string TrajectoryPlotter::sanitizeName(const std::string & ns)
{
  std::string out;
  out.reserve(ns.size());
  for (char c : ns) {
    if (c == '/') {
      if (out.empty()) {continue;}
      out.push_back('_');
    } else {
      out.push_back(c);
    }
  }
  return out;
}

// (sanitizeName 保留给后续按 uav 命名输出文件用，当前未用到此处)

bool TrajectoryPlotter::parseFlatAxes(
  const std::vector<double> & flat, std::vector<Axis> & out)
{
  out.clear();
  if (flat.size() % 2 != 0) {
    RCLCPP_ERROR(
      get_logger(), "fallback_obstacles 必须是偶数个元素 [x0, y0, x1, y1, ...]");
    return false;
  }
  // 兜底高度取与 obstacle_visualizer 一致的默认值 5 m
  constexpr double kFallbackHeight = 5.0;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    Axis a;
    a.x = flat[i];
    a.y = flat[i + 1];
    a.z_min = 0.0;
    a.z_max = kFallbackHeight;
    out.push_back(a);
  }
  return true;
}

void TrajectoryPlotter::loadFallbackAxes()
{
  fallback_axes_.clear();
  if (!parseFlatAxes(fallback_flat_, fallback_axes_)) {
    RCLCPP_WARN(get_logger(), "fallback_obstacles 解析失败，等待 MarkerArray");
  }
}

bool TrajectoryPlotter::rebuildFromMarkers(
  const visualization_msgs::msg::MarkerArray & msg)
{
  std::vector<Axis> parsed;
  for (const auto & m : msg.markers) {
    if (m.ns != kObstacleNamespace) {continue;}
    if (m.type != visualization_msgs::msg::Marker::CYLINDER) {continue;}
    if (m.action == visualization_msgs::msg::Marker::DELETE ||
      m.action == visualization_msgs::msg::Marker::DELETEALL)
    {
      continue;
    }
    Axis a;
    a.x = m.pose.position.x;
    a.y = m.pose.position.y;
    const double half = std::abs(m.scale.z) * 0.5;
    a.z_min = m.pose.position.z - half;
    a.z_max = m.pose.position.z + half;
    parsed.push_back(a);
  }
  if (parsed.empty()) {return false;}

  if (parsed.size() == axes_.size()) {
    bool same = true;
    for (size_t i = 0; i < parsed.size(); ++i) {
      if (std::abs(parsed[i].x - axes_[i].x) > 1e-6 ||
        std::abs(parsed[i].y - axes_[i].y) > 1e-6)
      {
        same = false;
        break;
      }
    }
    if (same) {return false;}
  }

  axes_ = parsed;
  dist_.assign(axes_.size(), {});
  RCLCPP_INFO(
    get_logger(), "obstacles updated from MarkerArray: %zu axis/axes", axes_.size());
  return true;
}

// ---------------------------------------------------------------------------
// 回调
// ---------------------------------------------------------------------------
void TrajectoryPlotter::onLocalPosition(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  if (!msg->xy_valid) {return;}
  if (drone_z_valid_ && !msg->z_valid) {return;}

  // PX4 local NED -> ENU
  pos_x_ = msg->y;
  pos_y_ = msg->x;
  pos_z_ = -static_cast<double>(msg->z);
  has_position_ = true;
}

void TrajectoryPlotter::onMarkers(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  rebuildFromMarkers(*msg);
}

void TrajectoryPlotter::onFinished(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {return;}

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (plotted_) {return;}
    plotted_ = true;   // 先占位，避免重复触发绘图
  }

  RCLCPP_INFO(get_logger(), "收到到点信号，开始绘图（%zu 个采样点）", t_.size());

  // show() 是阻塞的，放到独立线程里，别占住 ROS 执行器
  std::thread([this]() { plotAndShow(); }).detach();
}

// ---------------------------------------------------------------------------
// 绘图
// ---------------------------------------------------------------------------
void TrajectoryPlotter::plotAndShow()
{
  // 深拷贝一份，绘图期间不再持锁
  std::vector<double> t;
  std::vector<std::vector<double>> d;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    t = t_;
    d = dist_;
  }

  if (t.empty()) {
    RCLCPP_WARN(get_logger(), "没有累积到任何距离数据，跳过绘图");
    return;
  }

  try {
    if (!backend_.empty()) {
      plt::backend(backend_);
    }

    plt::figure_size(1200, 700);

    // 三条距离曲线：标签写成障碍物编号 + 圆心坐标，图例里一眼能对上
    for (size_t i = 0; i < axes_.size(); ++i) {
      if (i >= d.size() || d[i].size() != t.size()) {continue;}
      char label[64];
      std::snprintf(
        label, sizeof(label), "obstacle%zu (%.0f, %.0f)", i, axes_[i].x, axes_[i].y);
      plt::named_plot(std::string(label), t, d[i]);
    }

    // 安全距离：横向贯通整幅图的红色虚线
    plt::axhline(
      safe_distance_, 0.0, 1.0,
      {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1.5"},
        {"label", "safe distance"}});

    plt::xlabel("time [s]");
    plt::ylabel("distance to axis [m]");
    plt::title("UAV to obstacle axis distances");
    plt::grid(true);
    plt::legend();

    if (!output_png_.empty()) {
      plt::save(output_png_);
      RCLCPP_INFO(get_logger(), "图已保存到 %s", output_png_.c_str());
    }

    plt::show();   // 阻塞，直到窗口关闭
    plt::close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "绘图失败: %s", e.what());
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
    rclcpp::spin(std::make_shared<drones_pkg::TrajectoryPlotter>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_plotter"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
