// uav_link_visualizer.cpp
//
// 订阅各架无人机的实时位置，给每架打一个点，再把点两两连起来。
//
// 设计要点:
//   1. 「静态」= 每架只取启动后第一帧有效位置作快照，之后不再更新。
//      飞机随后飞走，点固定不动；节点周期性重发这份快照。
//   2. 不参与控制链路：只订阅位置、不发任何指令，可随时起停。
//   3. 位置来源与 trajectory_visualizer 一致：
//        <ns>/out/vehicle_local_position_v1
//      只取其中的水平位置 (x, y)，按 NED 接收、写入 RViz 时映射到 ENU；
//      高度不用实时值，统一钉在 point_altitude 这一层。
//   4. 快照未集齐时不发任何连线：只要有一架没锁定，就不画边，
//      避免出现「从已有点拉到原点」的假线。四架齐了才一次性出全部点+边。
//   5. 点与线合在一条话题 ~/markers，用 namespace 区分：
//        ns = "uav_points"  n 个 SPHERE，每架一色
//        ns = "uav_links"   一个 LINE_LIST，若干条边共用一份颜色
//   6. 连线的边用 LINE_LIST：一个 marker 装下所有边（每两点一段）。
//   7. 若迟迟收不到某架位置，snapshot_timeout_s 后不再等它，
//      有几架画几架，避免整个节点卡在等待上。

#include "drones_pkg/uav_link_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drones_pkg
{

namespace
{
// 与 trajectory_visualizer.cpp 共用同一话题后缀与 QoS 约定
constexpr char kLocalPositionSuffix[] = "/out/vehicle_local_position_v1";

constexpr char kPointNamespace[] = "uav_points";
constexpr char kLinkNamespace[] = "uav_links";

// 按序号循环取用的点颜色，便于在 RViz 里区分各架无人机。
struct Rgb
{
  double r;
  double g;
  double b;
};
constexpr Rgb kPointPalette[] = {
  {0.90, 0.20, 0.20},  // uav0 红
  {0.20, 0.85, 0.30},  // uav1 绿
  {0.25, 0.55, 1.00},  // uav2 蓝
  {0.95, 0.80, 0.20},  // uav3 黄
  {0.80, 0.35, 0.90},  // uav4 紫
  {0.20, 0.85, 0.85},  // uav5 青
};
constexpr size_t kPaletteSize = sizeof(kPointPalette) / sizeof(kPointPalette[0]);

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
UavLinkVisualizer::UavLinkVisualizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("uav_link_visualizer", options)
{
  declareParameters();

  // transient_local + KeepLast(1)：RViz 后启动也能拿到最近一次的内容
  pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/markers", rclcpp::QoS(1).transient_local());

  // 位置订阅的 QoS 与 px4_ros_com / trajectory_visualizer 一致
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();

  uavs_.reserve(uav_namespaces_.size());
  for (size_t i = 0; i < uav_namespaces_.size(); ++i) {
    UavSnapshot s;
    s.ns = uav_namespaces_[i];
    s.sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      s.ns + kLocalPositionSuffix, qos,
      [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onPosition(i, msg);
      });
    uavs_.push_back(std::move(s));
  }

  // 周期性重发快照。已就绪时重发 markers_；未就绪则什么都不发。
  const double period = 1.0 / std::clamp(publish_rate_hz_, 0.1, 10.0);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period)),
    [this]() {
      if (ready_) {
        pub_->publish(markers_);
      }
    });

  // 超时兜底：到点仍未集齐就有几架画几架
  if (snapshot_timeout_s_ > 0.0) {
    capture_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(snapshot_timeout_s_)),
      [this]() {
        if (ready_) {
          return;
        }
        ready_ = true;
        capture_timer_->cancel();
        buildMarkers();
        pub_->publish(markers_);
        RCLCPP_WARN(
          get_logger(), "snapshot timeout after %.1fs: %zu/%zu uav(s) captured",
          snapshot_timeout_s_, captured_count_, uavs_.size());
        for (const auto & s : uavs_) {
          if (!s.captured) {
            RCLCPP_WARN(get_logger(), "  missing: %s", s.ns.c_str());
          }
        }
      });
  }

  RCLCPP_INFO(
    get_logger(), "uav_link_visualizer started: %zu uav(s), snapshot mode, "
    "connect_all=%s, frame=%s, timeout=%.1fs",
    uavs_.size(), connect_all_ ? "true" : "false", frame_id_.c_str(),
    snapshot_timeout_s_);
  for (const auto & s : uavs_) {
    RCLCPP_INFO(get_logger(), "  waiting: %s%s", s.ns.c_str(), kLocalPositionSuffix);
  }
}

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
void UavLinkVisualizer::declareParameters()
{
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

  // 多机时由 launch 传入完整命名空间列表；默认单机 /fmu
  uav_namespaces_ = declare_parameter<std::vector<std::string>>(
    "uav_namespaces", std::vector<std::string>{"/fmu"});

  point_altitude_ = declare_parameter<double>("point_altitude", 5.0);

  point_size_ = declare_parameter<double>("point_size", 0.4);
  line_width_ = declare_parameter<double>("line_width", 0.08);
  connect_all_ = declare_parameter<bool>("connect_all", true);

  point_alpha_ = declare_parameter<double>("point_alpha", 1.0);
  link_color_r_ = declare_parameter<double>("link_color_r", 0.90);
  link_color_g_ = declare_parameter<double>("link_color_g", 0.90);
  link_color_b_ = declare_parameter<double>("link_color_b", 0.95);
  link_alpha_ = declare_parameter<double>("link_alpha", 0.85);

  snapshot_timeout_s_ = declare_parameter<double>("snapshot_timeout_s", 5.0);

  point_size_ = std::max(point_size_, 1e-3);
  line_width_ = std::max(line_width_, 1e-4);
  point_alpha_ = std::clamp(point_alpha_, 0.0, 1.0);
  link_alpha_ = std::clamp(link_alpha_, 0.0, 1.0);

  if (uav_namespaces_.empty()) {
    throw std::runtime_error("参数 uav_namespaces 不能为空");
  }
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
std::string UavLinkVisualizer::sanitizeName(const std::string & ns)
{
  // "/px4_1/fmu" -> "px4_1_fmu"：去掉前导 '/'，其余 '/' 换成 '_'
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

void UavLinkVisualizer::toRvizFrame(
  double ned_x, double ned_y, double & out_x, double & out_y) const
{
  // NED (North, East) -> ENU (East, North)；高度另行由 point_altitude_ 给出
  out_x = ned_y;
  out_y = ned_x;
}

// ---------------------------------------------------------------------------
// 位置回调：锁定第一帧有效位置
// ---------------------------------------------------------------------------
void UavLinkVisualizer::onPosition(
  size_t index, const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  if (index >= uavs_.size()) {
    return;
  }
  // 已锁定、或整体已就绪，就不再更新（快照语义）
  if (ready_ || uavs_[index].captured) {
    return;
  }
  if (!msg->xy_valid) {
    return;
  }

  toRvizFrame(
    msg->x, msg->y, uavs_[index].x, uavs_[index].y);
  uavs_[index].captured = true;
  ++captured_count_;

  RCLCPP_INFO(
    get_logger(), "captured %s at ENU (%.2f, %.2f) @ z=%.2f",
    uavs_[index].ns.c_str(),
    uavs_[index].x, uavs_[index].y, point_altitude_);

  // 集齐即出图
  if (captured_count_ == uavs_.size()) {
    ready_ = true;
    if (capture_timer_) {
      capture_timer_->cancel();
    }
    buildMarkers();
    pub_->publish(markers_);
    RCLCPP_INFO(
      get_logger(), "all %zu uav(s) captured, %zu link(s) drawn",
      uavs_.size(), connect_all_
      ? (uavs_.size() * (uavs_.size() - 1)) / 2
      : (uavs_.size() > 1 ? uavs_.size() - 1 : 0));
  }
}

// ---------------------------------------------------------------------------
// 构建 marker 数组
// ---------------------------------------------------------------------------
void UavLinkVisualizer::buildMarkers()
{
  markers_.markers.clear();

  const auto stamp = now();
  const auto forever = rclcpp::Duration::from_seconds(0.0);

  // 收集已锁定的点的序号，未锁定的（超时情形）不参与画点与连线
  std::vector<size_t> idx;
  idx.reserve(uavs_.size());
  for (size_t i = 0; i < uavs_.size(); ++i) {
    if (uavs_[i].captured) {
      idx.push_back(i);
    }
  }

  // ---- 点：每架一个 SPHERE ----
  for (size_t k = 0; k < idx.size(); ++k) {
    const UavSnapshot & s = uavs_[idx[k]];
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.ns = kPointNamespace;
    m.id = static_cast<int>(idx[k]);   // id 用原序号，颜色与身份一一对应
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position = makePoint(s.x, s.y, point_altitude_);
    m.pose.orientation.w = 1.0;

    m.scale.x = point_size_;
    m.scale.y = point_size_;
    m.scale.z = point_size_;

    const Rgb & c = kPointPalette[idx[k] % kPaletteSize];
    m.color.r = static_cast<float>(c.r);
    m.color.g = static_cast<float>(c.g);
    m.color.b = static_cast<float>(c.b);
    m.color.a = static_cast<float>(point_alpha_);

    m.lifetime = forever;
    markers_.markers.push_back(std::move(m));
  }

  // ---- 连线：一个 LINE_LIST ----
  if (idx.size() >= 2) {
    visualization_msgs::msg::Marker link;
    link.header.stamp = stamp;
    link.header.frame_id = frame_id_;
    link.ns = kLinkNamespace;
    link.id = 0;
    link.type = visualization_msgs::msg::Marker::LINE_LIST;
    link.action = visualization_msgs::msg::Marker::ADD;
    link.scale.x = line_width_;
    link.pose.orientation.w = 1.0;
    link.color.r = static_cast<float>(link_color_r_);
    link.color.g = static_cast<float>(link_color_g_);
    link.color.b = static_cast<float>(link_color_b_);
    link.color.a = static_cast<float>(link_alpha_);
    link.lifetime = forever;

    auto point_at = [this](size_t i) {
        const UavSnapshot & s = uavs_[i];
        return makePoint(s.x, s.y, point_altitude_);
      };

    if (connect_all_) {
      // 全连接：i < j 的所有组合
      for (size_t a = 0; a < idx.size(); ++a) {
        for (size_t b = a + 1; b < idx.size(); ++b) {
          link.points.push_back(point_at(idx[a]));
          link.points.push_back(point_at(idx[b]));
        }
      }
    } else {
      // 链式：只连相邻序号
      for (size_t a = 0; a + 1 < idx.size(); ++a) {
        link.points.push_back(point_at(idx[a]));
        link.points.push_back(point_at(idx[a + 1]));
      }
    }

    markers_.markers.push_back(std::move(link));
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
    rclcpp::spin(std::make_shared<drones_pkg::UavLinkVisualizer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("uav_link_visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
