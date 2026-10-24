// uav_link_visualizer.hpp
//
// 静态可视化节点：订阅各架无人机的实时位置，在 RViz 里给每架打一个点，
// 再把所有点两两连起来（完全图 K_n：n 架 => n 个点 + C(n,2) 条边）。
//
// 「静态」的含义：每架只取启动后收到的第一帧有效位置（快照），之后不再
// 更新，点固定在那里；节点周期性重发这份快照。飞机随后飞走，点也不动。
//
// 它不参与控制链路：只订阅位置话题、不发布任何指令，可以随时启动/退出，
// 不影响飞行。
//
// 位置来源与 trajectory_visualizer 一致：
//   <ns>/out/vehicle_local_position_v1（px4_msgs/VehicleLocalPosition）
// 内部按 NED 接收，写入 RViz 时映射到 ENU（与 trajectory_visualizer /
// obstacle_visualizer 保持同一可视坐标系）。
//
// Publishes:
//   ~/markers   visualization_msgs/MarkerArray
//               点（SPHERE）与连线（LINE_LIST）合在一条话题里，
//               ns 分别为 "uav_points" / "uav_links"。
//               四架的快照还未集齐时不发任何边，避免拉出到原点的假线。
//
// 参数:
//   uav_namespaces   string 数组，各架 PX4 实例命名空间
//                    （如 ["/fmu", "/px4_1/fmu", "/px4_2/fmu", "/px4_3/fmu"]）
//   frame_id         可视坐标系，默认 "map"
//   point_altitude   点与连线统一所在的高度 [m]（ENU 的 z），默认 5.0
//                    不使用飞机的实时高度，四点都被钉在同一水平面
//   point_size       点大小 [m]（SPHERE 的 scale.x）
//   line_width       连线线宽 [m]（LINE_LIST 的 scale.x）
//   connect_all      true 全连接 K_n；false 只连相邻序号 (0-1, 1-2, ...)
//   link_color_r/g/b, link_alpha   连线颜色与透明度
//   point_alpha      点的透明度（颜色按序号自动分配）
//   snapshot_timeout_s  等待快照的超时 [s]；超时后仍缺的点不再等，
//                       有几架画几架（0 表示无限等待）。默认 5.0
//   publish_rate_hz  重发频率 [Hz]，默认 1.0

#ifndef DRONES_PKG__UAV_LINK_VISUALIZER_HPP_
#define DRONES_PKG__UAV_LINK_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace drones_pkg
{

class UavLinkVisualizer : public rclcpp::Node
{
public:
  explicit UavLinkVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// 单架的快照状态：订阅 + 是否已锁定位置。
  struct UavSnapshot
  {
    std::string ns;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub;
    double x{0.0};      ///< RViz 可视坐标 (East)
    double y{0.0};      ///< RViz 可视坐标 (North)
    bool captured{false};
  };

  void declareParameters();

  /// 收到位置消息：若该架尚未锁定且位置有效，则记录一次快照并尝试
  /// 凑齐所有架；集齐后构建 markers_。
  void onPosition(
    size_t index, const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  /// 依据已锁定的快照生成点与连线 marker。只在全部（或超时后部分）
  /// 快照就绪时调用。
  void buildMarkers();

  static std::string sanitizeName(const std::string & ns);

  /// NED 水平面 -> RViz 可视坐标 (East, North)。高度不从这里取，
  /// 统一用 point_altitude_。
  void toRvizFrame(double ned_x, double ned_y, double & out_x, double & out_y) const;

  // ---- 参数 ----
  std::vector<std::string> uav_namespaces_{"/fmu"};
  std::string frame_id_{"map"};
  double point_altitude_{5.0};
  double point_size_{0.4};
  double line_width_{0.08};
  bool connect_all_{true};

  double point_alpha_{1.0};
  double link_color_r_{0.90};
  double link_color_g_{0.90};
  double link_color_b_{0.95};
  double link_alpha_{0.85};

  double snapshot_timeout_s_{5.0};
  double publish_rate_hz_{1.0};

  // ---- 状态 ----
  std::vector<UavSnapshot> uavs_;
  size_t captured_count_{0};
  bool ready_{false};        ///< 快照集齐（或超时）后置位，才开始发 markers
  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr capture_timer_;  ///< 超时兜底
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__UAV_LINK_VISUALIZER_HPP_
