// obstacle_visualizer.hpp
//
// 静态圆柱障碍物可视化节点。
//
// 它不参与控制链路：不订阅任何 PX4 话题，也不发布任何指令，只往 RViz 发
// MarkerArray，因此可以随时启动或退出，不影响飞行。
//
// Publishes:
//   ~/markers   visualization_msgs/MarkerArray   圆柱障碍物列表（transient_local）
//
// 参数:
//   frame_id          可视坐标系，默认 "map"
//   obstacles         障碍物圆心位置，扁平数组 [x0, y0, x1, y1, ...]（ENU）
//   obstacle_radius   圆柱半径 [m]，默认 0.5
//   obstacle_height   圆柱高度 [m]，默认 5.0（底面贴地，即 z ∈ [0, height]）
//   publish_rate_hz   重发频率 [Hz]，默认 1.0
//   color_r/g/b, alpha  颜色与透明度

#ifndef DRONES_PKG__OBSTACLE_VISUALIZER_HPP_
#define DRONES_PKG__OBSTACLE_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <utility>
#include <vector>

namespace drones_pkg
{

class ObstacleVisualizer : public rclcpp::Node
{
public:
  explicit ObstacleVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();

  /// 依据 obstacles_ / radius_ / height_ 生成一次完整的 MarkerArray。
  /// 障碍物是静态的，因此只在启动时构建一次，之后周期重发同一份数据。
  void buildMarkers();

  // ---- 参数 ----
  std::string frame_id_{"map"};
  /// 障碍物圆心 (x, y)，ENU 平面坐标 [m]
  std::vector<std::pair<double, double>> obstacles_;
  /// 参数读入时的扁平数组，仅 declareParameters 内部使用
  std::vector<double> obstacles_flat_;
  double radius_{0.5};
  double height_{5.0};
  double publish_rate_hz_{1.0};
  double color_r_{0.85};
  double color_g_{0.20};
  double color_b_{0.20};
  double alpha_{0.65};

  // ---- 状态 ----
  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__OBSTACLE_VISUALIZER_HPP_
