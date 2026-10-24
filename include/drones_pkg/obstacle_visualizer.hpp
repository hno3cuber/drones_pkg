// obstacle_visualizer.hpp
//
// 静态可视化节点：圆柱障碍物 + 一条来自 csv 的参考轨迹线。
//
// 它不参与控制链路：不订阅任何 PX4 话题，也不发布任何指令，只往 RViz 发
// MarkerArray，因此可以随时启动或退出，不影响飞行。
//
// Publishes:
//   ~/markers          visualization_msgs/MarkerArray   圆柱障碍物（transient_local）
//   ~/reference_path   visualization_msgs/MarkerArray   参考轨迹线（transient_local）
//                     两条话题独立，RViz 里可分别 Add / 移除。
//
// 参数:
//   frame_id             可视坐标系，默认 "map"
//   obstacles            障碍物圆心位置，扁平数组 [x0, y0, x1, y1, ...]（ENU）
//   obstacle_radius      圆柱半径 [m]，默认 0.5
//   obstacle_height      圆柱高度 [m]，默认 5.0（底面贴地，即 z ∈ [0, height]）
//   publish_rate_hz      重发频率 [Hz]，默认 1.0
//   color_r/g/b, alpha   障碍物颜色与透明度
//   path_file            参考轨迹 csv 路径（两行 x/y），留空则不画
//   path_altitude        轨迹线所在高度 [m]，默认 0（贴地）
//   path_line_width      线宽 [m]，默认 0.15
//   path_color_r/g/b, path_alpha  轨迹线颜色与透明度

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

  /// 依据 obstacles_ / radius_ / height_ 生成障碍物 marker，
  /// 依据 path_points_ 生成轨迹线 marker（分别存入 markers_ 和 path_markers_）。
  /// 内容都是静态的，因此只在启动时构建一次，之后周期重发同一份数据。
  void buildMarkers();

  /// 读取两行 x/y 格式的 csv（第一行 x，第二行 y），逐列配对为点。
  /// 解析失败或行列数不一致时抛异常，由 main 捕获后退出。
  static std::vector<std::pair<double, double>> loadTwoRowCsv(const std::string & file);

  /// 本包 share 目录下的默认轨迹文件 src/ref.csv 绝对路径；拿不到时返回空串。
  std::string defaultPackageCsvPath();

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

  // ---- 参考轨迹线参数 ----
  std::string path_file_;
  double path_altitude_{0.0};
  double path_line_width_{0.15};
  double path_color_r_{0.20};
  double path_color_g_{0.65};
  double path_color_b_{1.00};
  double path_alpha_{1.00};

  // ---- 状态 ----
  /// 参考轨迹点 (x, y)，由 path_file_ 解析得到；为空则不画线
  std::vector<std::pair<double, double>> path_points_;
  /// 障碍物 marker（~/markers）
  visualization_msgs::msg::MarkerArray markers_;
  /// 参考轨迹线 marker（~/reference_path）
  visualization_msgs::msg::MarkerArray path_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__OBSTACLE_VISUALIZER_HPP_
