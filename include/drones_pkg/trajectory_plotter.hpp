// trajectory_plotter.hpp
//
// 观测型节点：在跟踪任务进行时一路累积「无人机到各障碍物中轴线的距离」，
// 等 trajectory_follower 宣布到达终点后，用 matplotlibcpp 弹一张曲线图。
//
// 它不参与控制，只订阅三样东西:
//   <node_namespace>/out/vehicle_local_position_v1   位置（算距离用）
//   <obstacles_topic>                                障碍物 MarkerArray
//   <node_namespace>/trajectory_finished             到点信号（订阅到即绘图）
//
// 绘制：三条距离曲线共用一张图，横轴为任务时间，另加一条 y=安全距离 的红色
// 虚线。分隔线用 matplotlib 的 axhline 绘制，因此横向贯通整幅图。
//
// 注意: matplotlibcpp 是阻塞式的（show() 会一直等到窗口关闭）。因此绘图放在
// 独立线程里执行，避免阻塞 ROS 回调；同时用 backend("TkAgg") 之类可交互后端
// 时需要有显示环境（DISPLAY），无头环境请改用 "Agg" 并只看保存的 png。

#ifndef DRONES_PKG__TRAJECTORY_PLOTTER_HPP_
#define DRONES_PKG__TRAJECTORY_PLOTTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace drones_pkg
{

class TrajectoryPlotter : public rclcpp::Node
{
public:
  explicit TrajectoryPlotter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// 一根圆柱的中轴线：水平圆心 + z 区间（与 obstacle_distance_monitor 同约定）
  struct Axis
  {
    double x{0.0};
    double y{0.0};
    double z_min{0.0};
    double z_max{0.0};
  };

  void declareParameters();

  static std::string sanitizeName(const std::string & ns);
  bool parseFlatAxes(const std::vector<double> & flat, std::vector<Axis> & out);
  void loadFallbackAxes();
  bool rebuildFromMarkers(const visualization_msgs::msg::MarkerArray & msg);

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void onFinished(const std_msgs::msg::Bool::SharedPtr msg);

  /// 画图 + 弹窗。内部会先 mark 成已绘制，保证只画一次。
  void plotAndShow();

  // ---- 参数 ----
  std::string node_namespace_{"/fmu"};
  std::string obstacles_topic_;
  std::string output_png_;          // 非空则同时保存图片
  double safe_distance_{0.75};      // 红色虚线的 y 值
  double compute_rate_hz_{20.0};
  bool drone_z_valid_{true};
  std::string backend_{"TkAgg"};    // 交互弹窗后端；无头环境可用 "Agg"
  std::vector<double> fallback_flat_;

  // ---- 障碍物几何 ----
  std::vector<Axis> axes_;
  std::vector<Axis> fallback_axes_;

  // ---- 累计数据（时间, 每根柱子的距离）----
  std::vector<double> t_;
  std::vector<std::vector<double>> dist_;   // dist_[i] 对应 axes_[i]
  double t0_{0.0};
  bool t0_set_{false};

  // ---- 无人机位置 ----
  bool has_position_{false};
  double pos_x_{0.0};
  double pos_y_{0.0};
  double pos_z_{0.0};

  // 保护累计序列与 axes_，回调与绘图线程可能并发访问
  std::mutex mtx_;
  bool plotted_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_finished_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_PLOTTER_HPP_
