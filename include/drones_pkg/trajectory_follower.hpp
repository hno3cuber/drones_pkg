// trajectory_follower.hpp
//
// 让 PX4 无人机按 csv 中的参考轨迹飞行（OFFBOARD 模式）。
//
// 状态机:
//   WAIT_SYNC      等待 PX4 时间同步 + EKF 位置有效
//   SEND_SETPOINT  先持续发一段 setpoint，让 PX4 接受 offboard 心跳
//   ARM            发送解锁指令
//   OFFBOARD       切换 OFFBOARD 模式
//   APPROACH       从当前位置飞向轨迹起点（含爬升到目标高度）
//   TRACK          逐点播放参考轨迹（可选：避开障碍安全圈）
//   HOLD           轨迹播放完毕，保持在终点
//   RTL            返航
//
// 坐标系约定:
//   输入 csv 默认按 ENU (x=East, y=North) 解释,
//   内部转换为 PX4 使用的 NED (x=North, y=East, z=Down)。

#ifndef DRONES_PKG__TRAJECTORY_FOLLOWER_HPP_
#define DRONES_PKG__TRAJECTORY_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace drones_pkg
{

enum class State
{
  kWaitSync,
  kSendSetpoint,
  kOffboard,
  kApproach,
  kTrack,
  kHold,
  kRtl,
  kFailsafe,
};

class TrajectoryFollower : public rclcpp::Node
{
public:
  explicit TrajectoryFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- 参数 ----
  void declareParameters();
  void loadParameters();

  // ---- 轨迹加载 ----
  bool loadTrajectory();
  /// 在轨迹上按时间查询位置（线性插值），返回 NED 位置
  std::array<float, 3> sampleTrajectory(double t_sec) const;

  // ---- 避障 ----
  /// 把整条轨迹中落入障碍安全圆内的连续段替换成沿圆周的弧（NED 平面）。
  /// 在轨迹加载后一次性调用，运行时不再做几何计算。返回被修正的段数。
  size_t applyObstacleAvoidance();

  /// 把当前 traj_x_/traj_y_（NED）转回 ENU 并写成两行 csv。
  /// 仅在参数 export_avoided_csv 非空时调用，用于离线固化处理结果。
  bool exportAvoidedCsv(const std::string & path) const;

  // ---- 状态机 ----
  void stepStateMachine();
  void setState(State s);
  const char * stateName(State s) const;

  // ---- 指令 ----
  void publishOffboardControlMode();
  void publishTrajectorySetpoint(float x, float y, float z, float yaw);
  void publishVehicleCommand(uint32_t command, float p1, float p2);
  void arm();
  void engageOffboardMode();
  void sendRtl();
  void sendHold();
  /// 轨迹播放完毕、进入终点保持时发布一次 true（latched，只发一次）
  void publishFinished();

  // ---- 回调 ----
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // ---- 工具 ----
  static uint64_t nowMicros();
  float distance2d(float x1, float y1, float x2, float y2) const;

  // ---- 参数成员 ----
  // PX4 实例命名空间，多机时分别为 /fmu、/px4_1/fmu、/px4_2/fmu ...
  std::string node_namespace_{"/fmu"};
  // 目标 PX4 系统 ID（MAV_SYS_ID），多机时分别为 1、2、3 ...
  int target_system_{1};
  std::string trajectory_file_;
  double dt_{0.1};
  bool frame_is_enu_{true};
  double flight_altitude_{5.0};
  double origin_offset_x_{0.0};
  double origin_offset_y_{0.0};
  double approach_speed_{2.0};
  double setpoint_rate_hz_{20.0};
  double approach_tolerance_{0.5};
  bool loop_trajectory_{false};
  std::string end_action_{"hold"};
  // 避障处理结果导出路径（ENU 两行 csv）。非空时在加载+避障完成后落盘，
  // 用于把"平移+绕行"离线固化成新轨迹文件。留空则完全不产生副作用。
  std::string export_avoided_csv_;

  // ---- 避障 ----
  // 安全圆按 ENU 解释（与 obstacle_visualizer 的参数保持一致），
  // 内部换算到 NED 后再与轨迹点比较。
  bool avoidance_enable_{true};
  // 障碍圆心扁平数组 [x0, y0, x1, y1, ...]（ENU）
  std::vector<double> avoidance_obstacles_;
  // 安全圆半径 [m]：以圆心为中心，轨迹点靠近到该半径内即被推出
  double avoidance_radius_{1.5};
  // 绕行弧走向："short" 走角度差小的一侧（默认，与原行为一致）/ "long" 走
  // 另一侧。可用 arc_side_by_obstacle_ 按障碍下标逐项覆盖。
  std::string arc_side_{"short"};
  // 按障碍下标指定弧向（如 ["short","short","long"]），元素数不足的
  // 障碍沿用 arc_side_；为空则全部用 arc_side_。
  std::vector<std::string> arc_side_by_obstacle_;

  // ---- 轨迹数据 (NED, 已转换) ----
  std::vector<float> traj_x_;   // North
  std::vector<float> traj_y_;   // East
  double traj_duration_{0.0};

  // ---- 运行状态 ----
  // 状态机计时统一使用 steady_clock，避免与 ROS 时钟源混用
  State state_{State::kWaitSync};
  std::chrono::steady_clock::time_point state_enter_time_;
  double track_elapsed_{0.0};
  bool offboard_engaged_{false};
  bool arm_sent_{false};
  bool rtl_sent_{false};
  bool hold_sent_{false};
  bool finished_published_{false};

  // ---- 最新遥测 ----
  px4_msgs::msg::VehicleLocalPosition::SharedPtr local_pos_;
  px4_msgs::msg::VehicleStatus::SharedPtr status_;

  // ---- ROS 实体 ----
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
  // 跟踪完成信号：进入终点保持后发一次 true，供外部绘图/记录节点订阅。
  // 用 transient_local（latched），晚启动的订阅者也能拿到这一次事件。
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_finished_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_FOLLOWER_HPP_
