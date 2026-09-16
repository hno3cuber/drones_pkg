// trajectory_follower.hpp
//
// 让 PX4 无人机按 uav0.csv 中的参考轨迹飞行（OFFBOARD 模式）。
//
// 状态机:
//   WAIT_SYNC      等待 PX4 时间同步 + EKF 位置有效
//   SEND_SETPOINT  先持续发一段 setpoint，让 PX4 接受 offboard 心跳
//   ARM            发送解锁指令
//   OFFBOARD       切换 OFFBOARD 模式
//   APPROACH       从当前位置飞向轨迹起点（含爬升到目标高度）
//   TRACK          逐点播放参考轨迹
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

  // ---- 回调 ----
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // ---- 工具 ----
  static uint64_t nowMicros();
  float distance2d(float x1, float y1, float x2, float y2) const;

  // ---- 参数成员 ----
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

  // ---- 最新遥测 ----
  px4_msgs::msg::VehicleLocalPosition::SharedPtr local_pos_;
  px4_msgs::msg::VehicleStatus::SharedPtr status_;

  // ---- ROS 实体 ----
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drones_pkg

#endif  // DRONES_PKG__TRAJECTORY_FOLLOWER_HPP_
