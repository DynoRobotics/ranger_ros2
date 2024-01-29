#ifndef GAZEBO_RANGER_ROBOT_HPP
#define GAZEBO_RANGER_ROBOT_HPP

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <ranger_msgs/msg/motion_state.hpp>
#include <ranger_msgs/msg/actuator_state.hpp>
#include <ranger_msgs/msg/driver_state.hpp>
#include <ranger_msgs/msg/motor_state.hpp>

#include "ranger_base/ranger_params.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

namespace westonrobot {
class GazeboRangerRobot : public RangerRobot {
 public:
  GazeboRangerRobot(std::shared_ptr<rclcpp::Node> node);
  ~GazeboRangerRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;
  std::string RequestVersion(int timeout_sec) override;

  // functions to be implemented by each robot class
  void ResetRobotState() override;

  void DisableLightControl() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // robot control
  void SetMotionMode(uint8_t mode) override;
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double angular_vel = 0.0) override;
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                       AgxLightMode r_mode, uint8_t r_value) override;

  // get robot state
  RangerCoreState GetRobotState() override;
  RangerActuatorState GetActuatorState() override;
  RangerCommonSensorState GetCommonSensorState() override;

  // alias to use template instance with default allocator
  using MotionState = ranger_msgs::msg::MotionState_<std::allocator<void>>;

 private:
 std::shared_ptr<rclcpp::Node> node_;

 RangerCoreState ranger_state_;
 RangerActuatorState ranger_actuator_;
 RangerCommonSensorState ranger_common_sensor_;
 uint8_t motion_mode_;

 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_joint_state_pub;
 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_joint_state_pub;
};
}  // namespace westonrobot

#endif /* GAZEBO_RANGER_ROBOT_HPP */
