
#include "ranger_base/gazebo_ranger_robot.hpp"

#include <iostream>

namespace westonrobot {

GazeboRangerRobot::GazeboRangerRobot(std::shared_ptr<rclcpp::Node> node) : RangerRobot(false)
{   
    node_ = node;
    // Publishers
    arm_joint_state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_joints_controller/commands", 10);

    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"), "Constructing GazeboRangerRobot");
}

GazeboRangerRobot::~GazeboRangerRobot()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"), "Destructing GazeboRangerRobot");
}

bool GazeboRangerRobot::Connect(std::string can_name)
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"Gazebo robot, no need to connect");
    return true;
}

void GazeboRangerRobot::EnableCommandedMode()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::EnableCommandedMode");
}

std::string GazeboRangerRobot::RequestVersion(int timeout_sec)
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::RequestVersion");
    return "GazeboRangerRobot::RequestVersion";
}

// functions to be implemented by each robot class
void GazeboRangerRobot::ResetRobotState()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::ResetRobotState");
}

void GazeboRangerRobot::DisableLightControl() 
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::DisableLightControl");
}

ProtocolVersion GazeboRangerRobot::GetParserProtocolVersion()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::GetParserProtocolVersion");
    return ProtocolVersion::AGX_V2;
}

// robot control
void GazeboRangerRobot::SetMotionMode(uint8_t mode)
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetMotionMode to %d", mode);
    motion_mode_ = mode;
    std_msgs::msg::Float64MultiArray arm_joint_state;
    //set the arm joint positions here
    switch (motion_mode_)
    {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN:
        arm_joint_state.data = {0, 0, 0, 0};
        arm_joint_state_pub->publish(arm_joint_state);
        break;
    case MotionState::MOTION_MODE_PARALLEL:
        arm_joint_state.data = {0, 0, 0, 0}; // TODO(Chris): set the arm joint positions here
        arm_joint_state_pub->publish(arm_joint_state);
        break;
    case MotionState::MOTION_MODE_SPINNING:
        arm_joint_state.data = {-M_PI/2, M_PI/2, M_PI/2, -M_PI/2};
        arm_joint_state_pub->publish(arm_joint_state);
        break;
    default:
        break;
    }
}

void GazeboRangerRobot::SetMotionCommand(double linear_vel, double steer_angle,
                    double angular_vel)
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetMotionCommand");
    // set the velocity for all four wheels here
    // set position for all four arms here
    msg.body.motion_command_msg.linear_velocity = linear_vel;
    msg.body.motion_command_msg.angular_velocity = angular_vel;
    msg.body.motion_command_msg.lateral_velocity = lateral_vel;
    msg.body.motion_command_msg.steering_angle = steering_angle;
}

void GazeboRangerRobot::SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                    AgxLightMode r_mode, uint8_t r_value)
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetLightCommand");
}

// get robot state
RangerCoreState GazeboRangerRobot::GetRobotState()
{
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::GetRobotState");

    // get the state from gazebo, or do this elsewhere and just return here

    return ranger_state_;
}

RangerActuatorState GazeboRangerRobot::GetActuatorState()
{
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::GetActuatorState");

    // RangerActuatorState ranger_actuator;
    // ranger_actuator.time_stamp = actuator.time_stamp;

    // ranger_actuator.motor_speeds.speed_1 = actuator.motor_speeds.speed_1;
    // ranger_actuator.motor_speeds.speed_2 = actuator.motor_speeds.speed_2;
    // ranger_actuator.motor_speeds.speed_3 = actuator.motor_speeds.speed_3;
    // ranger_actuator.motor_speeds.speed_4 = actuator.motor_speeds.speed_4;
    // ranger_actuator.motor_angles.angle_5 = actuator.motor_angles.angle_5;
    // ranger_actuator.motor_angles.angle_6 = actuator.motor_angles.angle_6;
    // ranger_actuator.motor_angles.angle_7 = actuator.motor_angles.angle_7;
    // ranger_actuator.motor_angles.angle_8 = actuator.motor_angles.angle_8;

    // for (int i = 0; i < 8; ++i) {
    //   ranger_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
    //   ranger_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    // }
    // return ranger_actuator;
    return RangerActuatorState();
}

RangerCommonSensorState GazeboRangerRobot::GetCommonSensorState()
{
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::GetCommonSensorState");
    // auto common_sensor =
    //     AgilexBase<ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    // RangerCommonSensorState ranger_bms;

    // ranger_bms.time_stamp = common_sensor.time_stamp;

    // ranger_bms.bms_basic_state.current = common_sensor.bms_basic_state.current;
    // // Note: BMS CAN message definition is not consistent across AgileX robots.
    // // Robots with steering mechanism should additionally divide the voltage by
    // // 10.
    // ranger_bms.bms_basic_state.voltage =
    //     common_sensor.bms_basic_state.voltage * 0.1f;
    // ranger_bms.bms_basic_state.battery_soc =
    //     common_sensor.bms_basic_state.battery_soc;
    // ranger_bms.bms_basic_state.battery_soh =
    //     common_sensor.bms_basic_state.battery_soh;
    // ranger_bms.bms_basic_state.temperature =
    //     common_sensor.bms_basic_state.temperature;

    // return ranger_bms;
    return RangerCommonSensorState();
}

}