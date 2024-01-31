
#include "ranger_base/gazebo_ranger_robot.hpp"

#include <iostream>

namespace westonrobot {

GazeboRangerRobot::GazeboRangerRobot(std::shared_ptr<rclcpp::Node> node) : RangerRobot(false)
{   
    node_ = node;
    RCLCPP_INFO(node_->get_logger(), "Constructing GazeboRangerRobot");

    // Publishers
    arm_joint_state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_joints_controller/commands", 10);
    wheel_joint_state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_joints_controller/commands", 10);

    // Set some state variables
    SystemStateMessage system_state;
    system_state.vehicle_state = AgxVehicleState::VEHICLE_STATE_NORMAL;
    system_state.control_mode = AgxControlMode::CONTROL_MODE_CAN;
    system_state.error_code = 0;
    system_state.battery_voltage = 12.0;
    ranger_state_.system_state = system_state;
}

GazeboRangerRobot::~GazeboRangerRobot()
{
    RCLCPP_INFO(node_->get_logger(), "Destructing GazeboRangerRobot");
}

bool GazeboRangerRobot::Connect(std::string can_name)
{
    RCLCPP_INFO(node_->get_logger(),"Gazebo robot, no CAN to connect to");
    (void)can_name;
    return true;
}

void GazeboRangerRobot::EnableCommandedMode() // TODO
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::EnableCommandedMode");
}

std::string GazeboRangerRobot::RequestVersion(int timeout_sec) // TODO
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::RequestVersion");
    (void)timeout_sec;
    return "GazeboRangerRobot::RequestVersion";
}

// functions to be implemented by each robot class
void GazeboRangerRobot::ResetRobotState() // TODO
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::ResetRobotState");
}

void GazeboRangerRobot::DisableLightControl() // TODO
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::DisableLightControl");
}

ProtocolVersion GazeboRangerRobot::GetParserProtocolVersion()
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::GetParserProtocolVersion");
    return ProtocolVersion::AGX_V2;
}

void GazeboRangerRobot::SetMotionMode(uint8_t mode)
{
    ranger_state_.motion_mode_state.motion_mode = mode;
}

void GazeboRangerRobot::SetMotionCommand(double linear_vel, double steer_angle,
                    double angular_vel)
{
    std_msgs::msg::Float64MultiArray arm_joint_state;
    std_msgs::msg::Float64MultiArray wheel_joint_state;

    // info log all the things
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetMotionCommand linear_vel %f", linear_vel);
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetMotionCommand steer_angle %f", steer_angle);
    // RCLCPP_INFO(rclcpp::get_logger("gazebo_ranger_robot"),"GazeboRangerRobot::SetMotionCommand angular_vel %f", angular_vel);

    double lin_vel = linear_vel / RangerParams::wheel_radius;
    double ang_vel = angular_vel / RangerParams::wheel_radius;
    double steer_ang = (std::isnan(steer_angle)) ? 0 : steer_angle;

    // set the arm positions and velocities for all wheels here
    switch (ranger_state_.motion_mode_state.motion_mode)
    {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN:
        arm_joint_state.data = {steer_ang, steer_ang, -steer_ang, -steer_ang};
        wheel_joint_state.data = {lin_vel, lin_vel, lin_vel, lin_vel};
        break;
    case MotionState::MOTION_MODE_PARALLEL:
        arm_joint_state.data = {M_PI/2, M_PI/2, M_PI/2, M_PI/2};
        lin_vel *= -steer_ang/abs(steer_ang);
        steer_ang *= -1;
        wheel_joint_state.data = {lin_vel, lin_vel, lin_vel, lin_vel};
        break;
    case MotionState::MOTION_MODE_SPINNING:
        arm_joint_state.data = {-M_PI/4, M_PI/4, M_PI/4, -M_PI/4};
        wheel_joint_state.data = {-ang_vel, ang_vel, -ang_vel, ang_vel};
        break;
    default:
        RCLCPP_ERROR(node_->get_logger(),"GazeboRangerRobot::SetMotionCommand unrecognized mode %d", motion_mode_);
        break;
    }

    // publish the arm and wheel joint state
    arm_joint_state_pub->publish(arm_joint_state);
    wheel_joint_state_pub->publish(wheel_joint_state);

    // update ranger state
    MotionStateMessage motion_state;
    motion_state.linear_velocity = linear_vel;
    motion_state.steering_angle = steer_ang;
    motion_state.angular_velocity = angular_vel;
    ranger_state_.motion_state = motion_state;
}

void GazeboRangerRobot::SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                    AgxLightMode r_mode, uint8_t r_value) // TODO
{
    RCLCPP_INFO(node_->get_logger(),"GazeboRangerRobot::SetLightCommand");
    (void)f_mode;
    (void)f_value;
    (void)r_mode;
    (void)r_value;
}

// get robot state
RangerCoreState GazeboRangerRobot::GetRobotState() // TODO 
{
    // ranger_state.time_stamp = node_.get()->now().nanoseconds();
    return ranger_state_;
}

RangerActuatorState GazeboRangerRobot::GetActuatorState() // TODO
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

RangerCommonSensorState GazeboRangerRobot::GetCommonSensorState() // TODO
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