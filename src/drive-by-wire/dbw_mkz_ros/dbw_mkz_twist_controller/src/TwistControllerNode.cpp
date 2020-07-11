/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "TwistControllerNode.h"

// THROTTLE/BRAKE PARAMETERS
double angle_scale;
double accel_cmd_smoothing;
double throttle_cmd_smoothing;
double brake_cmd_smoothing;
double steering_smoothing;
double brake_deadband;

// VSI CONTROLLER PARAMETERS
bool enable_steering;
bool enable_throttle;
bool enable_brake;

bool ignore_steering;
bool ignore_throttle;
bool ignore_brake;

// ros globals
double current_yaw_rate = 0.0;
double current_accel = 0.0;
double prev_accel_cmd = 0.0;
double prev_brake_cmd = 0.0;
double prev_throttle_cmd = 0.0;
double prev_target_speed = 0.0;

double steering_cmd_, accel_cmd;
double last_command;

void read_configs() {
  if (!ros::param::get("/twist_controller/angle_scale", angle_scale) ||
  !ros::param::get("/twist_controller/brake_deadband", brake_deadband) ||
  !ros::param::get("/twist_controller/steering_smoothing", steering_smoothing) ||
  !ros::param::get("/twist_controller/accel_cmd_smoothing", accel_cmd_smoothing) ||
  !ros::param::get("/twist_controller/throttle_cmd_smoothing", throttle_cmd_smoothing) ||
  !ros::param::get("/twist_controller/brake_cmd_smoothing", brake_cmd_smoothing) ||
  !ros::param::get("/twist_controller/enable_steering", enable_steering) ||
  !ros::param::get("/twist_controller/enable_throttle", enable_throttle) ||
  !ros::param::get("/twist_controller/enable_brake", enable_brake)) {
    ROS_ERROR_STREAM("Error reading ROS configs in TwistController");
    ros::shutdown();
  }

  ROS_INFO_STREAM("Enable Steering: " << enable_steering);
  ROS_INFO_STREAM("Enable Throttle: " << enable_throttle);
  ROS_INFO_STREAM("Enable Brake: " << enable_brake);

  ignore_steering = !enable_steering;
  ignore_brake = !enable_brake;
  ignore_throttle = !enable_throttle;
}

namespace dbw_mkz_twist_controller {

TwistControllerNode::TwistControllerNode(ros::NodeHandle &n, ros::NodeHandle &pn) : srv_(pn)
{
  lpf_fuel_.setParams(60.0, 0.1);
  accel_pid_.setRange(0.0, 1.0);

  // Dynamic reconfigure
  srv_.setCallback(boost::bind(&TwistControllerNode::reconfig, this, _1, _2));

  // Control rate parameter
  double control_rate;
  pn.param("control_rate", control_rate, 50.0);
  control_period_ = 1.0 / control_rate;

  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498; // 112.2 inches
  acker_track_ = 1.5824; // 62.3 inches
  steering_ratio_ = 14.8;
  pn.getParam("ackermann_wheelbase", acker_wheelbase_);
  pn.getParam("ackermann_track", acker_track_);
  pn.getParam("steering_ratio", steering_ratio_);
  yaw_control_.setWheelBase(acker_wheelbase_);
  yaw_control_.setSteeringRatio(steering_ratio_);

  read_configs();

  // Subscribers
  sub_command_ = n.subscribe("/dbw_cmd", 1, &TwistControllerNode::recvCommand, this);
  sub_steering_ = n.subscribe("steering_report", 1, &TwistControllerNode::recvSteeringReport, this);
  sub_imu_ = n.subscribe("imu/data_raw", 1, &TwistControllerNode::recvImu, this);
  sub_imu_raw_ = n.subscribe("/imu_raw", 1, &TwistControllerNode::recvImuRaw, this);
  sub_enable_ = n.subscribe("dbw_enabled", 1, &TwistControllerNode::recvEnable, this);
  sub_fuel_level_ = n.subscribe("fuel_level_report", 1, &TwistControllerNode::recvFuel, this);

  // Publishers
  pub_throttle_ = n.advertise<dbw_mkz_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_brake_ = n.advertise<dbw_mkz_msgs::BrakeCmd>("brake_cmd", 1);
  pub_steering_ = n.advertise<dbw_mkz_msgs::SteeringCmd>("steering_cmd", 1);
  pub_delphi_vehicle_motion_ = n.advertise<geometry_msgs::TwistStamped>("/as_rx/vehicle_motion", 1);

  // Debug
  pub_accel_ = n.advertise<std_msgs::Float64>("filtered_accel", 1);
  pub_req_accel_ = n.advertise<std_msgs::Float64>("req_accel", 1);

  // Timers
  control_timer_ = n.createTimer(ros::Duration(control_period_), &TwistControllerNode::controlCallback, this);
}

void TwistControllerNode::controlCallback(const ros::TimerEvent& event)
{
  if ((event.current_real - cmd_stamp_).toSec() > (10.0 * control_period_)) {
    accel_pid_.resetIntegrator();
    return;
  }

  double vehicle_mass = cfg_.vehicle_mass + lpf_fuel_.get() / 100.0 * cfg_.fuel_capacity * GAS_DENSITY;

  // for debugging
  std_msgs::Float64 accel_cmd_msg;
  accel_cmd_msg.data = accel_cmd;
  pub_req_accel_.publish(accel_cmd_msg);

  if (sys_enable_) {
    dbw_mkz_msgs::ThrottleCmd throttle_cmd;
    dbw_mkz_msgs::BrakeCmd brake_cmd;
    dbw_mkz_msgs::SteeringCmd steering_cmd;

    // STEERING CONTROL - recieves target_ang_vel based on error_angle to look_ahead point
    steering_cmd.enable = enable_steering;
    steering_cmd.ignore = ignore_steering;
    steering_cmd.steering_wheel_angle_cmd = steering_cmd_ * steering_ratio_;

    // THROTTLE/BRAKE CONTROL - recieves target_speed and controls based on accel_cmd with brake_deadband
    throttle_cmd.enable = enable_throttle;
    throttle_cmd.ignore = ignore_throttle;
    throttle_cmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    if (accel_cmd >= 0) { // if accel_cmd is positive, send accel_cmd - current_accel into PID control
      double new_throttle_cmd = accel_pid_.step(accel_cmd - current_accel, control_period_);
      throttle_cmd.pedal_cmd = new_throttle_cmd * throttle_cmd_smoothing + prev_throttle_cmd * (1.0 - throttle_cmd_smoothing);
      prev_throttle_cmd = throttle_cmd.pedal_cmd;
    } else { // if accel_cmd is negative, reset accel pid integrator and set throttle to 0
      accel_pid_.resetIntegrator();
      throttle_cmd.pedal_cmd = 0;
      prev_throttle_cmd = 0;
    }

    brake_cmd.enable = enable_brake;
    brake_cmd.ignore = ignore_brake;
    brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE_RQ;
    if (accel_cmd < -1 * brake_deadband) { // if accel_cmd < -0.1 m/s^2, send some brake
      double new_brake_cmd = fabs(accel_cmd * vehicle_mass * cfg_.wheel_radius);
      brake_cmd.pedal_cmd = new_brake_cmd * brake_cmd_smoothing + prev_brake_cmd * (1.0 - brake_cmd_smoothing);
      prev_brake_cmd = brake_cmd.pedal_cmd;
    } else { // if accel_cmd > -0.1 m/s^2
      brake_cmd.pedal_cmd = 0;
      prev_brake_cmd = 0.0;
    }

    printf("Throttle: %f\n", throttle_cmd.pedal_cmd);
    printf("Brake: %f\n", brake_cmd.pedal_cmd);
    pub_throttle_.publish(throttle_cmd);
    pub_brake_.publish(brake_cmd);

    printf("Steering %f\n", steering_cmd.steering_wheel_angle_cmd);
    pub_steering_.publish(steering_cmd);

    std::cout << "Command Period: " << (ros::Time::now().toSec() - last_command) * 1000 << " ms\n\n" << std::endl;
  	last_command = ros::Time::now().toSec();
  } else {
    accel_pid_.resetIntegrator();
  }
}

void TwistControllerNode::reconfig(ControllerConfig& config, uint32_t level)
{
  cfg_ = config;
  cfg_.vehicle_mass -= cfg_.fuel_capacity * GAS_DENSITY; // Subtract weight of full gas tank
  cfg_.vehicle_mass += 150.0; // Account for some passengers

  accel_pid_.setGains(cfg_.accel_kp, cfg_.accel_ki, 0.0);
  yaw_control_.setLateralAccelMax(cfg_.max_lat_accel);
  lpf_accel_.setParams(cfg_.accel_tau, 0.02);
}

void TwistControllerNode::recvCommand(const dbw_mkz_msgs::dbw_cmd::ConstPtr& msg)
{
  steering_cmd_ = msg->steering_cmd * angle_scale;
  accel_cmd = msg->accel_cmd;
  accel_cmd = accel_cmd * accel_cmd_smoothing + prev_accel_cmd * (1 - accel_cmd_smoothing);
  prev_accel_cmd = accel_cmd;
  cmd_stamp_ = ros::Time::now();

  // ROS_WARN_STREAM("Steering Cmd: " << msg->steering_cmd);
  // ROS_WARN_STREAM("Accel Cmd: " << accel_cmd);
}

void TwistControllerNode::recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg)
{
  lpf_fuel_.filt(msg->fuel_level);
}

void TwistControllerNode::recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  double raw_accel = 50.0 * (msg->speed - actual_.linear.x);
  lpf_accel_.filt(raw_accel);

  std_msgs::Float64 accel_msg;
  accel_msg.data = lpf_accel_.get();
  pub_accel_.publish(accel_msg);

  actual_.linear.x = msg->speed;
}

void TwistControllerNode::recvImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  actual_.angular.z = msg->angular_velocity.z; // current yaw rate
}

void TwistControllerNode::recvImuRaw(const sensor_msgs::Imu::ConstPtr& msg)
{
  current_yaw_rate = 0.0; //msg->angular_velocity.z; // current yaw rate from imu
  current_accel = msg->linear_acceleration.x;

  geometry_msgs::TwistStamped vehicle_motion_msg;
  vehicle_motion_msg.header.stamp = ros::Time::now();
  vehicle_motion_msg.twist.linear.x = actual_.linear.x;
  vehicle_motion_msg.twist.angular.x = msg->angular_velocity.x;
  vehicle_motion_msg.twist.angular.y = msg->angular_velocity.y;
  vehicle_motion_msg.twist.angular.z = msg->angular_velocity.z;
  pub_delphi_vehicle_motion_.publish(vehicle_motion_msg);
}

void TwistControllerNode::recvEnable(const std_msgs::Bool::ConstPtr& msg)
{
  sys_enable_ = msg->data;
}

}
