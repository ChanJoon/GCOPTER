#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <uav_utils/utils.h>
#include <uav_utils/converters.h>
#include "ctrl_param.h"

class RC_Data_t {
  public:
    double mode;
    double gear;
    double last_mode;
    double last_gear;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_api_mode;
    bool enter_api_mode;    

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;

    RC_Data_t(){
      rcv_stamp = ros::Time(0);

      last_mode = -1.0;
      last_gear = -1.0;

      is_command_mode = false;
      enter_command_mode = false;
      is_api_mode = false;
      enter_api_mode = false;
    };

    void check_validity(){
      if ( mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1) {
        // pass
      } else {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f", mode, gear);
      }
    };

    void feed(mavros_msgs::RCInConstPtr pMsg){
      msg = *pMsg;
      rcv_stamp = ros::Time::now();

      mode = ((double)msg.channels[4]-1000.0)/1000.0;
      gear = ((double)msg.channels[5]-1000.0)/1000.0;

      if ( !have_init_last_mode )
      {
        have_init_last_mode = true;
        last_mode = mode;
      } 
      if ( !have_init_last_gear )
      {
        have_init_last_gear = true;
        last_gear = gear;
      } 

      check_validity();

      if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_api_mode = true;
      else
        enter_api_mode = false;

      if (mode > API_MODE_THRESHOLD_VALUE)
        is_api_mode = true;
      else
        is_api_mode = false;

      if (is_api_mode && last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE) {
        enter_command_mode = true;
      } else if (gear < GEAR_SHIFT_VALUE) {
        enter_command_mode = false;
      }

      if (gear > GEAR_SHIFT_VALUE)
        is_command_mode = true;
      else
        is_command_mode = false;

      last_mode = mode;
      last_gear = gear;
    };
};


class Odom_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool odom_init;
    ros::Publisher debug_odom_pub;
    ros::NodeHandle nh_;

    Odom_Data_t(){
      rcv_stamp = ros::Time(0);
      q.setIdentity();
      odom_init = false;
      debug_odom_pub = nh_.advertise<nav_msgs::Odometry>("/debug_odom", 10);
    };

    void feed(nav_msgs::OdometryConstPtr pMsg){
      msg = *pMsg;
      rcv_stamp = ros::Time::now();
      uav_utils::extract_odometry(pMsg, p, v, q, w);
      odom_init = true;

      msg.header.stamp = rcv_stamp;
      msg.header.frame_id = "odom";     // instead of "map", transform to "odom" (global_planning visualization)
      debug_odom_pub.publish(msg);
    };
};


class Imu_Data_t {
  public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    bool imu_init;
    Imu_Data_t(){
      rcv_stamp = ros::Time(0);
      imu_init = false;
    };

    void feed(sensor_msgs::ImuConstPtr pMsg){
      msg = *pMsg;
      rcv_stamp = ros::Time::now();

      w(0) = msg.angular_velocity.x;
      w(1) = msg.angular_velocity.y;
      w(2) = msg.angular_velocity.z;

      a(0) = msg.linear_acceleration.x;
      a(1) = msg.linear_acceleration.y;
      a(2) = msg.linear_acceleration.z;

      q.x() = msg.orientation.x;
      q.y() = msg.orientation.y;
      q.z() = msg.orientation.z;
      q.w() = msg.orientation.w;
      imu_init = true;
    };
};

class State_Data_t {
  public:
    mavros_msgs::State current_state;
    bool state_init;

    State_Data_t(){
      state_init = false;
    };
    void feed(mavros_msgs::StateConstPtr pMsg){  
      current_state = *pMsg;
      state_init = true;
    };
};

class Command_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d jerk;
    double yaw;
    double head_rate;
    bool cmd_init;
    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;
    ros::Publisher debug_cmd_pub;
    nav_msgs::Odometry des_odom_msg;
    ros::NodeHandle nh_;

    Command_Data_t(){
      rcv_stamp = ros::Time(0);
      cmd_init = false;
      debug_cmd_pub = nh_.advertise<nav_msgs::Odometry>("/debug_cmd", 10);
    };

    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg){
      static double last_time;
      static double last_yaw;
      double now_time;

      msg = *pMsg;
      rcv_stamp = ros::Time::now();
      now_time = ros::Time::now().toSec();
      p(0) = msg.position.x;
      p(1) = msg.position.y;
      p(2) = msg.position.z;

      v(0) = msg.velocity.x;
      v(1) = msg.velocity.y;
      v(2) = msg.velocity.z;

      a(0) = msg.acceleration.x;
      a(1) = msg.acceleration.y;
      a(2) = msg.acceleration.z;

      jerk(0) = msg.jerk.x;
      jerk(1) = msg.jerk.y;
      jerk(2) = msg.jerk.z;

      yaw = uav_utils::normalize_angle(msg.yaw);
      if(!cmd_init){
        last_time = now_time;
        head_rate = 0.0;
        last_yaw = yaw;
      }
      else{
        double diff_time = now_time-last_time;
        last_time = now_time;
        double diff_yaw;
        double angle1 = yaw;
        double angle2 = last_yaw;
        last_yaw = yaw;

        double TwoPi = 2*M_PI;
        if (angle1 < 0)
            angle1 = TwoPi + angle1;
        if (angle2 < 0)
            angle2 = TwoPi + angle2;
        double dist = angle1 - angle2;
        if (dist > M_PI)
            angle1 = angle1 - TwoPi;
        //if two much on other side then invert second angle
        else if (dist < -M_PI)
            angle2 = angle2 - TwoPi;
        diff_yaw = (angle1-angle2);
        diff_time = 0.01;//hzchzc
        head_rate = diff_yaw/diff_time;
        uav_utils::limit_range(head_rate,1.0);     
        // printf("angle1: %f, angle2: %f, head_rate: %f \n, diff_time: %f",angle1,angle2,head_rate,diff_time);
      }
      cmd_init = true;

      des_odom_msg.header.frame_id = "odom";
      des_odom_msg.header.stamp = rcv_stamp;
      des_odom_msg.pose.pose.position.x = msg.position.x;
      des_odom_msg.pose.pose.position.y = msg.position.y;
      des_odom_msg.pose.pose.position.z = msg.position.z;

      Eigen::Quaterniond q = uav_utils::yaw_to_quaternion(msg.yaw);
      des_odom_msg.pose.pose.orientation.w = q.w();
      des_odom_msg.pose.pose.orientation.x = q.x();
      des_odom_msg.pose.pose.orientation.y = q.y();
      des_odom_msg.pose.pose.orientation.z = q.z();
      debug_cmd_pub.publish(des_odom_msg);
    };
};

#endif