#ifndef __PX4_CONTROL_H
#define __PX4_CONTROL_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "input.h"
#include "hovthrkf.h"
#include "controller.h"

#include <uav_utils/converters.h>

class Control
{
public:
	Parameter_t& param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;

	Controller& controller;
	HovThrKF& hov_thr_kf;

	ros::Publisher des_pose_pub;
	ros::Publisher traj_start_trigger_pub;

	Eigen::Vector4d hover_pose;

	enum State_t
	{
		MANUAL_CTRL,             // Ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER,	 			// Ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL				 // Ctrl is actived, and controling the drone.
	};

	Control(Parameter_t &, Controller &, HovThrKF &);
	void process();
	bool rc_is_received(const ros::Time& now_time);
	bool cmd_is_received(const ros::Time& now_time);
	bool odom_is_received(const ros::Time& now_time);
	bool imu_is_received(const ros::Time& now_time);
	bool px4_init();


private:
	State_t state;
	// ---- control related ----
	void process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	void process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	void publish_desire(const Desired_State_t& des);

	// ---- tools ----
	double get_yaw_from_odom();
	void align_with_imu(Controller_Output_t& u);
	void set_hov_with_odom();

	void toggle_offboard_mode(bool on_off);  // It will only try to toggle once, so not blocked.

	void publish_trigger(const nav_msgs::Odometry& odom_msg);
};


Control::Control(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_):
	param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

void Control::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;

	// if (!rc_is_received(now_time))
	// {
	// 	ROS_ERROR("RC lost for %3f seconds!!!!", (now_time - rc_data.rcv_stamp).toSec());
	// }

	// std::cout << state << " " << rc_data.is_api_mode  << std::endl;

	switch (state)
	{
		case MANUAL_CTRL:
		{
			if ( odom_is_received(now_time) )
			{
				state = AUTO_HOVER;
				controller.config_gain(param.hover_gain);
				set_hov_with_odom();
				toggle_offboard_mode(true);

				ROS_INFO("\033[32m[Ctrl] Start AUTO_HOVER by Ctrl!\033[32m");
			}
			else
			{
				ROS_ERROR("[Ctrl] Reject AUTO_HOVER. No odom!");
			}

			break;
		}

		case AUTO_HOVER:
		{
			if ( cmd_is_received(now_time) )
			{
				ROS_INFO("Command received!");
				if ( state_data.current_state.mode == "OFFBOARD" )
				{
					state = CMD_CTRL;
					controller.config_gain(param.track_gain);
					process_cmd_control(u, u_so3);
					ROS_INFO("\033[32m[Ctrl] position commands received!\033[32m");
				}
			}
			else
			{
				process_hover_control(u, u_so3);
				ROS_INFO("\033[32m[Ctrl] Hovering!\033[32m");
			}

			break;
		}

		case CMD_CTRL:
		{
			if ( !cmd_is_received(now_time) )
			{
				state = AUTO_HOVER;
            	controller.config_gain(param.hover_gain);
            	set_hov_with_odom();
				process_hover_control(u, u_so3);
				ROS_WARN("[Ctrl] Return to AUTO_HOVER!");
			}
			else
			{
				process_cmd_control(u, u_so3);
			}
			
			break;
		}
	
		default:
			break;
	}
	
	if (state == AUTO_HOVER){
		//MEMO: Currently, use Controll_Output_t of 'process_hover_control' is unstable for hovering
		// So just use position control
		Desired_State_t des;
		des.p = hover_pose.head<3>();
		des.v = Eigen::Vector3d::Zero();
		des.yaw = hover_pose(3);
		des.a = Eigen::Vector3d::Zero();
		des.jerk = Eigen::Vector3d::Zero();
		publish_desire(des);
	}
	else if (state == CMD_CTRL)
		controller.publish_ctrl(u, now_time);
	hov_thr_kf.simple_update(u.des_v_real, odom_data.v );
	// This line may not take effect according to param.hov.use_hov_percent_kf
	param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

void Control::publish_trigger(const nav_msgs::Odometry& odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	
    traj_start_trigger_pub.publish(msg);
}

bool Control::rc_is_received(const ros::Time& now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool Control::cmd_is_received(const ros::Time& now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool Control::odom_is_received(const ros::Time& now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool Control::imu_is_received(const ros::Time& now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

double Control::get_yaw_from_odom()
{
	return uav_utils::get_yaw_from_quaternion(odom_data.q);
}

void Control::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.a = Eigen::Vector3d::Zero();
	des.jerk = Eigen::Vector3d::Zero();
	controller.update(des, odom_data, u, u_so3);

	// publish_desire(des);
}

void Control::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;
	des.jerk = cmd_data.jerk;
	des.head_rate = cmd_data.head_rate;

	controller.update(des, odom_data, u, u_so3);

	// publish_desire(des);	
}

void Control::align_with_imu(Controller_Output_t& u)
{
	double imu_yaw = uav_utils::get_yaw_from_quaternion(imu_data.q); 
	double odom_yaw = get_yaw_from_odom();
	double des_yaw = u.yaw;
	// ROS_INFO_STREAM("imu yaw: "<<imu_yaw<<" odom_yaw: "<<odom_yaw);
	u.yaw = uav_utils::yaw_add(uav_utils::yaw_add(des_yaw, -odom_yaw), imu_yaw); 

	//out << "imu_yaw=" << imu_yaw << " odom_yaw=" << odom_yaw << " des_yaw=" << des_yaw << " u.yaw=" << u.yaw << endl;
};

void Control::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(2) = 3.0; // Temporailiy set to 3.0 m
	hover_pose(3) = get_yaw_from_odom();
}

void Control::publish_desire(const Desired_State_t& des)
{
	geometry_msgs::PoseStamped msg;
	msg.header = odom_data.msg.header;

	msg.pose.position.x = des.p(0);
	msg.pose.position.y = des.p(1);
	msg.pose.position.z = des.p(2);

	Eigen::Quaterniond q = uav_utils::yaw_to_quaternion(des.yaw);

	msg.pose.orientation.w = q.w();
	msg.pose.orientation.x = q.x();
	msg.pose.orientation.y = q.y();
	msg.pose.orientation.z = q.z();

	des_pose_pub.publish(msg);
}

void Control::toggle_offboard_mode(bool on_off)
{	
	mavros_msgs::SetMode offb_set_mode;
	ros::Time last_request = ros::Time::now();

	if ( on_off )
	{
		offb_set_mode.request.custom_mode = "OFFBOARD";
		controller.set_FCU_mode.call(offb_set_mode);
		int count = 0;
		while(count < 5 && ros::ok())
		{
			offb_set_mode.request.custom_mode = "OFFBOARD";
			if( state_data.current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
			{
				// geometry_msgs::PoseStamped init_pos;
				// init_pos.pose.position.x = 0.0;
				// init_pos.pose.position.y = 0.0;
				// init_pos.pose.position.z = 4.0;
				// init_pos.pose.orientation.x = 0.0;
				// init_pos.pose.orientation.y = 0.0;
				// init_pos.pose.orientation.z = 0.0;
				// init_pos.pose.orientation.w = 1.0;
				// des_pose_pub.publish(init_pos);
				if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				{
					ROS_INFO("Offboard enabled");
					return;
				}
				last_request = ros::Time::now();
				ROS_WARN("on Again.");
				count++;
			}
        	ros::spinOnce();
		}
		ROS_WARN("Toggle OFFBOARD mode on failed.");
	}
	else
	{
		offb_set_mode.request.custom_mode = "ALTCTL";
		controller.set_FCU_mode.call(offb_set_mode);
		int count = 0;
		while(count < 5 && ros::ok())
		{
			if ( state_data.current_state.mode == "OFFBOARD" )
			{
				ROS_INFO("Not in OFFBOARD mode");
				return;
			}
			offb_set_mode.request.custom_mode = "ALTCTL";
			if( state_data.current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
			{
				if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				{
					ROS_INFO("Return from OFFBOARD mode");
					return;
				}
				ROS_WARN("off Again.");
				last_request = ros::Time::now();
				count++;
			}
        	ros::spinOnce();
		}
		ROS_ERROR("Toggle OFFBOARD mode off failed. EMERGENCY!!!!!");
	}
	
}
bool Control::px4_init(){
	bool state_check = odom_data.odom_init&&imu_data.imu_init&&state_data.state_init;
	bool armed_check = state_data.current_state.armed;
	if(state_check){
		if(!armed_check){
			mavros_msgs::CommandBool arming;
			arming.request.value = true;
			controller.cmd_FCU_arming.call(arming);
			ROS_WARN("Arming...");
		}
	}
	return state_check && armed_check;
}


#endif