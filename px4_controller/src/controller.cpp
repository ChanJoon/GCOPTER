#include "controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <boost/format.hpp>
#include "std_msgs/Float32.h"
using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_)
{
	is_configured = false;
	int_e_v.setZero();
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
}

void Controller::config_gain(const Parameter_t::Gain& gain)
{
	Kp.setZero();
	Kv.setZero();
	Ka.setZero();
	Kvi.setZero();
	Kp(0,0) = gain.Kp0;
	Kp(1,1) = gain.Kp1;
	Kp(2,2) = gain.Kp2;
	Kv(0,0) = gain.Kv0;
	Kv(1,1) = gain.Kv1;
	Kv(2,2) = gain.Kv2;
	Kvi(0,0) = gain.Kvi0;
	Kvi(1,1) = gain.Kvi1;
	Kvi(2,2) = gain.Kvi2;
	Ka(0,0) = gain.Ka0;
	Ka(1,1) = gain.Ka1;
	Ka(2,2) = gain.Ka2;
	Kyaw = gain.Kyaw;
}
Eigen::Quaterniond Controller::computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) const{
	//desired_acceleration means the desired thrust and is perpendicular to the body frame.
	
	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));
	// Compute desired orientation
  	const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  	const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  	Eigen::Vector3d z_B;
  	if (almostZero(desired_acceleration.norm())) {
		// In case of free fall we keep the thrust direction to be the estimated one
		// This only works assuming that we are in this condition for a very short
		// time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  	}	
	else {
    	z_B = desired_acceleration.normalized();
  	}

  	const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  	const Eigen::Vector3d x_B =
    computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);
  	const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  	// From the computed desired body axes we can now compose a desired attitude
	const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
	// ROS_INFO_STREAM("R_W_B: "<<R_W_B);
	const Eigen::Quaterniond desired_attitude(R_W_B);

	return desired_attitude;
}

Controller_Output_t Controller::computeNominalReferenceInputs(
    const Desired_State_t& reference_state,
    const Odom_Data_t& attitude_estimate) const{

	Controller_Output_t reference_command;
  	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_state.yaw, Eigen::Vector3d::UnitZ()));
  	const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
 	const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  	const Eigen::Vector3d des_acc = reference_state.a + Vector3d(0,0,param.gra);
	//desired thrust direction, the same as se3 planner

	//	F_des = u_v * param.mass + 
	//	Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
  // Reference attitude
  	const Eigen::Quaterniond q_W_B = computeDesiredAttitude(
      des_acc, reference_state.yaw, attitude_estimate.q);
	//same as planner

  const Eigen::Vector3d x_B = q_W_B * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_B = q_W_B * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();

  reference_command.orientation = q_W_B;

  // Reference thrust
  reference_command.normalized_thrust = des_acc.norm();

  // Reference body rates
  if (almostZeroThrust(reference_command.normalized_thrust)) {
    reference_command.roll_rate = 0.0;
    reference_command.pitch_rate = 0.0;
  } else {
    reference_command.roll_rate = -1.0 /
                                      reference_command.normalized_thrust *
                                      y_B.dot(reference_state.jerk);
    reference_command.pitch_rate = 1.0 /
                                      reference_command.normalized_thrust *
                                      x_B.dot(reference_state.jerk);
  }
//   reference_command.yaw_rate = 0.0;

  if (almostZero((y_C.cross(z_B)).norm())) {
    reference_command.yaw_rate = 0.0;
  } else {
    reference_command.yaw_rate =
        1.0 / (y_C.cross(z_B)).norm() *
        (reference_state.head_rate * x_C.dot(x_B) +
         reference_command.pitch_rate * y_C.dot(z_B));
  }

  // Reference angular accelerations
//   if (almostZeroThrust(reference_command.collective_thrust)) {
//     reference_command.angular_accelerations.x() = 0.0;
//     reference_command.angular_accelerations.y() = 0.0;
//   } else {
//     const double thrust_dot = z_B.dot(reference_state.jerk);
//     reference_command.angular_accelerations.x() =
//         -1.0 / reference_command.collective_thrust *
//         (y_B.dot(reference_state.snap) +
//          2.0 * thrust_dot * reference_command.bodyrates.x() -
//          reference_command.collective_thrust * reference_command.bodyrates.y() *
//              reference_command.bodyrates.z());
//     reference_command.angular_accelerations.y() =
//         1.0 / reference_command.collective_thrust *
//         (x_B.dot(reference_state.snap) -
//          2.0 * thrust_dot * reference_command.bodyrates.y() -
//          reference_command.collective_thrust * reference_command.bodyrates.x() *
//              reference_command.bodyrates.z());
//   }

//   if (almostZero((y_C.cross(z_B)).norm())) {
//     reference_command.angular_accelerations.z() = 0.0;
//   } else {
//     reference_command.angular_accelerations.z() =
//         1.0 / (y_C.cross(z_B)).norm() *
//         (reference_state.heading_acceleration * x_C.dot(x_B) +
//          2.0 * reference_state.heading_rate * reference_command.bodyrates.z() *
//              x_C.dot(y_B) -
//          2.0 * reference_state.heading_rate * reference_command.bodyrates.y() *
//              x_C.dot(z_B) -
//          reference_command.bodyrates.x() * reference_command.bodyrates.y() *
//              y_C.dot(y_B) -
//          reference_command.bodyrates.x() * reference_command.bodyrates.z() *
//              y_C.dot(z_B) +
//          reference_command.angular_accelerations.y() * y_C.dot(z_B));
//   }

  return reference_command;


	}
	
void Controller::update(
	const Desired_State_t& des, 
	const Odom_Data_t& odom, 
	Controller_Output_t& u, 
	SO3_Controller_Output_t& u_so3
)
{
	/*ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized!");
	std::string constraint_info("");
	Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;


	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v.setZero();
	}

	double yaw_curr = get_yaw_from_quaternion(odom.q);

	// ROS_INFO_STREAM("YAW: "<<yaw_curr);
	double	yaw_des = des.yaw;
	Matrix3d wRc = rotz(yaw_curr);
	Matrix3d cRw = wRc.transpose();
	// ROS_INFO_STREAM("desp: "<<des.p<<"odomp: "<<odom.p);
	// ROS_INFO_STREAM("WRC: "<<wRc);
	e_p = des.p - odom.p;
	Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;

	
	u.des_v_real = des.v + u_p; // For estimating hover percent
	e_v = des.v + u_p - odom.v;

	const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}
  
	Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;
	// ROS_INFO_STREAM("wRc: "<<wRc);
	// cout<<endl;
	// ROS_INFO_STREAM(" Kvi:"<<Kvi<<" cRw: "<<cRw<<" e_v: "<<int_e_v);
	// ROS_INFO_STREAM("u_v_iaaaaaaaaaaAA: "<<u_v_i);
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}
	//proble is u_v_i
	// ROS_INFO_STREAM("u_v_p: "<<u_v_p<<" u_v_i: "<<u_v_i);
	Eigen::Vector3d u_v = u_v_p + u_v_i;
	// ROS_INFO_STREAM("u_v: "<<u_v);

	e_yaw = yaw_des - yaw_curr;

	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);

	double u_yaw = Kyaw * e_yaw;
	
	// ROS_INFO_STREAM("u_v: "<<u_v<<"Ka: "<<Ka<<" des a"<<des.a);//problem is uv
	F_des = u_v * param.mass + 
		Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;




	
	
	// ROS_INFO_STREAM("1111111111F_des: "<<F_des);
	// ROS_INFO_STREAM("F_des: "<<F_des);
	//no problems
	if (F_des(2) < 0.5 * param.mass * param.gra)
	{
		// cout<<"aaaaaaaaaa";
		constraint_info = boost::str(
			boost::format("thrust too low F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (0.5 * param.mass * param.gra);
	}
	else if (F_des(2) > 2 * param.mass * param.gra)
	{
		// cout<<"bbbbbbbbbbbbbb";
		constraint_info = boost::str(
			boost::format("thrust too high F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (2 * param.mass * param.gra);
	}

	if (std::fabs(F_des(0)/F_des(2)) > std::tan(toRad(50.0)))
	{
		// cout<<"cccccccccccccccc";
		constraint_info += boost::str(boost::format("x(%f) too tilt; ")
			% toDeg(std::atan2(F_des(0),F_des(2))));
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(toRad(30.0));
	}

	if (std::fabs(F_des(1)/F_des(2)) > std::tan(toRad(50.0)))
	{
		// cout<<"dddddddddddddddddddddd";
		constraint_info += boost::str(boost::format("y(%f) too tilt; ")
			% toDeg(std::atan2(F_des(1),F_des(2))));
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(toRad(30.0));	
	}

	Vector3d z_b_des = F_des / F_des.norm();
	//理论的z轴方向，世界系
	
	/////////////////////////////////////////////////
	// Z-X-Y Rotation Sequence                
	// Vector3d x_c_des = Vector3d(std::cos(yaw_des), sin(yaw_des), 0.0);
	// Vector3d y_b_des = z_b_des.cross(x_c_des) / z_b_des.cross(x_c_des).norm();
	// Vector3d x_b_des = y_b_des.cross(z_b_des);
	/////////////////////////////////////////////////

	/////////////////////////////////////////////////
	// Z-Y-X Rotation Sequence                
	Vector3d y_c_des = Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
	Vector3d x_b_des = y_c_des.cross(z_b_des) / y_c_des.cross(z_b_des).norm();
	Vector3d y_b_des = z_b_des.cross(x_b_des);
	///////////////////////////////////////////////// 

	Matrix3d R_des1; // it's wRb
	R_des1 << x_b_des, y_b_des, z_b_des;
	
	Matrix3d R_des2; // it's wRb
	R_des2 << -x_b_des, -y_b_des, z_b_des;
	
	Vector3d e1 = R_to_ypr(R_des1.transpose() * odom.q.toRotationMatrix());
	Vector3d e2 = R_to_ypr(R_des2.transpose() * odom.q.toRotationMatrix());

	Matrix3d R_des; // it's wRb

	if (e1.norm() < e2.norm())
	{
		R_des = R_des1;
	}
	else
	{
		R_des = R_des2;
	}

	{	
		//debug_msg
		Eigen::Vector3d debug_F;
		debug_F = Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
		Vector3d debug_Fc = wRc.transpose()*debug_F;
		double debug_fx = debug_Fc(0);
		double debug_fy = debug_Fc(1);
		double debug_fz = debug_Fc(2);
		std_msgs::Float32 debug_roll;
		debug_roll.data  = 180*std::atan2(-debug_fy, debug_fz)/M_PI;
		std_msgs::Float32 debug_pitch; 
		debug_pitch.data= 180*std::atan2( debug_fx, debug_fz)/M_PI;
		debug_roll_pub.publish(debug_roll);
		debug_pitch_pub.publish(debug_pitch);




		// ROS_INFO_STREAM("F_des : "<<F_des);
		Vector3d F_c = wRc.transpose() * F_des;
		Matrix3d wRb_odom = odom.q.toRotationMatrix();
		Vector3d z_b_curr = wRb_odom.col(2);
		double u1 = F_des.dot(z_b_curr);
		// double u1 = F_des.norm();
		double fx = F_c(0);
		double fy = F_c(1);
		double fz = F_c(2);
		// ROS_INFO_STREAM("FX: "<<fx<<"FY: "<<fy<<"FZ: "<<fz);
		u.roll  = std::atan2(-fy, fz);
		u.pitch = std::atan2( fx, fz);
		u.thrust = u1 / param.full_thrust;
		u.thrust = u.thrust>=1?1:u.thrust;
		if(param.use_yaw_rate_ctrl){
			u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
			u.yaw = u_yaw;
		}
		else{
			u.yaw_mode = Controller_Output_t::CTRL_YAW;
			u.yaw = des.yaw;
		}
	}*/
	//hzchzc
	/*
	*/
	// ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized!");
	// std::string constraint_info("");
	// Vector3d e_p, e_v, F_des;
	// double e_yaw = 0.0;
	// if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
	// 	// ROS_INFO("Reset integration");
	// 	int_e_v.setZero();
	// }
	// double yaw_curr = get_yaw_from_quaternion(odom.q);
	// double	yaw_des = des.yaw;
	// Matrix3d wRc = rotz(yaw_curr);
	// Matrix3d cRw = wRc.transpose();
	// e_p = des.p - odom.p;
	// Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;
	// u.des_v_real = des.v + u_p; // For estimating hover percent
	// e_v = des.v + u_p - odom.v;
	// const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	// for (size_t k = 0; k < 3; ++k) {
	// 	if (std::fabs(e_v(k)) < 0.2) {
	// 		int_e_v(k) += e_v(k) * 1.0 / 50.0;
	// 	}
	// }
	// Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
	// const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	// Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;
	// for (size_t k = 0; k < 3; ++k) {
	// 	if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
	// 		uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
	// 		ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
	// 	}
	// }
	// Eigen::Vector3d u_v = u_v_p + u_v_i;
	// e_yaw = yaw_des - yaw_curr;
	// while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	// while(e_yaw < -M_PI) e_yaw += (2 * M_PI);
	// double u_yaw = Kyaw * e_yaw;
	// F_des = u_v * param.mass + 
	// 	Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;


	//hzchzc
	/*






	*step1
	Compute reference inputs
	*/
	 // Compute reference inputs
	std::string constraint_info("");
	Eigen::Vector3d drag_accelerations = Eigen::Vector3d::Zero();
	Controller_Output_t reference_inputs;
	if (param.perform_aerodynamics_compensation) {
    // Compute reference inputs that compensate for aerodynamic drag
    computeAeroCompensatedReferenceInputs(des, odom, &reference_inputs, &drag_accelerations);
	} 
	else {
		// In this case we are not considering aerodynamic accelerations
		drag_accelerations = Eigen::Vector3d::Zero();
		// Compute reference inputs as feed forward terms
		reference_inputs = computeNominalReferenceInputs(des, odom);
	}
	/*
	step2 get the pid error
	*/
	// Vector3d F_des;
	// F_des = computePIDErrorAcc(odom,des)*param.mass;//backfeed
	// F_des += Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;//feed back
	// F_des -= drag_accelerations*param.mass;

	Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;
	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v.setZero();
	}
	double yaw_curr = get_yaw_from_quaternion(odom.q);
	ROS_INFO("Desire state px: %f, py: %f, pz: %f, yaw: %f, head_rate: %f", des.p(0), des.p(1), des.p(2), des.yaw, des.head_rate);
	ROS_INFO("Current state px: %f, py: %f, pz: %f, yaw: %f", odom.p(0), odom.p(1), odom.p(2), yaw_curr);
	double	yaw_des = des.yaw;
	Matrix3d wRc = rotz(yaw_curr);
	Matrix3d cRw = wRc.transpose();
	e_p = des.p - odom.p;
	Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;
	u.des_v_real = des.v + u_p; // For estimating hover percent
	e_v = des.v + u_p - odom.v;

	const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}
	Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			// ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}
	Eigen::Vector3d u_v = u_v_p + u_v_i;
	e_yaw = yaw_des - yaw_curr;
	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);
	double u_yaw = Kyaw * e_yaw;
	F_des = u_v * param.mass + 
		Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
	F_des -= drag_accelerations*param.mass;
	



	Matrix3d wRb_odom = odom.q.toRotationMatrix();
	Vector3d z_b_curr = wRb_odom.col(2);
	double u1 = F_des.dot(z_b_curr);
	double fullparam = 0.85; // 0.7
	u.thrust = u1 / param.full_thrust;
	ROS_INFO("Thrust: %f", u.thrust);
	if(u.thrust>=fullparam)
		ROS_WARN("FULL THRUST");
	u.thrust = u.thrust>=fullparam?fullparam:u.thrust;



	// if(param.use_yaw_rate_ctrl){
	// 	u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
	// 	u.yaw = u_yaw;
	// }
	// else{
	// 	u.yaw_mode = Controller_Output_t::CTRL_YAW;
	// 	u.yaw = des.yaw;
	// }
	const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(F_des/param.mass, des.yaw, odom.q);
	const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q);
	//MEMO: Somewhat 'feedback_bodyrates' are so noisy, only use reference_inputs works well (in OFFBOARD) but those values are just [9.81, 0, 0, 0].
	u.roll_rate = reference_inputs.roll_rate+feedback_bodyrates.x();
	// u.roll_rate = reference_inputs.roll_rate;
	u.pitch_rate = reference_inputs.pitch_rate+feedback_bodyrates.y();
	// u.pitch_rate = reference_inputs.pitch_rate;
	u.yaw_rate = reference_inputs.yaw_rate+feedback_bodyrates.z();
	// u.roll_rate = 0.0;
	// u.pitch_rate = 0.0;
	
	ROS_INFO_STREAM("Reference input thrust: " << reference_inputs.normalized_thrust << " roll_rate: " << reference_inputs.roll_rate << " yaw rate: " << reference_inputs.yaw_rate);
	ROS_INFO_STREAM("feedback bodyrate x: " << feedback_bodyrates.x() << " y: " << feedback_bodyrates.y() << " z: " << feedback_bodyrates.z());

	// double limit_rate = 6*3.14;
	double limit_rate = 1.5*3.14; // https://github.com/HKUST-Aerial-Robotics/PredRecon/blob/master/Planner/Code/src/px4ctrl/src/controller.cpp
	if(u.roll_rate>=limit_rate)
		ROS_WARN("ROLL RATE LIMIT! %f", u.roll_rate);
	if(u.pitch_rate>=limit_rate)
		ROS_WARN("pitch_rate_limit! %f", u.pitch_rate);
	if(u.yaw_rate>=1.5)
		ROS_WARN("yaw rate: %f", u.yaw_rate);

	uav_utils::limit_range(u.roll_rate,limit_rate);//3.0
	uav_utils::limit_range(u.pitch_rate,limit_rate);
	uav_utils::limit_range(u.yaw_rate,1.5);
};

void Controller::publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	// msg.header.frame_id = std::string("FCU");
	msg.header.frame_id = std::string("map");
	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
	msg.body_rate.x = u.roll_rate;
	msg.body_rate.y = u.pitch_rate;
	msg.body_rate.z = u.yaw_rate;
	msg.thrust = u.thrust;

  ctrl_FCU_pub.publish(msg);
}

void Controller::publish_zero_ctrl(const ros::Time& stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE  | 
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | 
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	Eigen::Vector3d uAngle(0.0, 0.0, 0.0);
	Eigen::Quaterniond quaternion = ypr_to_quaternion(uAngle);
	msg.orientation.x = quaternion.x();
	msg.orientation.y = quaternion.y();
	msg.orientation.z = quaternion.z();
	msg.orientation.w = quaternion.w();
	
	msg.thrust = param.hov_percent*0.8;
	
  ctrl_FCU_pub.publish(msg);
}
bool Controller::almostZero(const double value) const{
	return fabs(value) < 0.001; 
}
bool Controller::almostZeroThrust(const double thrust_value) const {
  return fabs(thrust_value) < 0.01;
}
Eigen::Vector3d Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const {
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }

  return x_B;
}
Eigen::Vector3d Controller::computeFeedBackControlBodyrates(const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate){
		  // Compute the error quaternion
  const Eigen::Quaterniond q_e = attitude_estimate.inverse() * desired_attitude;
  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;
  double krp = param.track_gain.Krp;
  double kyaw = param.track_gain.Kyaw;

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * krp * q_e.x();
    bodyrates.y() = 2.0 * krp * q_e.y();
    bodyrates.z() = 2.0 * kyaw * q_e.z();
  } else {
    bodyrates.x() = -2.0 * krp * q_e.x();
    bodyrates.y() = -2.0 * krp * q_e.y();
    bodyrates.z() = -2.0 * kyaw * q_e.z();
  }

  return bodyrates;
}
Eigen::Vector3d Controller::computePIDErrorAcc(
    const Odom_Data_t& state_estimate,
    const Desired_State_t& reference_state){
	  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error =
      reference_state.p[0] - state_estimate.p[0];
  //uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
  uav_utils::limit_range(x_pos_error,param.pxy_error_max);

  double x_vel_error =
      reference_state.v[0] - state_estimate.v[0];
  uav_utils::limit_range(x_vel_error,param.vxy_error_max);

  acc_error.x() = Kp(0,0) * x_pos_error + Kv(0,0) * x_vel_error;
  // y acceleration
  double y_pos_error =
      reference_state.p[1] - state_estimate.p[1];
  uav_utils::limit_range(y_pos_error,param.pxy_error_max);

  double y_vel_error =
      reference_state.v[1]- state_estimate.v[1];
  uav_utils::limit_range(y_vel_error,param.vxy_error_max);
  acc_error.y() = Kp(1,1) * y_pos_error + Kv(1,1) * y_vel_error;
  // z acceleration
  double z_pos_error =
      reference_state.p[2] - state_estimate.p[2];
  uav_utils::limit_range(z_pos_error,
                          param.pz_error_max);

  double z_vel_error =
      reference_state.v[2] - state_estimate.v[2];
  uav_utils::limit_range(z_vel_error, param.vz_error_max);

  acc_error.z() = Kp(2,2) * z_pos_error + Kv(2,2) * z_vel_error;

  return acc_error;
}

// https://github.com/uzh-rpg/rpg_quadrotor_control/blob/master/control/position_controller/src/position_controller.cpp
//MEMO Odom_Data_t and Controller_Output_t has no member `bodyrates`, `angular_acceleration`
// Also, we need Eigen::Matrix3d `omega_hat` = skew(reference_command.bodyrates)
void Controller::computeAeroCompensatedReferenceInputs(
    const Desired_State_t& reference_state,
    const Odom_Data_t& state_estimate,
    Controller_Output_t* reference_inputs,
    Eigen::Vector3d* drag_accelerations){
  Controller_Output_t reference_command;
	const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, param.gra);

  const Eigen::Vector3d velocity_in_body =
      state_estimate.q.inverse() * reference_state.v;

  const double dx = param.k_drag_x;
  const double dy = param.k_drag_y;
  const double dz = param.k_drag_z;

  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.yaw, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d alpha =
      reference_state.a - kGravity_ + dx * reference_state.v;
  const Eigen::Vector3d beta =
      reference_state.a - kGravity_ + dy * reference_state.v;
  const Eigen::Vector3d gamma =
      reference_state.a - kGravity_ + dz * reference_state.v;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(
      x_B_prototype, x_C, y_C, state_estimate.q);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm())) {
    const Eigen::Vector3d z_B_estimated =
        state_estimate.q * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm())) {
      y_B = y_C;
    } else {
      y_B.normalize();
    }
  } else {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  reference_command.orientation = Eigen::Quaterniond(R_W_B_ref);

  // Reference thrust
  reference_command.normalized_thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates and angular accelerations
  const double B1 = reference_command.normalized_thrust -
                    (dz - dx) * z_B.dot(reference_state.v);
  const double C1 = -(dx - dy) * y_B.dot(reference_state.v);
  const double D1 = x_B.dot(reference_state.jerk) +
                    dx * x_B.dot(reference_state.a);
  const double A2 = reference_command.normalized_thrust +
                    (dy - dz) * z_B.dot(reference_state.v);
  const double C2 = (dx - dy) * x_B.dot(reference_state.v);
  const double D2 = -y_B.dot(reference_state.jerk) -
                    dy * y_B.dot(reference_state.a);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = reference_state.head_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator)) {
    // reference_command.bodyrates = Eigen::Vector3d::Zero();
    // reference_command.angular_accelerations = Eigen::Vector3d::Zero();
  } else {
    // Compute body rates
    if (almostZero(A2)) {
      // reference_command.bodyrates.x() = 0.0;
    } else {
      // reference_command.bodyrates.x() =
          // (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          // (A2 * denominator);
    }
    // reference_command.bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
    // reference_command.bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;

    // Compute angular accelerations
    // const double thrust_dot = z_B.dot(reference_state.jerk) +
                              // reference_command.bodyrates.x() * (dy - dz) *
                                  // y_B.dot(reference_state.v) +
                              // reference_command.bodyrates.y() * (dz - dx) *
                                  // x_B.dot(reference_state.v) +
                              // dz * z_B.dot(reference_state.a);

    // const Eigen::Matrix3d omega_hat =
        // quadrotor_common::skew(reference_command.bodyrates);
    // const Eigen::Vector3d xi =
        // R_W_B_ref *
            // (omega_hat * omega_hat * D + D * omega_hat * omega_hat +
            //  2.0 * omega_hat * D * omega_hat.transpose()) *
            // R_W_B_ref.transpose() * reference_state.v +
        // 2.0 * R_W_B_ref * (omega_hat * D + D * omega_hat.transpose()) *
            // R_W_B_ref.transpose() * reference_state.a +
        // R_W_B_ref * D * R_W_B_ref.transpose() * reference_state.jerk;

    // const double E1 = x_B.dot(reference_state.snap) -
                      // 2.0 * thrust_dot * reference_command.bodyrates.y() -
                      // reference_command.normalized_thrust *
                          // reference_command.bodyrates.x() *
                          // reference_command.bodyrates.z() +
                      // x_B.dot(xi);
    // const double E2 = -y_B.dot(reference_state.snap) -
                      // 2.0 * thrust_dot * reference_command.bodyrates.x() +
                      // reference_command.normalized_thrust *
                          // reference_command.bodyrates.y() *
                          // reference_command.bodyrates.z() -
                      // y_B.dot(xi);
    // const double E3 = reference_state.heading_acceleration * x_C.dot(x_B) +
                      // 2.0 * reference_state.head_rate *
                          // (reference_command.bodyrates.z() * x_C.dot(y_B) -
                          //  reference_command.bodyrates.y() * x_C.dot(z_B)) -
                      // reference_command.bodyrates.x() *
                          // (reference_command.bodyrates.y() * y_C.dot(y_B) +
                          //  reference_command.bodyrates.z() * y_C.dot(z_B));

    if (almostZero(A2)) {
      // reference_command.angular_accelerations.x() = 0.0;
    } else {
      // reference_command.angular_accelerations.x() =
          // (-B1 * C2 * E3 + B1 * C3 * E2 - B3 * C1 * E2 + B3 * C2 * E1) /
          // (A2 * denominator);
    }
    // reference_command.angular_accelerations.y() =
        // (-C1 * E3 + C3 * E1) / denominator;
    // reference_command.angular_accelerations.z() =
        // (B1 * E3 - B3 * E1) / denominator;
  }

  // Transform reference rates and derivatives into estimated body frame
  const Eigen::Matrix3d R_trans =
      state_estimate.q.toRotationMatrix().transpose() * R_W_B_ref;
  // const Eigen::Vector3d bodyrates_ref = reference_command.bodyrates;
  // const Eigen::Vector3d angular_accelerations_ref =
      // reference_command.angular_accelerations;
  // const Eigen::Matrix3d bodyrates_est_hat =
      // quadrotor_common::skew(state_estimate.bodyrates);

  // reference_command.bodyrates = R_trans * bodyrates_ref;
  // reference_command.angular_accelerations =
      // R_trans * angular_accelerations_ref -
      // bodyrates_est_hat * R_trans * bodyrates_ref;

  *reference_inputs = reference_command;

  // Drag accelerations
  *drag_accelerations =
      -1.0 *
      (R_W_B_ref * (D * (R_W_B_ref.transpose() * reference_state.v)));
}