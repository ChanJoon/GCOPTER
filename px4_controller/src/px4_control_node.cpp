#include <ros/ros.h>
#include "px4_control.h"

//#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <std_msgs/Header.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <signal.h>
#include "std_msgs/Float32.h"

Control* pFSM;

void mySigintHandler(int sig) {
    ROS_INFO("[Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "Ctrl");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);
    Parameter_t param;
    Controller controller(param);
    HovThrKF hov_thr_kf(param);
    Control fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    param.config_from_ros_handle(nh);
    param.init();//recompute the full thrust.
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);//x(0) = hov_percent
    
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    fsm.controller.config();//

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("state", 
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    ros::Subscriber cmd_sub = 
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu",
                                         100,
                                         boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    fsm.controller.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    // fsm.controller.debug_roll_pub = nh.advertise<std_msgs::Float32>("/debug_roll",10);
    // fsm.controller.debug_pitch_pub = nh.advertise<std_msgs::Float32>("/debug_pitch",10);
    fsm.controller.set_FCU_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.controller.cmd_FCU_arming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.des_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    // fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    ros::Rate r(param.ctrl_rate);
    // ---- process ----
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        if(fsm.px4_init()){
            fsm.process();
        }
        else
            ROS_WARN("px4 not ready!!!");
    }

    return 0;
}
