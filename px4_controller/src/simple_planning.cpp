#include "visualization.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "px4_controller/Command.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <vector>
#include <algorithm>

class SimplePlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    ros::Subscriber trajSub;
		ros::Subscriber odomSub;
		ros::ServiceServer targetSrv;
    ros::Publisher trajPub;
    ros::Publisher cmdPub;
    ros::Publisher attPub;
    ros::Timer controlTimer;

    bool mapInitialized;
    bool odomInitialized = false;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

    quadrotor_msgs::PolynomialTrajectory traj_msg;
    quadrotor_msgs::PositionCommand cmd;
    nav_msgs::Odometry init_odom;
    nav_msgs::Odometry odom;

    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    std::vector<int> _order;
    std::vector<Eigen::MatrixXd> _normalizedcoeflist;

    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;
    Eigen::VectorXd _time;

    double mag_coeff;
    double _start_yaw = 0.0, _final_yaw = 0.0;

    enum ServerState{INIT = 0, TRAJ, HOVER} state = INIT;;

public:
    SimplePlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh, config)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &SimplePlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &SimplePlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
				odomSub = nh.subscribe(config.odomTopic, 1, &SimplePlanner::odomCallback, this,
																 ros::TransportHints().tcpNoDelay());
        trajSub = nh.subscribe("trajectory", 2, &SimplePlanner::rcvTrajectoryCallabck, this);
        trajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
        cmdPub = nh.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);
        attPub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 50);
        // controlTimer = nh.createTimer(ros::Duration(0.01), &SimplePlanner::pubPositionCommand, this); // TODO: 이게 문제일 수 있다...
				targetSrv = nh.advertiseService("target_srv", &SimplePlanner::targetSrvCallBack, this);
    }

		inline void odomCallback(const nav_msgs::Odometry &msg)
		{
			if (!odomInitialized)
			{
				init_odom = msg;
				odomInitialized = true;
			}
			else
			{
				odom = msg;
				if(state == INIT )
				{
						cmd.position.x = init_odom.pose.pose.position.x;
						cmd.position.y = init_odom.pose.pose.position.y;
						cmd.position.z = 3.0;		// Temporailiy set to 3.0 m
						
						cmd.header.stamp = odom.header.stamp;
						cmd.header.frame_id = "/world_enu";
						//cmd.trajectory_flag = _traj_flag;
						cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

						cmd.velocity.x = 0.0;
						cmd.velocity.y = 0.0;
						cmd.velocity.z = 0.0;
						
						cmd.acceleration.x = 0.0;
						cmd.acceleration.y = 0.0;
						cmd.acceleration.z = 0.0;

						cmd.jerk.x = 0.0;
						cmd.jerk.y = 0.0;
						cmd.jerk.z = 0.0;
						cmd.yaw = acos(-1)/2;	// == PI / 2
						cmdPub.publish(cmd);

						return;
				}

				// change the order between #2 and #3. zxzxzxzx
				
				// #2. try to calculate the new state
				if (state == TRAJ && ( (odom.header.stamp - _start_time).toSec() / mag_coeff > (_final_time - _start_time).toSec() ) )
				{
						state = HOVER;
						_traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
				}

				// #3. try to publish command
				pubPositionCommand();
			}
		}

    inline quadrotor_msgs::PolynomialTrajectory traj2msg(Trajectory<5> traj)
    {
        static int count=0;
        quadrotor_msgs::PolynomialTrajectory traj_msg;
        traj_msg.header.seq = count;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "map";
        traj_msg.trajectory_id = count;
        traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
        traj_msg.num_order = traj[0].getDegree(); // the order of polynomial
        traj_msg.num_segment = traj.getPieceNum();
        traj_msg.start_yaw = 0;
        traj_msg.final_yaw = 0;
        for(unsigned int i=0; i<traj_msg.num_segment; i++)
        {
            for (unsigned int j = 0; j <= traj[i].getDegree(); j++)
            {
                Piece<5>::CoefficientMat coemat = traj[i].normalizePosCoeffMat();
                traj_msg.coef_x.push_back(coemat(0,j));
                traj_msg.coef_y.push_back(coemat(1,j));
                traj_msg.coef_z.push_back(coemat(2,j));
            }
            traj_msg.time.push_back(traj[i].getDuration());
            traj_msg.order.push_back(traj[i].getDegree());
        }
        traj_msg.mag_coeff = 1;
        count++;
        return traj_msg;
    }

    inline void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory & traj)
    {
        // #1. try to execuse the action
        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
        {
            state = TRAJ;
            _traj_id = traj.trajectory_id;
            _n_segment = traj.num_segment;
            _final_time = _start_time = traj.header.stamp;
            _time.resize(_n_segment);

            _order.clear();
            _normalizedcoeflist.clear();
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                _final_time += ros::Duration(traj.time[idx]);
                _time(idx) = traj.time[idx];
                _order.push_back(traj.order[idx]);
            }

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;
            mag_coeff  = traj.mag_coeff;
            
            //ROS_WARN("stack the coefficients");
            int shift = 0;
            for (int idx = 0; idx < _n_segment; idx++)
            {     
                int order = traj.order[idx];
                Eigen::MatrixXd coefmat;
                coefmat = Eigen::MatrixXd::Zero(3,order+1);

                for (int j = 0; j <= order; j++)
                {
                    coefmat(0, j) = traj.coef_x[shift + j];
                    coefmat(1, j) = traj.coef_y[shift + j];
                    coefmat(2, j) = traj.coef_z[shift + j];
                }
                _normalizedcoeflist.push_back(coefmat);
                shift += (order + 1);
            }
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) 
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }
        else if(traj.action==quadrotor_msgs::PositionCommand::ACTION_STOP){
            ROS_WARN("Emergency!!!");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }

    inline void pubPositionCommand()
    {
        // #1. check if it is right state
        if (state == INIT) return;
        if (state == HOVER)
        {
            if (cmd.header.frame_id != "/world_enu"){
                cmd.position = odom.pose.pose.position;
            }
           
           // cmd.yaw = odom.pose.pose.orientation.;
            // cmd.yaw =hover_yaw;
            //hzc
                
            cmd.header.stamp = odom.header.stamp;
            cmd.header.frame_id = "/world_enu";
            cmd.trajectory_flag = _traj_flag;

            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;

            cmd.jerk.x = 0.0;
            cmd.jerk.y = 0.0;
            cmd.jerk.z = 0.0;
        }
        // #2. locate the trajectory segment
        if (state == TRAJ)
        {
            cmd.header.stamp = odom.header.stamp;
            if (cmd.header.stamp > _final_time)
                state = HOVER;

            cmd.header.frame_id = "/world_enu";
            cmd.trajectory_flag = _traj_flag;
            cmd.trajectory_id = _traj_id;

            double t = std::max(0.0, (odom.header.stamp - _start_time).toSec());// / mag_coeff;;
        // #3. calculate the desired states
            int seg_idx;
            double dur;
            for (seg_idx = 0;
             seg_idx < _n_segment &&
             t > (dur = _time[seg_idx]);
             seg_idx++)
            {
                t -= dur;
            }
            if (seg_idx == _n_segment)
            {
                seg_idx--;
                t += _time[seg_idx];
            }
            t /= _time[seg_idx];


            const int cur_order = _order[seg_idx];
            const int cur_poly_num = cur_order + 1;
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            Eigen::Vector3d jerk(0.0,0.0,0.0);

            double tn = 1.0,tnvel=1.0,tnacc=1.0,tnjerk = 1.0;
            int n=1,k=1,l=2,j_1=1,j_2=2,j_3=3;
            for (int i = cur_order; i >= 0; i--)
            {
                pos += tn * _normalizedcoeflist[seg_idx].col(i);
                tn *= t;
                if(i<=cur_order-1){
                    vel+=n*tnvel*_normalizedcoeflist[seg_idx].col(i);
                    tnvel*=t;
                    n++;
                    if(i<=cur_order-2){
                        acc+=l*k*tnacc*_normalizedcoeflist[seg_idx].col(i);
                        tnacc*=t;
                        l++;
                        k++;
                        if(i<=cur_order-3){
                            jerk+=j_1*j_2*j_3*tnjerk*_normalizedcoeflist[seg_idx].col(i);
                            tnjerk*=t;
                            j_1++;
                            j_2++;
                            j_3++;
                        }
                    }
                }
            }
            vel/=_time[seg_idx];
            acc/=(_time[seg_idx]*_time[seg_idx]);
            jerk/=(_time[seg_idx]*_time[seg_idx]*_time[seg_idx]);

            cmd.position.x = pos[0];
            cmd.position.y = pos[1];
            cmd.position.z = pos[2];
            cmd.velocity.x = vel[0];
            cmd.velocity.y = vel[1];
            cmd.velocity.z = vel[2];
            cmd.acceleration.x = acc[0];
            cmd.acceleration.y = acc[1];
            cmd.acceleration.z = acc[2];   

            cmd.jerk.x = jerk[0];
            cmd.jerk.y = jerk[1];
            cmd.jerk.z = jerk[2]; 
            // cmd.yaw = atan2(cmd.velocity.y, cmd.velocity.x);
            cmd.yaw = acos(-1)/2; // PI / 2
            cmd.yaw_dot = 0.01;
        }
        // #4. just publish
        cmdPub.publish(cmd);
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
						ROS_INFO("Map initialized!");
        }
    }

    inline void plan()
    {
			// TODO: Instead of calling in 'targetCallback' function, Use given waypoints in config file.
        if (startGoal.size() == 2)
        {
					/* TODO: To use multiple goal points, This section should be in the loop */
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
                ROS_INFO("Begin to optimize the traj~");
                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams,
                                   config.modelType, config.cuboid, config.ellipsoid))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }
                std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
                double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;

                printf("finished!!!\n");
                std::cout << "Optimization time usage: " << compTime << " ms" << std::endl;
                std::cout << "Maximum Vel: " << traj.getMaxVelRate() << std::endl;
                std::cout << "Maximum Acc: " << traj.getMaxAccRate() << std::endl;
                std::cout << "Total Duation: " << traj.getTotalDuration() << std::endl;

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);
                }
                traj_msg = traj2msg(traj);
                trajPub.publish(traj_msg);
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
					startGoal.clear();
					const double zGoal = config.mapBound[4] + config.dilateRadius +
															 fabs(msg->pose.orientation.z) *
															 (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
					const Eigen::Vector3d current(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
					const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
					ROS_INFO("GOAL Selected! x: %f, y: %f, z: %f\n", goal[0], goal[1], goal[2]);
					if (voxelMap.query(current) == 0)
					{
						visualizer.visualizeStartGoal(current, 0.5, startGoal.size());
						startGoal.emplace_back(current);
						if (voxelMap.query(goal) == 0)
						{
								visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
								startGoal.emplace_back(goal);
						}
						else
						{
								ROS_WARN("Infeasible Position Selected !!!\n");
						}
					}
					else
					{
							ROS_WARN("Infeasible Hover Position !!!\n");
					}

					plan();
        }
        return;
    }

		inline bool targetSrvCallBack(px4_controller::Command::Request &req,
																	px4_controller::Command::Response &res)
		{
			if (mapInitialized)
			{
					startGoal.clear();
					const double zGoal = config.mapBound[4] + config.dilateRadius +
																fabs(odom.pose.pose.orientation.z) *
																		(config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
					const Eigen::Vector3d current(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
					const Eigen::Vector3d goal(req.x, req.y, zGoal);
					ROS_INFO("GOAL Received! x: %f, y: %f, z: %f\n", goal[0], goal[1], goal[2]);
					if (voxelMap.query(current) == 0)
					{
						visualizer.visualizeStartGoal(current, 0.5, startGoal.size());
						startGoal.emplace_back(current);
						if (voxelMap.query(goal) == 0)
						{
								visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
								startGoal.emplace_back(goal);
						}
						else
						{
								ROS_WARN("Infeasible Position Received !!!\n");
						}
					}
					else
					{
							ROS_WARN("Infeasible Hover Position !!!\n");
					}

					plan();
			}
			return true;
		}

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);

							// Open-loop control (No Feedback)
                // mavros_msgs::AttitudeTarget attMsg;
                // attMsg.header.stamp = ros::Time::now();
                // attMsg.header.frame_id = std::string("FCU");
                // attMsg.thrust = thr;
                // attMsg.body_rate.x = omg(0);
                // attMsg.body_rate.y = omg(1);
                // attMsg.body_rate.z = omg(2);
                // attPub.publish(attMsg);   
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_planning_node");
    ros::NodeHandle nh_;

    SimplePlanner simple_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        simple_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
