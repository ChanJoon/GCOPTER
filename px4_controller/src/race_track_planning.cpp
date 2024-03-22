#include "misc/visualizer.hpp"
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
#include "controller_msgs/FlatTarget.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <vector>
#include <algorithm>
#include <utility>
#include <xmlrpcpp/XmlRpcValue.h>

struct Config
{
    XmlRpc::XmlRpcValue waypoints;
    std::string mapTopic;
    std::string odomTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("WayPoints", waypoints);
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("OdomTopic", odomTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

class TrackPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    ros::Subscriber trajSub;
    ros::Subscriber odomSub;
    ros::Publisher trajPub;
    ros::Publisher cmdPub;
    ros::Publisher desOdomPub;
    ros::Publisher attPub;
    ros::Publisher flatReferencePub;
    ros::Timer controlTimer;

    bool mapInitialized;
    bool goalInitialized = false;
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
    TrackPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &TrackPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        odomSub = nh.subscribe(config.odomTopic, 1, &TrackPlanner::odomCallback, this,
																 ros::TransportHints().tcpNoDelay());
        trajSub = nh.subscribe("trajectory", 2, &TrackPlanner::rcvTrajectoryCallabck, this);
        trajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
        desOdomPub = nh.advertise<nav_msgs::Odometry>("desired_state", 50);
        // Position controller (Fast-Racing)
        cmdPub = nh.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);
        attPub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 50);

        // Geometric controller (mavros_controllers)
        flatReferencePub = nh.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
    }

    inline void setWaypoints()
    {
        if (mapInitialized)
        {
            startGoal.clear();
            // if (startGoal.size() == 0)
            // {
            //     const Eigen::Vector3d current(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
            //     if (voxelMap.query(current) == 0)
            //     {
            //         visualizer.visualizeStartGoal(current, 0.5, startGoal.size());
            //         startGoal.emplace_back(current);
            //     }
            //     else
            //     {
            //         ROS_WARN("Infeasible Hover Position !!!\n");
            //     }
            // }

            for (int i = 0; i < config.waypoints.size(); ++i)
            {
                Eigen::Vector3d goal;
                goal[0] = static_cast<double>(config.waypoints[i]["x"]);
                goal[1] = static_cast<double>(config.waypoints[i]["y"]);
                goal[2] = static_cast<double>(config.waypoints[i]["z"]);
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
            ROS_INFO("Selecting feasible waypoints is DONE!!\n Start Planning...");
            goalInitialized = true;
            plan();
        }
        return;
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
                pubflatrefState();
                pubDesiredState();

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
        traj_msg.header.frame_id = "odom";
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
            // MEMO: https://github.com/ZJU-FAST-Lab/Fast-tracker/blob/main/src/plan_manage/src/plan_manage.cpp
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
        pubflatrefState();
        pubDesiredState();
    }

    inline void pubflatrefState()
    {
        controller_msgs::FlatTarget msg;

        msg.header.stamp = odom.header.stamp;
        msg.header.frame_id = "map";
        msg.type_mask = 2;      // pubreference_type_;
        msg.position.x = cmd.position.x;
        msg.position.y = cmd.position.y;
        msg.position.z = cmd.position.z;
        msg.velocity.x = cmd.velocity.x;
        msg.velocity.y = cmd.velocity.y;
        msg.velocity.z = cmd.velocity.z;
        msg.acceleration.x = cmd.acceleration.x;
        msg.acceleration.y = cmd.acceleration.y;
        msg.acceleration.z = cmd.acceleration.z;
        flatReferencePub.publish(msg);
    }
    
    inline void pubDesiredState()
    {
        nav_msgs::Odometry msg;

        msg.header.stamp = cmd.header.stamp;
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = cmd.position.x;
        msg.pose.pose.position.y = cmd.position.y;
        msg.pose.pose.position.z = cmd.position.z;

        Eigen::Vector3d rpy(0, 0, cmd.yaw);
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        desOdomPub.publish(msg);
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
        if (!goalInitialized) setWaypoints();
    }

    inline void plan()
    {
        std::vector<Eigen::Vector3d> routes;
        std::vector<Eigen::MatrixX4d> hPolys;
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);

        std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
        for (unsigned int i = 0; i < startGoal.size() - 1; i++)
        {
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[i],
                                                    startGoal[i+1],
                                                    voxelMap.getOrigin(),
                                                    voxelMap.getCorner(),
                                                    &voxelMap, 0.01,
                                                    route);
            routes.insert(routes.end(), route.begin(), route.end());

            std::vector<Eigen::MatrixX4d> htemp;
            sfc_gen::convexCover(route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                7.0,
                                3.0,
                                htemp);
            sfc_gen::shortCut(htemp);
            hPolys.insert(hPolys.end(), htemp.begin(), htemp.end());
        }
        std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
        double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
        std::cout << "Safe fight corridor time usage: " << compTime << " ms" << std::endl;

        if (routes.size() > 1)
        {
            visualizer.visualizePolytope(hPolys);

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

            Eigen::Matrix3d iniState;
            Eigen::Matrix3d finState;
            iniState << routes.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            finState << routes.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

            ROS_INFO("Begin to optimize the traj~");
            std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
            if (!gcopter.setup(config.weightT,
                               iniState, finState,
                               hPolys, INFINITY,
                               config.smoothingEps,
                               quadratureRes,
                               magnitudeBounds,
                               penaltyWeights,
                               physicalParams,
                               0, Eigen::Vector3d::Zero()))
            {
                return;
            }
            std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
            double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
            std::cout << "GCOPTER SETUP DONE! Setup time usage: " << compTime << " ms\n" << std::endl;

            if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
            {
                return;
            }
            std::chrono::high_resolution_clock::time_point tac = std::chrono::high_resolution_clock::now();
            compTime = std::chrono::duration_cast<std::chrono::microseconds>(tac - toc).count() * 1.0e-3;
            std::cout << "GCOPTER OPTIMIZATION DONE! Optimization time usage: " << compTime << " ms\n" << std::endl;

            compTime = std::chrono::duration_cast<std::chrono::microseconds>(tac - tic).count() * 1.0e-3;
            printf("finished!!!\n");
            std::cout << "Total time usage: " << compTime << " ms" << std::endl;
            std::cout << "Maximum Vel: " << traj.getMaxVelRate() << std::endl;
            std::cout << "Maximum Acc: " << traj.getMaxAccRate() << std::endl;
            std::cout << "Total Duation: " << traj.getTotalDuration() << std::endl;

            if (traj.getPieceNum() > 0)
            {
                trajStamp = ros::Time::now().toSec();
                visualizer.visualize(traj, routes);
            }
            traj_msg = traj2msg(traj);
            trajPub.publish(traj_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "race_track_planning_node");
    ros::NodeHandle nh_;

    TrackPlanner race_track_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
