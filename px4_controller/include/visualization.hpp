#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <gcopter/trajectory.hpp>
#include <gcopter/quickhull.hpp>
#include <gcopter/geo_utils.hpp>

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

    double modelType;
    double horizHalfLen;
    double vertHalfLen;
    Eigen::Vector3d ellipsoid;
    std::vector<double> vecRectangularBox;
    Eigen::Vector3d cuboid;
    // TODO Add parsing 2D vector with XmlRpc or yaml-cpp feature for polyhedron vertices
    // https://wiki.ros.org/roscpp/Overview/Parameter%20Server#Retrieving_Lists
    // https://answers.ros.org/question/212372/get-the-name-of-a-sub-parameter-not-the-whole-structure/#212523
    Eigen::VectorXd polyhedronVertices;

    // TODO
    std::string packageUrl = "package://px4_controller";
    std::string odomFrame;
    double trajVizWidth;
    std::vector<double> vecTrajVizRGB, vecEllipsoidVizRGBA;
    Eigen::Vector3d trajVizRGB;
    std::string ellipsoidPath;
    Eigen::Vector4d ellipsoidVizRGBA;
    std::string quadrotorPath;

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

        nh_priv.getParam("ModelType", modelType);
        nh_priv.getParam("HorizHalfLen", horizHalfLen);
        nh_priv.getParam("VertHalfLen", vertHalfLen);
        ellipsoid << horizHalfLen, horizHalfLen, vertHalfLen;
        nh_priv.getParam("RectangularBox", vecRectangularBox);
        cuboid << vecRectangularBox[0], vecRectangularBox[1], vecRectangularBox[2];

        // TODO
        nh_priv.getParam("OdomFrame", odomFrame);
        nh_priv.getParam("TrajVizWidth", trajVizWidth);
        nh_priv.getParam("TrajVizRGB", vecTrajVizRGB);
        trajVizRGB << vecTrajVizRGB[0], vecTrajVizRGB[1], vecTrajVizRGB[2];
        nh_priv.getParam("EllipsoidPath", ellipsoidPath);
        ellipsoidPath = packageUrl + ellipsoidPath;
        nh_priv.getParam("EllipsoidVizRGBA", vecEllipsoidVizRGBA);
        ellipsoidVizRGBA << vecEllipsoidVizRGBA[0], vecEllipsoidVizRGBA[1], vecEllipsoidVizRGBA[2], vecEllipsoidVizRGBA[3];
        nh_priv.getParam("QuadrotorPath", quadrotorPath);
        quadrotorPath = packageUrl + quadrotorPath;
    }
};

// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    Config config;
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher spherePub;

public:
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;
    ros::Publisher ellipsoidPub;
    ros::Publisher quadrotorPub;

public:
    Visualizer(ros::NodeHandle &nh_, Config &conf)
        : nh(nh_), config(conf)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);

        ellipsoidPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/ellipsoid", 1);
        quadrotorPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/quadrotor", 1);    
    }

    // Visualize the trajectory and its front-end path
    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.30;

        if (route.size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub.publish(routeMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }
    }

    // Visualize some polytopes in H-representation
    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {

        // Due to the fact that H-representation cannot be directly visualized
        // We first conduct vertex enumeration of them, then apply quickhull
        // to obtain triangle meshs of polyhedra
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
        for (size_t id = 0; id < hPolys.size(); id++)
        {
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPolys[id], vPoly);

            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // RVIZ support tris for visualization
        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.00;
        meshMarker.color.g = 0.00;
        meshMarker.color.b = 1.00;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;
        edgeMarker.color.b = 1.00;
        edgeMarker.color.a = 1.00;
        edgeMarker.scale.x = 0.02;

        geometry_msgs::Point point;

        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        meshPub.publish(meshMarker);
        edgePub.publish(edgeMarker);

        return;
    }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "map";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "map";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub.publish(sphereDeleter);
            ros::Duration(1.0e-9).sleep();
            sphereMarkers.header.stamp = ros::Time::now();
        }
        spherePub.publish(sphereMarkers);
    }

    template <int D>
    inline void visualizeEllipsoid(const Trajectory<D> &traj, const int samples)
    {
        visualization_msgs::Marker ellipsoidMarker;
        visualization_msgs::MarkerArray ellipsoidMarkers;

        ellipsoidMarker.id = 0;
        ellipsoidMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        ellipsoidMarker.mesh_resource = config.ellipsoidPath;
        ellipsoidMarker.header.stamp = ros::Time::now();
        ellipsoidMarker.header.frame_id = config.odomFrame;
        ellipsoidMarker.pose.orientation.w = 1.00;
        ellipsoidMarker.action = visualization_msgs::Marker::ADD;
        ellipsoidMarker.ns = "ellipsoids";
        ellipsoidMarker.color.r = config.ellipsoidVizRGBA(0);
        ellipsoidMarker.color.g = config.ellipsoidVizRGBA(1);
        ellipsoidMarker.color.b = config.ellipsoidVizRGBA(2);
        ellipsoidMarker.color.a = config.ellipsoidVizRGBA(3);
        ellipsoidMarker.scale.x = config.horizHalfLen * 2.0;
        ellipsoidMarker.scale.y = config.horizHalfLen * 2.0;
        ellipsoidMarker.scale.z = config.vertHalfLen * 2.0;

        ellipsoidMarker.action = visualization_msgs::Marker::DELETEALL;
        ellipsoidMarkers.markers.push_back(ellipsoidMarker);
        ellipsoidPub.publish(ellipsoidMarkers);
        ellipsoidMarker.action = visualization_msgs::Marker::ADD;
        ellipsoidMarkers.markers.clear();

        double dt = traj.getTotalDuration() / samples;
        geometry_msgs::Point point;
        Eigen::Vector3d pos;
        Eigen::Matrix3d rotM;
        Eigen::Quaterniond quat;
        for (int i = 0; i <= samples; i++)
        {
            pos = traj.getPos(dt * i);
            ellipsoidMarker.pose.position.x = pos(0);
            ellipsoidMarker.pose.position.y = pos(1);
            ellipsoidMarker.pose.position.z = pos(2);
            traj.getRotation(dt * i, 0.0, config.gravAcc, rotM);
            quat = Eigen::Quaterniond(rotM);
            ellipsoidMarker.pose.orientation.w = quat.w();
            ellipsoidMarker.pose.orientation.x = quat.x();
            ellipsoidMarker.pose.orientation.y = quat.y();
            ellipsoidMarker.pose.orientation.z = quat.z();
            ellipsoidMarkers.markers.push_back(ellipsoidMarker);
            ellipsoidMarker.id++;
        }

        ellipsoidPub.publish(ellipsoidMarkers);
    }

    template <int D>
    inline void visualizeQuadrotor(const Trajectory<D> &traj, const int samples)
    {
        visualization_msgs::Marker quadrotorMarker;
        visualization_msgs::MarkerArray quadrotorMarkers;

        quadrotorMarker.id = 0;
        quadrotorMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        quadrotorMarker.mesh_use_embedded_materials = true;
        quadrotorMarker.mesh_resource = config.quadrotorPath;
        quadrotorMarker.header.stamp = ros::Time::now();
        quadrotorMarker.header.frame_id = config.odomFrame;
        quadrotorMarker.pose.orientation.w = 1.00;
        quadrotorMarker.action = visualization_msgs::Marker::ADD;
        quadrotorMarker.ns = "quadrotor";
        quadrotorMarker.color.r = 0.0;
        quadrotorMarker.color.g = 0.0;
        quadrotorMarker.color.b = 0.0;
        quadrotorMarker.color.a = 0.0;
        quadrotorMarker.scale.x = (config.horizHalfLen) * sqrt(2.0);
        quadrotorMarker.scale.y = (config.horizHalfLen) * sqrt(2.0);
        quadrotorMarker.scale.z = config.vertHalfLen * 8.0;

        quadrotorMarker.action = visualization_msgs::Marker::DELETEALL;
        quadrotorMarkers.markers.push_back(quadrotorMarker);
        quadrotorPub.publish(quadrotorMarkers);
        quadrotorMarker.action = visualization_msgs::Marker::ADD;
        quadrotorMarkers.markers.clear();

        double dt = traj.getTotalDuration() / samples;
        //hzc30
        // double dt = 5.0/samples;
        geometry_msgs::Point point;
        Eigen::Vector3d pos;
        Eigen::Matrix3d rotM;
        Eigen::Quaterniond quat;
        for (int i = 0; i <= samples; i++)
        {
            // pos = traj.getPos(6+dt * i);//4.5
            pos = traj.getPos(dt * i);
            quadrotorMarker.pose.position.x = pos(0);
            quadrotorMarker.pose.position.y = pos(1);
            quadrotorMarker.pose.position.z = pos(2);
            // traj.getRotation(6+dt * i, M_PI_4, config.gravAcc, rotM);hzc
            // traj.getRotation(6+dt * i, M_PI_2, config.gravAcc, rotM);
            traj.getRotation(dt * i, M_PI_2, config.gravAcc, rotM);
            quat = Eigen::Quaterniond(rotM);
            quadrotorMarker.pose.orientation.w = quat.w();
            quadrotorMarker.pose.orientation.x = quat.x();
            quadrotorMarker.pose.orientation.y = quat.y();
            quadrotorMarker.pose.orientation.z = quat.z();
            quadrotorMarkers.markers.push_back(quadrotorMarker);
            quadrotorMarker.id++;
        }

        quadrotorPub.publish(quadrotorMarkers);
    }
};

#endif