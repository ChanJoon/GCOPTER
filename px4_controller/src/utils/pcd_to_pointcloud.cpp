#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <sstream>

class pcd_to_pointcloud {
    ros::NodeHandle nh;
    std::string cloud_topic;
    std::string file_name;
    double interval;
    std::string frame_id;
    sensor_msgs::PointCloud2 cloud;
    ros::Publisher pub;
    ros::Timer timer;

    void timer_callback(ros::TimerEvent const&) {
        cloud.header.stamp = ros::Time::now();
        pub.publish(cloud);
    }

public:
    pcd_to_pointcloud(ros::NodeHandle nh_)
    : nh(nh_), cloud_topic("gt_map"), interval(0.1), frame_id("map")
    {
        cloud_topic = nh.resolveName(cloud_topic);
        nh.getParam("/file_name", file_name);
        pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
        timer = nh.createTimer(ros::Duration(interval), &pcd_to_pointcloud::timer_callback, this);
    }

    bool load_pcd_file() {
        if (file_name.empty()) {
            ROS_ERROR_STREAM("Can't load pointcloud: no file name provided");
            return false;
        }
        else if (pcl::io::loadPCDFile(file_name, cloud) < 0) {
            ROS_ERROR_STREAM("Failed to parse pointcloud from file ('" << file_name << "')");
            return false;
        }
        cloud.header.frame_id = frame_id;
        return true;
    }

    void print_info() {
        ROS_INFO_STREAM("Loaded " << file_name << " with " << cloud.width * cloud.height << " points and " << pcl::getFieldsList(cloud) << "channels");
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "pcd_to_pointcloud");
    ros::NodeHandle nh;
    pcd_to_pointcloud node(nh);
    if (!node.load_pcd_file()) {
        return -1;
    }
    node.print_info();
    ros::spin();
}