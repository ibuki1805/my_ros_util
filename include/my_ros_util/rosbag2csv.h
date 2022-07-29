#ifndef ROSBAG2CSV_H
#define ROSBAG2CSV_H

#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<tf2_msgs/TFMessage.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Time.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<my_ros_util/quaternion_euler_converter.h>
#include<fstream>
#include<sstream>
#include<geometry_msgs/TransformStamped.h>
// #include<boost/foreach.hpp>
// #define for_each BOOST_FOREACH

using MyROSUtil::QEConverter;
class ROSBag2CSV
{
public:
    ROSBag2CSV();
    ~ROSBag2CSV();
    void process();

private:
    void read_bag();
    void write_tf2csv(rosbag::Bag& bag);
    void write_imu2csv(rosbag::Bag& bag);
    void write_odom2csv(rosbag::Bag& bag);
    void write_poseWCS2csv(rosbag::Bag& bag);
    void write_pose2csv(const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pre_pose, std::ofstream& ofs);
    geometry_msgs::PoseStamped convert_to_footprint(const geometry_msgs::TransformStamped& tf);

    std::string bag_file_name_;
    std::string tf_topic_;
    std::string lookup_frame_id_;
    std::string tf2csv_file_name_;
    bool is_tf_topic_set_;

    std::string imu_topic_;
    std::string imu2csv_file_name_;
    bool is_imu_topic_set_;

    std::string odom_topic_;
    std::string odom2csv_file_name_;
    bool is_odom_topic_set_;

    std::string poseWCS_topic_;
    std::string poseWCS2csv_file_name_;
    bool is_poseWCS_topic_set_;

    bool write_footprint_;
    QEConverter *QE_converter_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
};

#endif // ROSBAG2CSV_H


