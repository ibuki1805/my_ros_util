#ifndef ROSBAG2CSV_H
#define ROSBAG2CSV_H

#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<tf2_msgs/TFMessage.h>
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
    void write_CSV(const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pre_pose);
    geometry_msgs::PoseStamped convert_to_footprint(const geometry_msgs::TransformStamped& tf);

    std::string bag_file_name_;
    std::string csv_file_name_;
    std::string lookup_frame_id_;
    QEConverter *QE_converter_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
};

#endif // ROSBAG2CSV_H


