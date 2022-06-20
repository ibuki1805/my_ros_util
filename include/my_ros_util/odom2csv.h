#ifndef odom2CSV_H
#define odom2CSV_H

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<fstream>
#include<sstream>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>
class Odom2CSV
{
public:
    Odom2CSV();
    ~Odom2CSV();
    void process();

private:
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void write_odom2csv(const nav_msgs::Odometry::ConstPtr &odom, const nav_msgs::Odometry  &pre_odom);
    void quaternion2euler(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw);

    int hz_;
    std::string odom_topic_;
    std::string csv_file_name_;
    nav_msgs::Odometry odom_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_odom_;

};

#endif // odom2CSV_H
