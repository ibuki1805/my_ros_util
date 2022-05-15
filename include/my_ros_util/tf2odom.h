#ifndef TF2ODOM_H
#define TF2ODOM_H

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<nav_msgs/Path.h>

class TF2Odom
{
public:
    TF2Odom();
    void process();

private:
    int hz_;
    bool show_path=false;
    std::string

    void convert2odom();
    void convert2path();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub_path_;
    ros::Publisher pub_odom_;


#endif //TF2ODOM_H
