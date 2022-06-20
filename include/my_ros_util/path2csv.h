#ifndef PATH2CSV_H
#define PATH2CSV_H

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<fstream>
#include<sstream>
#include<my_ros_util/quaternion_euler_converter.h>
class Path2CSV
{
public:
    Path2CSV();
    ~Path2CSV();
    void process();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void write_path2csv(const nav_msgs::Path &path, const int &lookup_length);

    int hz_;
    std::string path_topic_;
    std::string csv_file_name_;
    nav_msgs::Path path_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;

};

#endif // PATH2CSV_H
