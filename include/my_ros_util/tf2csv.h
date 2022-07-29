#ifndef TF2CSV_H
#define TF2CSV_H

#include<ros/ros.h>
#include<tf2_msgs/TFMessage.h>
#include<fstream>
#include<sstream>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>
#include<my_ros_util/quaternion_euler_converter.h>

using MyROSUtil::QEConverter;

class TF2CSV
{
public:
    TF2CSV();
    ~TF2CSV();
    void process();

private:
    void TF_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void write_TF2CSV(const geometry_msgs::PoseStamped& pose);

    int hz_;
    std::string lookup_frame_id_;
    std::string world_frame_id_;
    std::string csv_file_name_;
    QEConverter *qe_converter_;

    geometry_msgs::PoseStamped pre_pose_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub_head_pose_;
    ros::Publisher pub_footprint_pose_;
    ros::Subscriber sub_TF_;
    tf2_ros::Buffer TF_buffer_;
    tf2_ros::TransformListener TF_listener_;

};

#endif // odom2CSV_H
