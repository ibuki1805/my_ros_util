#include<my_ros_util/rosbag2csv.h>

ROSBag2CSV::ROSBag2CSV(): private_nh_("~")
{
    if(private_nh_.hasParam("bag_file_name"))
        private_nh_.getParam("bag_file_name", bag_file_name_);
    else
    {
        ROS_ERROR("bag_file_name is not set");
        ros::shutdown();
    }

    if(private_nh_.hasParam("tf_topic"))
    {
        private_nh_.getParam("tf_topic", tf_topic_);
        is_tf_topic_set_ = true;
        if(private_nh_.hasParam("lookup_frame_id"))
            private_nh_.getParam("lookup_frame_id", lookup_frame_id_);
        else
        {
            ROS_ERROR("lookup_frame_id is not set");
            ros::shutdown();
        }
        if(private_nh_.hasParam("tf2csv_file_name"))
            private_nh_.getParam("tf2csv_file_name", tf2csv_file_name_);
        else
        {
            ROS_ERROR("tf2csv_file_name is not set");
            ros::shutdown();
        }
    }
    else is_tf_topic_set_ = false;

    if(private_nh_.hasParam("imu_topic"))
    {
        private_nh_.getParam("imu_topic", imu_topic_);
        is_imu_topic_set_ = true;
        if(private_nh_.hasParam("imu2csv_file_name"))
            private_nh_.getParam("imu2csv_file_name", imu2csv_file_name_);
        else
        {
            ROS_ERROR("imu2csv_file_name is not set");
            ros::shutdown();
        }
    }
    else is_imu_topic_set_ = false;

    if(private_nh_.hasParam("odom_topic"))
    {
        private_nh_.getParam("odom_topic", odom_topic_);
        is_odom_topic_set_ = true;
        if(private_nh_.hasParam("odom2csv_file_name"))
            private_nh_.getParam("odom2csv_file_name", odom2csv_file_name_);
        else
        {
            ROS_ERROR("odom2csv_file_name is not set");
            ros::shutdown();
        }
    }
    else is_odom_topic_set_ = false;

    if(private_nh_.hasParam("poseWCS_topic"))
    {
        private_nh_.getParam("poseWCS_topic", poseWCS_topic_);
        is_poseWCS_topic_set_ = true;
        if(private_nh_.hasParam("poseWCS2csv_file_name"))
            private_nh_.getParam("poseWCS2csv_file_name", poseWCS2csv_file_name_);
        else
        {
            ROS_ERROR("poseWCS2csv_file_name is not set");
            ros::shutdown();
        }
    }
    else is_poseWCS_topic_set_ = false;

    private_nh_.param<bool>("write_footprint", write_footprint_, false);
}

ROSBag2CSV::~ROSBag2CSV(){}

geometry_msgs::PoseStamped ROSBag2CSV::convert_to_footprint(const geometry_msgs::TransformStamped& tf)
{
    geometry_msgs::PoseStamped footprint_pose;
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.pose.position.z = -1.2;
    pose_in.pose.orientation.w = 1;
    pose_in.header.frame_id = lookup_frame_id_;
    tf2::doTransform(pose_in, pose_out, tf);
    return pose_out;
}

void ROSBag2CSV::write_pose2csv(const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pre_pose, std::ofstream& ofs)
{
    std::stringstream ss;
    ss << pose.header.seq << ",";
    ss << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ",";

    double dt = ros::Duration(pose.header.stamp - pre_pose.header.stamp).toSec();
    double vx = (pose.pose.position.x - pre_pose.pose.position.x) / dt;
    double vy = (pose.pose.position.y - pre_pose.pose.position.y) / dt;
    double v = std::hypot(vx, vy);
    ss << vx << "," << vy << "," << v << ",";

    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = pose.pose.orientation;
    QE_converter_->quaternion2euler(pose.pose.orientation, roll, pitch, yaw);
    ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
    ss << pose.header.stamp << "," << dt << "," << pose.header.frame_id << ",";
    // ss << "\n";
    ofs<< ss.str().c_str();
    pre_pose = pose;


}

void ROSBag2CSV::write_tf2csv(rosbag::Bag& bag)
{
    std::ofstream ofs(tf2csv_file_name_, std::ios::out);
    ofs << "head_seq,x,y,z,vx,vy,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,time,dt,frame_id,";
    if(write_footprint_) ofs << "FT_seq,x,y,z,vx,vy,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,time,dt,frame_id,";
    ofs << "\n";
    static geometry_msgs::PoseStamped pre_head_pose_, pre_footprint_pose_;
    pre_head_pose_.pose.orientation.w = 1;
    if(write_footprint_)
    {
        static geometry_msgs::PoseStamped pre_footprint_pose_;
        pre_footprint_pose_.pose.orientation.w = 1;
    }

    for(const rosbag::MessageInstance& msg : rosbag::View(bag, rosbag::TopicQuery(tf_topic_)))
    {
        tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
        for(const auto& tf : tf_msg->transforms)
        {
            if(tf.child_frame_id == lookup_frame_id_)
            {
                geometry_msgs::PoseStamped head_pose;
                head_pose.header = tf.header;
                head_pose.pose.position.x = tf.transform.translation.x;
                head_pose.pose.position.y = tf.transform.translation.y;
                head_pose.pose.position.z = tf.transform.translation.z;
                head_pose.pose.orientation = tf.transform.rotation;
                write_pose2csv(head_pose, pre_head_pose_, ofs);

                if(write_footprint_)
                {
                    geometry_msgs::PoseStamped footprint_pose = convert_to_footprint(tf);
                    write_pose2csv(footprint_pose, pre_footprint_pose_, ofs);
                }
                ofs<< "\n";
            }
        }
    }
}

void ROSBag2CSV::write_imu2csv(rosbag::Bag& bag)
{
    std::ofstream ofs(imu2csv_file_name_, std::ios::out);
    ofs << "seq,ax,ay,az,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,roll_rate,roll_rate_deg,pitch_rate,pitch_rate_deg,yaw_rate,yaw_rate_deg,time,dt,frame_id,";
    ofs << "\n";
    static ros::Time pre_stamp = ros::Time::now();

    for(const rosbag::MessageInstance& msg : rosbag::View(bag, rosbag::TopicQuery(imu_topic_)))
    {
        sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
        ofs<< imu->header.seq << "," << imu->linear_acceleration.x << "," << imu->linear_acceleration.y << "," << imu->linear_acceleration.z << ",";
        double roll, pitch, yaw;
        QE_converter_->quaternion2euler(imu->orientation, roll, pitch, yaw);
        ofs<< roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
        ofs<< imu->angular_velocity.x << "," << imu->angular_velocity.x*180/M_PI << "," << imu->angular_velocity.y << "," << imu->angular_velocity.y*180/M_PI << "," << imu->angular_velocity.z << "," << imu->angular_velocity.z*180/M_PI << ",";
        ofs<< imu->header.stamp << "," << imu->header.stamp - pre_stamp << "," << imu->header.frame_id << ",";
        ofs<< "\n";
        pre_stamp = imu->header.stamp;
    }
    ofs.close();
}

void ROSBag2CSV::write_odom2csv(rosbag::Bag& bag)
{
    std::ofstream ofs(odom2csv_file_name_, std::ios::out);
    ofs << "seq,x,y,z,vx,vy,vz,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,roll_rate,roll_rate_deg,pitch_rate,pitch_rate_deg,yaw_rate,yaw_rate_deg,time,dt,frame_id,";
    ofs << "\n";
    static ros::Time pre_stamp;

    for(const rosbag::MessageInstance& msg : rosbag::View(bag, rosbag::TopicQuery(odom_topic_)))
    {
        nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
        ofs<< odom_msg->header.seq << "," << odom_msg->pose.pose.position.x << "," << odom_msg->pose.pose.position.y << "," << odom_msg->pose.pose.position.z << ",";
        ofs<< odom_msg->twist.twist.linear.x << "," << odom_msg->twist.twist.linear.y << "," << odom_msg->twist.twist.linear.z << ",";
        ofs<< std::hypot(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y) << ",";
        double roll, pitch, yaw;
        QE_converter_->quaternion2euler(odom_msg->pose.pose.orientation, roll, pitch, yaw);
        ofs<< roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
        ofs<< odom_msg->twist.twist.angular.x << "," << odom_msg->twist.twist.angular.x*180/M_PI << "," << odom_msg->twist.twist.angular.y << "," << odom_msg->twist.twist.angular.y*180/M_PI << "," << odom_msg->twist.twist.angular.z << "," << odom_msg->twist.twist.angular.z*180/M_PI << ",";
        ofs<< odom_msg->header.stamp << "," << odom_msg->header.stamp - pre_stamp << "," << odom_msg->header.frame_id << ",";
        ofs<< "\n";
        pre_stamp = odom_msg->header.stamp;
    }
    ofs.close();
}

void ROSBag2CSV::write_poseWCS2csv(rosbag::Bag& bag)
{
    std::ofstream ofs(poseWCS2csv_file_name_, std::ios::out);
    ofs << "head_seq,x,y,z,vx,vy,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,time,dt,frame_id,";
    ofs << "\n";
    static geometry_msgs::PoseStamped pre_head_pose;
    pre_head_pose.pose.orientation.w = 1;

    for(const rosbag::MessageInstance& msg : rosbag::View(bag, rosbag::TopicQuery(poseWCS_topic_)))
    {
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        geometry_msgs::PoseStamped head_pose;
        head_pose.header = pose_msg->header;
        head_pose.pose.position.x = pose_msg->pose.pose.position.x;
        head_pose.pose.position.y = pose_msg->pose.pose.position.y;
        head_pose.pose.position.z = pose_msg->pose.pose.position.z;
        head_pose.pose.orientation = pose_msg->pose.pose.orientation;
        write_pose2csv(head_pose, pre_head_pose, ofs);
        ofs<< "\n";
    }
    ofs.close();
}

void ROSBag2CSV::read_bag()
{
    rosbag::Bag bag;
    bag.open(bag_file_name_, rosbag::bagmode::Read);
    if(is_tf_topic_set_) write_tf2csv(bag);
    if(is_imu_topic_set_) write_imu2csv(bag);
    if(is_odom_topic_set_) write_odom2csv(bag);
    if(is_poseWCS_topic_set_) write_poseWCS2csv(bag);
    bag.close();
}

void ROSBag2CSV::process()

{
    read_bag();
}
