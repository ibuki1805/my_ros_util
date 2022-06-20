#include<my_ros_util/odom2csv.h>

Odom2CSV::Odom2CSV():private_nh_("~"), nh_("")
{
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/steering_odom");
    private_nh_.param<std::string>("csv_file_name_", csv_file_name_, "/home/amsl/csv/trj_test_result/steer/footprint_odom.csv");
    private_nh_.param<int>("hz", hz_, 1);

    sub_odom_ = nh_.subscribe(odom_topic_, 1, &Odom2CSV::odom_callback, this);
}

Odom2CSV::~Odom2CSV()
{
}

void Odom2CSV::quaternion2euler(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void Odom2CSV::write_odom2csv(const nav_msgs::Odometry::ConstPtr &odom, const nav_msgs::Odometry &pre_odom)
{
    std::ofstream ofs(csv_file_name_, std::ios::app);
    std::stringstream ss;
    ss << odom.header.seq << ",";
    ss << odom.pose.pose.position.x << "," << odom.pose.pose.position.y << "," << odom.pose.pose.position.z << ",";
    double roll, pitch, yaw;
    quaternion2euler(odom.pose.pose.orientation, roll, pitch, yaw);
    ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
    ss << odom.header.stamp << "," << odom.header.frame_id << ",";
    ss << "\n";
    ofs<< ss.str().c_str();
    ROS_INFO("written csv file");
}


void Odom2CSV::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    write_odom2csv(msg, odom_);
    odom_ = *msg;
    // ROS_INFO("odom is updated with %d points", (int)odom_.poses.size());

}

void Odom2CSV::process()
{
    std::ofstream ofs(csv_file_name_, std::ios::out);
    ofs << "seq,x,y,z,vx, vy, v, roll,roll(deg),pitch,pitch(deg),yaw,yaw(deg),time,frame_id\n";
    ofs.close();
    while(ros::ok())
    {
        if(ros::Time::now() < odom_.header.stamp)
        {
            ROS_WARN("Detected jump back: shutting down");
            return;
        }
        ros::spinOnce();
        // ros::Rate(hz_).sleep();
    }
}


