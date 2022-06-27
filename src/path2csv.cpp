#include<my_ros_util/path2csv.h>

Path2CSV::Path2CSV():private_nh_("~"), nh_("")
{
    private_nh_.param<std::string>("path_topic", path_topic_, "/path");
    private_nh_.param<std::string>("csv_file_name_", csv_file_name_, "/home/amsl/csv/trj_test_result/steer/footprint_path.csv");
    private_nh_.param<int>("hz", hz_, 1);

    sub_path_ = nh_.subscribe(path_topic_, 1, &Path2CSV::pathCallback, this);
}

Path2CSV::~Path2CSV()
{
}

void Path2CSV::quaternion2euler(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void Path2CSV::write_path2csv(const nav_msgs::Path &path, const int &lookup_length)
{
    std::ofstream ofs(csv_file_name_, std::ios::out);
    // int begin_idx = path.poses.size() - lookup_length -1;
    // int end_idx = path.poses.size() - 1;
    ofs << "seq,x,y,z,roll,roll(deg),pitch,pitch(deg),yaw,yaw(deg),time,frame_id\n";
    std::stringstream ss;
    for(const auto &pose : path.poses)
    {
        ss << pose.header.seq << ",";
        ss << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ",";
        double roll, pitch, yaw;

        quaternion2euler(pose.pose.orientation, roll, pitch, yaw);
        ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
        double time = ros::Duration(pose.header.stamp - path.poses[0].header.stamp).toSec();
        ss << time << "," << pose.header.frame_id << ",";
        ss << "\n";
    }
    ofs<< ss.str().c_str();
    ofs.close();
    ROS_INFO("written csv file");
}


void Path2CSV::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->header.stamp > path_.header.stamp)
    {
        int increased_size = msg->poses.size() - path_.poses.size();
        path_ = *msg;
        write_path2csv(path_, increased_size);
        // ROS_INFO("path is updated with %d points", (int)path_.poses.size());

    }
}

void Path2CSV::process()
{
    std::ofstream ofs(csv_file_name_, std::ios::out);
    ofs << "seq,x,y,z,roll,roll(deg),pitch,pitch(deg),yaw,yaw(deg),time,frame_id\n";
    ofs.close();
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Rate(hz_).sleep();
    }
}


