#include<my_ros_util/path2csv.h>

using MyROSUtil::QuaternionEulerConverter::quaternion2euler ;

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

void Path2CSV::write_path2csv(const nav_msgs::Path &path, const int &lookup_length)
{
    std::ofstream ofs(csv_file_name_, std::ios::app);
    int begin_idx = path.poses.size() - lookup_length -1;
    int end_idx = path.poses.size() - 1;
    std::stringstream ss;
    for(int i = begin_idx; i < end_idx; i++)
    {
        ss << path.header.seq << ",";
        ss << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << "," << path.poses[i].pose.position.z << ",";
        double roll, pitch, yaw;

        quaternion2euler(path.poses[i].pose.orientation, roll, pitch, yaw);
        ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
        ss << path.header.stamp << "," << path.header.frame_id << ",";
        ss << "\n";
    }
    ofs<< ss.str().c_str();
    ROS_INFO("written csv file");
}


void Path2CSV::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->header.stamp > path_.header.stamp)
    {
        int increased_size = msg->poses.size() - path_.poses.size();
        path_ = *msg;
        if(increased_size > 0) write_path2csv(path_, increased_size);
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


