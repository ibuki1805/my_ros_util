#include<my_ros_util/path2csv.h>

Path2CSV::Path2CSV():private_nh_("~"), nh_("")
{
    private_nh_.param<std::string>("path_topic", path_topic_, "/path");
    private_nh_.param<int>("hz", hz_, 10);

    sub_path_ = nh_.subscribe(path_topic_, 1, &Path2CSV::pathCallback, this);
}

Path2CSV::~Path2CSV()
{
}

void Path2CSV::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->header.stamp > path_.header.stamp)
    {
        path_ = *msg;
        ROS_INFO("path is updated with %d points", (int)path_.poses.size());
    }
}

void Path2CSV::process()
{
    ros::spin();
}


