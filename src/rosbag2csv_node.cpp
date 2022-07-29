#include<my_ros_util/rosbag2csv.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag2csv");
    ROSBag2CSV rosbag2csv;
    rosbag2csv.process();
    return 0;
}
