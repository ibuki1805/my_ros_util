#include<my_ros_util/odom2csv.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2csv");
    Odom2CSV odom2csv;
    odom2csv.process();
    return 0;
}
