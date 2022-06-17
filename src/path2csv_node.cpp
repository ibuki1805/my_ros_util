#include<my_ros_util/path2csv.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path2csv");
    Path2CSV path2csv;
    path2csv.process();
    return 0;
}
