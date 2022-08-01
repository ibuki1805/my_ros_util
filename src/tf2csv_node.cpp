#include <my_ros_util/tf2csv.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2csv");
    TF2CSV tf2csv;
    tf2csv.process();
    return 0;
}
