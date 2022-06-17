#include<my_ros_util/quaternion_eular_converter.h>

QuaternionEulerConverter::QuaternionEulerConverter()
{}

QuaternionEulerConverter::~QuaternionEulerConverter()
{}

geometry_msgs::Quaternion QuaternionEulerConverter::eular2quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);
    // ROS_WARN_STREAM("tf2_quaternion: " << tf2_quaternion.x() << ", " << tf2_quaternion.y() << ", " << tf2_quaternion.z() << ", " << tf2_quaternion.w());
    return tf2::toMsg(tf2_quaternion);
}

void QuaternionEulerConverter::quaternion2eular(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

