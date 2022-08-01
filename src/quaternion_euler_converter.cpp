#include<my_ros_util/quaternion_euler_converter.h>


namespace MyROSUtil
{

    QEConverter::QEConverter(){}

    QEConverter::~QEConverter(){}

    geometry_msgs::Quaternion QEConverter::euler2quaternion(const double &roll, const double &pitch, const double &yaw)
    {
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(roll, pitch, yaw);
        // ROS_WARN_STREAM("tf2_quaternion: " << tf2_quaternion.x() << ", " << tf2_quaternion.y() << ", " << tf2_quaternion.z() << ", " << tf2_quaternion.w());
        return tf2::toMsg(tf2_quaternion);
    }

    geometry_msgs::Quaternion QEConverter::yaw2quaternion(const double &yaw)
    {
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(0, 0, yaw);
        // ROS_WARN_STREAM("tf2_quaternion: " << tf2_quaternion.x() << ", " << tf2_quaternion.y() << ", " << tf2_quaternion.z() << ", " << tf2_quaternion.w());
        return tf2::toMsg(tf2_quaternion);
    }

    void QEConverter::quaternion2euler(const geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw)
    {
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    void QEConverter::quaternion2yaw(const geometry_msgs::Quaternion quaternion, double &yaw)
    {
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

}//namespace MyROSUtil
