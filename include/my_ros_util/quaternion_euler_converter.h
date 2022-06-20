#ifndef QUATERNION_EULER_CONVERTER_H
#define QUATERNION_EULER_CONVERTER_H

#include<ros/ros.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>

namespace MyROSUtil
{
    namespace QuaternionEulerConverter
    {

        geometry_msgs::Quaternion euler2quaternion(double roll, double pitch, double yaw)
        {
            tf2::Quaternion tf2_quaternion;
            tf2_quaternion.setRPY(roll, pitch, yaw);
            // ROS_WARN_STREAM("tf2_quaternion: " << tf2_quaternion.x() << ", " << tf2_quaternion.y() << ", " << tf2_quaternion.z() << ", " << tf2_quaternion.w());
            return tf2::toMsg(tf2_quaternion);
        }

        void quaternion2euler(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw)
        {
            tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        }

    }
}


#endif // QUATERNION_EULER_CONVERTER_H
