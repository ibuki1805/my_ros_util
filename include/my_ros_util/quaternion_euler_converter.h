#ifndef QUATERNION_EULER_CONVERTER_H
#define QUATERNION_EULER_CONVERTER_H

#include<ros/ros.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>

namespace MyROSUtil
{
    class QEConverter
    {
    public:
        QEConverter();
        ~QEConverter();
        geometry_msgs::Quaternion euler2quaternion(const double &roll, const double &pitch, const double &yaw);
        geometry_msgs::Quaternion yaw2quaternion(const double &yaw);
        void quaternion2euler(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw);
        void quaternion2yaw(geometry_msgs::Quaternion quaternion, double &yaw);
    };

}//namespace MyROSUtil


#endif // QUATERNION_EULER_CONVERTER_H
