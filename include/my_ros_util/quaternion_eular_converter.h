#ifndef QUATERNION_EULER_CONVERTER_H
#define QUATERNION_EULER_CONVERTER_H

#include<ros/ros.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>


class QuaternionEulerConverter
{
public:
    QuaternionEulerConverter();
    ~QuaternionEulerConverter();
    geometry_msgs::Quaternion eular2quaternion(double roll, double pitch, double yaw);
    void quaternion2eular(geometry_msgs::Quaternion quaternion, double &roll, double &pitch, double &yaw);

private:


};
#endif // QUATERNION_EULER_CONVERTER_H
