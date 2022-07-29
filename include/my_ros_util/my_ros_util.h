#ifndef MY_ROS_UTIL_H
#define MY_ROS_UTIL_H

// OccupancyGrid
// width is cell count in x direction
// height is cell count in y direction
// resolution is the size of each cell in meters
// data's order is x->y start from bottom right corner
//TODO make function for rotateing gridmap
#include<my_ros_util/gridmap_editor.h>

//TODO odom_update
#include<my_ros_util/raycaster.h>

#include<my_ros_util/quaternion_euler_converter.h>

#endif // MY_ROS_UTIL_H

