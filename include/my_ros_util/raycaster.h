#ifndef RAYCASTER_H
#define RAYCASTER_H

#include<my_ros_util/gridmap_editor.h>
#include<sensor_msgs/LaserScan.h>

//TODO odom_update
namespace MyROSUtil
{
    class RayCaster
    {
        public:
            RayCaster();
            ~RayCaster();
            bool set_map_settings(nav_msgs::OccupancyGrid &map_settings);
            void set_frame_id(std::string frame_id){this -> map_settings_.header.frame_id = frame_id;}
            void set_resolution(double resolution){this -> map_settings_.info.resolution = resolution;}
            void set_origin(double x, double y, double theta){this -> map_settings_.info.origin.position.x = x;
                                                              this -> map_settings_.info.origin.position.y = y;
                                                              this -> map_settings_.info.origin.orientation.w = 1;}
            void set_width(int width){this -> map_settings_.info.width = width;}
            void set_height(int height){this -> map_settings_.info.height = height;}
            nav_msgs::OccupancyGrid get_map_settings(){return this -> map_settings_;}
            nav_msgs::OccupancyGrid get_map_msg(){return this -> map_msg_;}

            void raycasting(const sensor_msgs::LaserScan &scan);

        private:
            Gridmap* gridmap_;
            nav_msgs::OccupancyGrid map_settings_;
            nav_msgs::OccupancyGrid map_msg_;
            const int OCCUPIED = 100;
            const int FREE = 0;
            const int UNKNOWN = -1;
    };
} // namespace MyROSUtil

#endif // RAYCASTER_H
