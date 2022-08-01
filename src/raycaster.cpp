#include<my_ros_util/raycaster.h>

namespace MyROSUtil
{
    RayCaster::RayCaster(nav_msgs::OccupancyGrid &map_settings)
    {
        delete gridmap_;
        gridmap_ = new Gridmap;
        gridmap_ -> set_map(map_settings);
        map_settings_ = map_settings;
    }

    RayCaster::~RayCaster(){ delete gridmap_; }


    void RayCaster::raycasting(const sensor_msgs::LaserScan &scan)
    {
        double angle = scan.angle_min+M_PI/2;
        for(const auto &laser : scan.ranges)
        {
            double r = 0;
            while(1)
            {
                double x = r*cos(angle);
                double y = r*sin(angle);
                double diff = laser - r;
                if(!gridmap_ -> set_grid(y, x, UNKNOWN))break;
                if(diff > map_settings_.info.resolution) gridmap_ -> set_grid(y, x, FREE);
                else if(diff < -map_settings_.info.resolution) gridmap_ -> set_grid(y, x, UNKNOWN);
                else gridmap_ -> set_grid(y, x, OCCUPIED);
                r += map_settings_.info.resolution;
            }
            angle += scan.angle_increment;
            //hode

        }
        map_msg_ = gridmap_ -> get_grid();
    }

} // namespace MyROSUtil
