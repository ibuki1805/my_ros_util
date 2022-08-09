#include<my_ros_util/raycaster.h>

namespace MyROSUtil
{
    RayCaster::RayCaster()
    {
        gridmap_ = new Gridmap();
    }

    RayCaster::~RayCaster(){ delete gridmap_; }

    bool RayCaster::set_map_settings(nav_msgs::OccupancyGrid &map_settings)
    {
        bool success = gridmap_ -> set_map(map_settings);
        if(success)
        {
            map_settings_ = map_settings;
            return success;
        }
        else return success;
    }


    void RayCaster::raycasting(const sensor_msgs::LaserScan &scan)
    {
        double angle = scan.angle_min; //+M_PI/2;
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
