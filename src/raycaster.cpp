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
                if(!gridmap_ -> is_in_map(y, x)) break;
                if(diff > map_settings_.info.resolution) gridmap_ -> set_grid(y, x, FREE);
                else if(diff < -map_settings_.info.resolution)
                {
                    // if(gridmap_->get_grid(y, x) == OCCUPIED) gridmap_->set_grid(y, x, OCCUPIED);
                    // else gridmap_->set_grid(y, x, UNKNOWN);
                    gridmap_ -> set_grid(y, x, UNKNOWN);
                }
                else gridmap_ -> set_grid(y, x, OCCUPIED);
                r += map_settings_.info.resolution;
            }
            angle += scan.angle_increment;

        }
    }

    void RayCaster::odom_update(const double dy, const double dx, const double dtheta)
    {
        Gridmap new_gridmap;
        new_gridmap.set_map(map_settings_);
        double max_x = (double)map_settings_.info.width*map_settings_.info.resolution*0.5;
        double max_y = (double)map_settings_.info.height*map_settings_.info.resolution*0.5;
        double resolution = map_settings_.info.resolution;
        for(double x = -max_x; x < max_x; x+=resolution)
        {
            for(double y = -max_y; y < max_y; y+=resolution)
            {
                double new_x = (x)*cos(-dtheta) - (y)*sin(-dtheta) - dx;
                double new_y = (x)*sin(-dtheta) + (y)*cos(-dtheta) - dy;
                if(gridmap_ -> is_in_map(y, x))
                    new_gridmap.set_grid(new_y, new_x, gridmap_ -> get_grid(y, x));
                else new_gridmap.set_grid(new_y, new_x, UNKNOWN);
            }
        }
        *gridmap_ = new_gridmap;
    }

    void RayCaster::clear_map()
    {
        gridmap_ -> clear_map(UNKNOWN);
    }

    nav_msgs::OccupancyGrid RayCaster::get_map_msg()
    {
        return gridmap_ -> get_grid();
    }
} // namespace MyROSUtil
