#ifndef GRIDMAP_EDITOR_H
#define GRIDMAP_EDITOR_H

// OccupancyGrid
// width is cell count in x direction
// height is cell count in y direction
// resolution is the size of each cell in meters
// data's order is x->y start from bottom right corner
//TODO make function for rotateing gridmap

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>

class Gridmap
{
public:
    bool set_map(nav_msgs::OccupancyGrid &map);
    int get_grid(double y, double x);
    int get_grid(int y, int x);
    double get_resolution(){return this->resolution_;}
    int get_width(){return this->width_;}
    int get_height(){return this->height_;}
    double get_origin_x(){return this->origin_x_;}
    double get_origin_y(){return this->origin_y_;}
    std::string get_frame_id(){return this->frame_id_;}
    nav_msgs::OccupancyGrid set_grid();


private:
    std::vector<std::vector<int>> grid_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double resolution_;
    std::string frame_id_;
};

#endif // GRIDMAP_EDITOR_H
