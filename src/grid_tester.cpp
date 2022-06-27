#ifndef GRID_TESTER
#define GRID_TESTER

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PointStamped.h>

class GridTester
{
public:
    GridTester();
    ~GridTester();
    void process();

private:
    void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void point_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

    bool is_first_map_;
    int map_width_;
    int map_height_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_grid_;
    ros::Subscriber sub_point_;
    ros::Publisher pub_grid_;
};

#endif // GRID_TESTER

GridTester::GridTester():nh_("")
{
    sub_grid_ = nh_.subscribe("/map", 1, &GridTester::grid_callback, this);
    sub_point_ = nh_.subscribe("/clicked_point", 1, &GridTester::point_callback, this);
    pub_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/test_grid", 1);
    is_first_map_ = true;
}

GridTester::~GridTester(){}

void GridTester::grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(is_first_map_)
    {
        map_width_ = msg->info.width;
        map_height_ = msg->info.height;
        map_resolution_ = msg->info.resolution;
        ROS_INFO("map_width: %d, map_height: %d, map_resolution: %f", map_width_, map_height_, map_resolution_);
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;
        ROS_INFO("map_origin_x: %f, map_origin_y: %f", map_origin_x_, map_origin_y_);
        int counter = 0;
        for(const auto &cell: msg->data)
        {
            if(cell != -1)
            {
                ROS_INFO("first known cell: %d", counter);
                break;
            }
            counter++;
        }

        nav_msgs::OccupancyGrid test_grid;
        test_grid = *msg;
        for(int i=0; i<10; i++) test_grid.data[i] = 100;
        test_grid.header.stamp = ros::Time::now();
        pub_grid_.publish(test_grid);

        is_first_map_ = false;
    }
}

void GridTester::point_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("point: %f, %f", msg->point.x, msg->point.y);
}

void GridTester::process()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_tester");
    GridTester grid_tester;
    grid_tester.process();
    return 0;
}
