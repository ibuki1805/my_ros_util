#ifndef TF_VISUALIZER_H
#define TF_VISUALIZER_H

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>

class TFVisualizer
{
public:
    TFVisualizer();
    ~TFVisualizer();
    void process();
};

#endif // TF_VISUALIZER_H
