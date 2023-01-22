#include<tf_visualizer/tf_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    TFVisualizer tf_visualizer;
    tf_visualizer.process();
    return 0;
}
