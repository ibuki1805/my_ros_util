#include<tf_visualizer/tf_visualizer.h>

TFVisualizer::TFVisualizer(){}

TFVisualizer::~TFVisualizer(){}

void TFVisualizer::process()
{
    ros::NodeHandle nh("");
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("base_link_pose", 1);
    ros::NodeHandle pnh("~");
    std::string target_frame;
    pnh.param<std::string>("target_frame", target_frame, "base_link");
    std::string world_frame;
    pnh.param<std::string>("world_frame", world_frame, "map");
    ros::Rate rate(100);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.header.frame_id = target_frame;
        pose_pub.publish(pose_stamped);
        rate.sleep();
    }
}
