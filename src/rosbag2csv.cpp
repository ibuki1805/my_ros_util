#include<my_ros_util/rosbag2csv.h>

ROSBag2CSV::ROSBag2CSV(): private_nh_("~")
{
    if(private_nh_.hasPram("bag_file_name"))
        private_nh_.getParam("bag_file_name", bag_file_name_);
    else
    {
        ROS_ERROR("bag_file_name is not set");
        ros::shutdown();
    }
    private_nh_.param<std::string>("csv_file_name", csv_file_name_, "/home/amsl/csv/trj_test_result/steer/extract_from_bag.csv");
    private_nh_.param<std::string>("lookup_frame_id", lookup_frame_id_, "vicon/ccv2/ccv2");
}

ROSBag2CSV::~ROSBag2CSV(){}

geometry_msgs::PoseStamped ROSBag2CSV::convert_to_footprint(const geometry_msgs::TransformStamped& tf)
{
    geometry_msgs::PoseStamped footprint_pose;
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.pose.position.z = -1.2;
    pose_in.pose.orientation.w = 1;
    pose_in.header.frame_id = lookup_frame_id_;
    tf2::doTransform(pose_in, pose_out, tf);
    return pose_out;
}

void ROSBag2CSV::write_CSV(const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pre_pose)
{
    std::ofstream ofs(csv_file_name_, std::ios::app);
    std::stringstream ss;
    ss << pose.header.seq << ",";
    ss << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ",";

    double dt = ros::Duration(pose.header.stamp - pre_pose.header.stamp).toSec();
    double vx = (pose.pose.position.x - pre_pose.pose.position.x) / dt;
    double vy = (pose.pose.position.y - pre_pose.pose.position.y) / dt;
    double v = std::hypot(vx, vy);
    ss << vx << "," << vy << "," << v << ",";

    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = pose.pose.orientation;
    QE_converter_->quaternion2euler(pose.pose.orientation, roll, pitch, yaw);
    ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
    ss << pose.header.stamp << "," << dt << "," << pose.header.frame_id << ",";
    // ss << "\n";
    ofs<< ss.str().c_str();
    pre_pose = pose;


}

void ROSBag2CSV::read_bag()
{
    std::ofstream ofs(csv_file_name_, std::ios::out);
    ofs << "head_seq,x,y,z,vx,vy,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,time,dt,frame_id,";
    ofs << "FT_seq,x,y,z,vx,vy,v,roll,roll_deg,pitch,pitch_deg,yaw,yaw_deg,time,dt,frame_id\n";
    ofs.close();
    rosbag::Bag bag;
    bag.open(bag_file_name_, rosbag::bagmode::Read);
    static geometry_msgs::PoseStamped pre_head_pose_, pre_footprint_pose_;
    pre_head_pose_.pose.orientation.w = 1;
    pre_footprint_pose_.pose.orientation.w = 1;

    for(const rosbag::MessageInstance& msg : rosbag::View(bag, rosbag::TopicQuery("/tf")))
    {
        tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
        for(const auto& tf : tf_msg->transforms)
        {
            if(tf.child_frame_id == lookup_frame_id_)
            {
                geometry_msgs::PoseStamped head_pose;
                head_pose.header = tf.header;
                head_pose.pose.position.x = tf.transform.translation.x;
                head_pose.pose.position.y = tf.transform.translation.y;
                head_pose.pose.position.z = tf.transform.translation.z;
                head_pose.pose.orientation = tf.transform.rotation;

                geometry_msgs::PoseStamped footprint_pose = convert_to_footprint(tf);
                write_CSV(head_pose, pre_head_pose_);
                write_CSV(footprint_pose, pre_footprint_pose_);
                std::ofstream ofs_n(csv_file_name_, std::ios::app);
                ofs_n<< "\n";
            }
        }
    }
    bag.close();
}

void ROSBag2CSV::process()

{
    read_bag();
}
