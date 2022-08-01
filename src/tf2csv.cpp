#include<my_ros_util/tf2csv.h>

TF2CSV::TF2CSV():private_nh_("~"), nh_(""), TF_listener_(TF_buffer_)
{
    private_nh_.param<std::string>("csv_file_name_", csv_file_name_, "/home/amsl/csv/trj_test_result/steer/footprint_tf.csv");
    private_nh_.param<int>("hz", hz_, 100);
    private_nh_.param<std::string>("lookup_frame_id", lookup_frame_id_, "vicon/ccv2/ccv2");
    private_nh_.param<std::string>("world_frame_id", world_frame_id_, "world");

    pre_pose_.header.stamp = ros::Time::now();

    sub_TF_ = nh_.subscribe("/tf", 1, &TF2CSV::TF_callback, this, ros::TransportHints().tcpNoDelay());
    pub_footprint_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/footprint_pose", 1);
    pub_head_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/head_pose", 1);

}

TF2CSV::~TF2CSV(){}

void TF2CSV::write_TF2CSV(const geometry_msgs::PoseStamped& pose)
{
    std::ofstream ofs(csv_file_name_, std::ios::app);
    std::stringstream ss;
    ss << pose.header.seq << ",";
    ss << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ",";

    double dt = ros::Duration(pose.header.stamp - pre_pose_.header.stamp).toSec();
    double vx = (pose.pose.position.x - pre_pose_.pose.position.x) / dt;
    double vy = (pose.pose.position.y - pre_pose_.pose.position.y) / dt;
    double v = std::hypot(vx, vy);
    ss << vx << "," << vy << "," << v << ",";

    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = pose.pose.orientation;
    qe_converter_->quaternion2euler(pose.pose.orientation, roll, pitch, yaw);
    ss << roll << "," << roll*180/M_PI << "," << pitch << "," << pitch*180/M_PI << "," << yaw << "," << yaw*180/M_PI << ",";
    ss << pose.header.stamp << "," << dt << "," << pose.header.frame_id << ",";
    ss << "\n";
    ofs<< ss.str().c_str();
    pre_pose_ = pose;
    ROS_INFO("written csv file");
}

void TF2CSV::TF_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for(const auto& tf : msg->transforms)
    {
        if(tf.child_frame_id == lookup_frame_id_)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = tf.header;
            pose.pose.position.x = tf.transform.translation.x;
            pose.pose.position.y = tf.transform.translation.y;
            pose.pose.position.z = tf.transform.translation.z;
            pose.pose.orientation = tf.transform.rotation;
            pub_head_pose_.publish(pose);
            geometry_msgs::PoseStamped pose_in;
            pose_in.pose.position.z = -1.2;
            pose_in.pose.orientation.w = 1.0;
            pose_in.header.frame_id = lookup_frame_id_;
            geometry_msgs::PoseStamped pose_out;
            tf2::doTransform(pose_in, pose_out, tf);
            ROS_INFO_STREAM("pose_out: \n" << pose_out);
            write_TF2CSV(pose_out);
            pub_footprint_pose_.publish(pose_out);
        }
    }
}

void TF2CSV::process()
{
    std::ofstream ofs(csv_file_name_, std::ios::out);
    ofs << "seq,x,y,z,vx, vy, v, roll,roll(deg),pitch,pitch(deg),yaw,yaw(deg),time,dt,frame_id\n";
    ofs.close();
    while(ros::ok())
    {
        if(ros::Time::now() < pre_pose_.header.stamp)
        {
            ROS_WARN("Detected jump back: shutting down");
            // return;
        }
        // geometry_msgs::TransformStamped tranformstamed;
        // try{
        //     tranformstamed = TF_buffer_.lookupTransform(world_frame_id_, lookup_frame_id_, ros::Time(0));
        //     geometry_msgs::PoseStamped pose_in;
        //     pose_in.pose.position.z = -1.2;
        //     pose_in.pose.orientation.w = 1.0;
        //     pose_in.header.frame_id = lookup_frame_id_;
        //     geometry_msgs::PoseStamped pose_out;
        //     tf2::doTransform(pose_in, pose_out, tranformstamed);
        //     ROS_INFO_STREAM("pose_out: \n" << pose_out);
        //     write_TF2CSV(pose_out);
        //     pub_footprint_pose_.publish(pose_out);
        // }
        // catch(tf2::TransformException &ex)
        // {
        //     ROS_WARN("%s", ex.what());
        //     continue;
        // }
        ros::spinOnce();
        ros::Rate(hz_).sleep();
    }
    ofs.close();
}


