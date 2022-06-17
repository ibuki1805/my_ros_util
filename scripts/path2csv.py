#!/usr/bin/env python3
import os
import pandas as pd
import rospy
from nav_msgs.msg import Path
from tf import transformations
from geometry_msgs.msg import Vector3
import rosbag

class Path2CSV:
    def __init__(self):
        rospy.init_node("path2csv")

        self.hz = rospy.get_param("~hz", 100)
        self.footprint_path_topic = rospy.get_param("~footprint_path_topic", "/footprint_path")
        self.base_link_path_topic = rospy.get_param("~base_link_path_topic", "/base_link_path")
        self.save_dir_path = rospy.get_param("~save_dir_path", "/home/amsl/csv/trj_test_result/")
        self.output_file_name = rospy.get_param("~output_file_name", "path.csv")
        self.bag_file_name = rospy.get_param("~bag_file_name", "path.bag")


        self.base_link_path_data = pd.DataFrame(columns=["x", "y", "z", "roll", "pitch", "yaw"])
        self.footprint_path_data = pd.DataFrame(columns=["x", "y", "z", "roll", "pitch", "yaw"])

        self.footprint_path_length = 0
        self.latest_footprint_path_time = rospy.get_time()

    def process(self):
        print(os.getcwd())
        loop_rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():

            loop_rate.sleep()


    def footprint_path_callback(self, msg):
        if rospy.get_time() > self.latest_footprint_path_time :
            self.footprint_path_data = pd.DataFrame(columns=self.footprint_path_data.columns)
            for pose in msg.poses:
                angle = transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])

                self.footprint_path_data = self.footprint_path_data.append({"x": pose.pose.position.x,"y": pose.pose.position.y,"z": pose.pose.position.z,"roll": angle[0],"pitch": angle[1],"yaw": angle[2]}, ignore_index=True)


        print(self.footprint_path_data.shape)


    def read_rosbag(self, bag_file_name):
        rosbag.


if __name__ == "__main__":
    path2csv = Path2CSV()
    path2csv.process()

