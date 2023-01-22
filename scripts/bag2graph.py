#!/usr/bin/python3

from itertools import islice
from typing import List, cast
import  math
import numpy as np
import os
import time

import rosbag
from tf.transformations import euler_from_quaternion

import matplotlib.pyplot as plt

class BagReader:
    def __init__(self, bagfile_path: str, file_name: str) -> None:
        self.bagfile_path = bagfile_path
        self.conditions = self.get_conditions_from_file_name(file_name)
        self.pose_data = {"time": [], "x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": [], "roll_deg": [], "pitch_deg": [], "yaw_deg": []}
        self.target_trajectory = {"x": [], "y": []}


    def tf2pose(self) -> None:
        with rosbag.Bag(self.bagfile_path) as bag:
            for topic, msg, t in bag.read_messages(topics=["/tf"]):
                for transform in msg.transforms:
                    if transform.child_frame_id == "vicon/ccv/ccv":
                        _time = transform.header.stamp.to_sec()
                        _x = transform.transform.translation.x
                        _y = transform.transform.translation.y
                        _z = transform.transform.translation.z
                        _rotation = transform.transform.rotation
                        _roll, _pitch, _yaw = euler_from_quaternion([_rotation.x, _rotation.y, _rotation.z, _rotation.w])
                        _roll_deg = _roll * 180 / math.pi
                        _pitch_deg = _pitch * 180 / math.pi
                        _yaw_deg = _yaw * 180 / math.pi
                        self.pose_data["time"].append(_time)
                        self.pose_data["x"].append(_x)
                        self.pose_data["y"].append(_y)
                        self.pose_data["z"].append(_z)
                        self.pose_data["roll"].append(_roll)
                        self.pose_data["pitch"].append(_pitch)
                        self.pose_data["yaw"].append(_yaw)
                        self.pose_data["roll_deg"].append(_roll_deg)
                        self.pose_data["pitch_deg"].append(_pitch_deg)
                        self.pose_data["yaw_deg"].append(_yaw_deg)
        # print(self.pose_data)


    def path2course(self) -> None:
        topics = ["/local_path"]
        with rosbag.Bag(self.bagfile_path) as bag:
            _message_count = bag.get_message_count(topic_filters=topics)
            last_message = islice(bag.read_messages(topics=topics), _message_count - 1, _message_count)
            _, msg, *_ = next(last_message)
            _path = msg
            for pose in _path.poses:
                _x = pose.pose.position.x
                _y = pose.pose.position.y
                self.target_trajectory["x"].append(_x)
                self.target_trajectory["y"].append(_y)

        # print(self.target_trajectory)


    def show_graph(self) -> None:
        fig , ax = plt.subplots()

        ax.plot(self.pose_data["x"], self.pose_data["y"], label="trajectory", linewidth=3)
        ax.plot(self.target_trajectory["x"], self.target_trajectory["y"], label="target", linewidth=3)

        ax.set_xlabel("x [m]")#x軸のラベル
        ax.set_ylabel("y [m]")#y軸のラベル
        ax.set_xlim(-1, 7)#x軸の範囲
        ax.set_ylim(-0.75, 0.75)#y軸の範囲
        ax.set_xticks(np.arange(-1, 7.1, 1.0))#x軸の目盛り
        ax.set_yticks(np.arange(-0.75, 0.80, 0.25))#y軸の目盛り
        ax.minorticks_on()#補助目盛りを表示
        ax.grid(which="major", color="black", linestyle="-")#目盛り線を表示
        ax.grid(which="minor", color="gray", linestyle="--")#補助目盛り線を表示
        ax.legend(loc="best")#凡例を表示
        ax.set_aspect("equal")#アスペクト比を1:1に
        fig.set_figheight(3)#グラフの縦幅
        fig.set_figwidth(10)#グラフの横幅
        fig.tight_layout()#グラフの余白を調整

        graph_title = str()
        for key in self.conditions.keys():
            graph_title = graph_title + key + "-" + str(self.conditions[key]) + "_"
        graph_title = "/home/amsl/master_thesis/thesis/pictures/trj_test_steer/" + graph_title + ".png"
        # print(graph_title)

        fig.savefig(graph_title, format="png", dpi=300)

        # plt.show()
        plt.clf()
        plt.cla()
        plt.close()



    def get_conditions_from_file_name(self, file_name: str) -> List[dict]:
        split_str = file_name.split("_")
        # print(split_str)
        split_split_str = split_str[4].split("-")
        conditions = {split_str[0]: float(split_str[1][0:2])/10, split_str[1][3:5]: float(split_str[2][0:2])/10, split_str[2][3:5]: float(split_str[3][0:2])/10, "t": split_split_str[0]}
        print(conditions)
        return conditions

    def run(self) -> None:
        self.tf2pose()
        self.path2course()
        self.show_graph()


class FileNameGetter:
    def __init__(self, dir_path: str) -> None:
        self.dir_path = dir_path

    def get_file_name(self) -> List[str]:
        _file_name = os.listdir(self.dir_path)
        return _file_name

if __name__ == "__main__":
    _bagfile_dir = "/home/amsl/bagfiles/trj_test0121/steer"
    _file_name_getter = FileNameGetter(_bagfile_dir)
    _file_name_list = _file_name_getter.get_file_name()
    # print(_file_name_list)
    print(len(_file_name_list))
    for _file_name in _file_name_list:
        _bagfile_path = _bagfile_dir + "/" + _file_name
        bag_reader = BagReader(_bagfile_path, _file_name)
        bag_reader.run()
        # time.sleep(0.5)

    # _bagfile_path = "/home/amsl/bagfiles/trj_test0121/steer/v_12-L1_05-L2_05_1v-2023-01-21-18-42steering_odom_test.bag"
    # bag_reader = BagReader(_bagfile_path)
    # bag_reader.run()

