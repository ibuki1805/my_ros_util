#!/usr/bin/python3

from itertools import islice
from typing import List, cast
import  math
import numpy as np
import os
import sys

import rosbag
from tf.transformations import euler_from_quaternion

import matplotlib.pyplot as plt

class BagReader:
    def __init__(self, bagfile_path: str, file_name: str) -> None:
        self.bagfile_path = bagfile_path
        self.conditions = self.get_conditions_from_file_name(file_name)
        self.pose_data = {"time": [], "x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": [], "roll_deg": [], "pitch_deg": [], "yaw_deg": [], "dt": [], "v": [], "vx": [], "vy": [], "yaw_rate": [], "yaw_rate_deg": []}
        self.target_trajectory = {"x": [], "y": []}
        self.steer_data = {"time": [], "left": [], "right": [], "left_deg": [], "right_deg": []}
        self.peson0_data = {"time": [], "x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": [], "roll_deg": [], "pitch_deg": [], "yaw_deg": []}

    def dxl2steer_angle(self) -> dict:
        with rosbag.Bag(self.bagfile_path) as bag:
            is_first_loop = True
            for topic, msg, t in bag.read_messages(topics=["/ccv_dynamixel_controller/dynamixel_state"]):
                if is_first_loop:
                    offsetTime = t.to_sec()
                self.steer_data["time"].append(t.to_sec() - offsetTime)
                for state in msg.dynamixel_state:
                    if state.name == "SteerL":
                        if is_first_loop:
                            offsetL = state.present_position
                        left_rad =  float(state.present_position - offsetL) / 303750 * math.pi
                        left_deg = left_rad * 180 / math.pi
                        self.steer_data["left"].append(left_rad)
                        self.steer_data["left_deg"].append(left_deg)
                    elif state.name == "SteerR":
                        if is_first_loop:
                            offsetR = state.present_position
                        right_rad = float(state.present_position - offsetR)/303750 * math.pi
                        right_deg = right_rad * 180 / math.pi
                        self.steer_data["right"].append(right_rad)
                        self.steer_data["right_deg"].append(right_deg)
                is_first_loop = False

        return self.steer_data

    def tf2pose(self) -> dict:
        with rosbag.Bag(self.bagfile_path) as bag:
            is_first_loop = True
            for topic, msg, t in bag.read_messages(topics=["/tf"]):
                for transform in msg.transforms:
                    if transform.child_frame_id == "vicon/ccv/ccv":
                        if is_first_loop:
                            offsetTime = transform.header.stamp.to_sec()
                            is_first_loop = False
                        _time = transform.header.stamp.to_sec() - offsetTime
                        _x = transform.transform.translation.x
                        _y = transform.transform.translation.y
                        _z = transform.transform.translation.z
                        _rotation = transform.transform.rotation
                        _roll, _pitch, _yaw = euler_from_quaternion([_rotation.x, _rotation.y, _rotation.z, _rotation.w])
                        if len(self.pose_data["time"]) == 0 :
                            _dt = 0.0
                            _vx = 0.0
                            _vy = 0.0
                            _v = 0.0
                            _yaw_rate = 0.0
                            _yaw_rate_deg = 0.0
                        else :
                            _dt = _time - self.pose_data["time"][-1]
                            _vx = (_x - self.pose_data["x"][-1]) / _dt
                            _vy = (_y - self.pose_data["y"][-1]) / _dt
                            _v = math.sqrt(_vx**2 + _vy**2)
                            _yaw_rate = (_yaw - self.pose_data["yaw"][-1]) / _dt
                            _yaw_rate_deg = _yaw_rate * 180 / math.pi

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
                        self.pose_data["dt"].append(_dt)
                        self.pose_data["vx"].append(_vx)
                        self.pose_data["vy"].append(_vy)
                        self.pose_data["v"].append(_v)
                        self.pose_data["yaw_rate"].append(_yaw_rate)
                        self.pose_data["yaw_rate_deg"].append(_yaw_rate_deg)


        # print(self.pose_data)
        return self.pose_data


    def path2course(self) -> dict:
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
        return self.target_trajectory





    def get_conditions_from_file_name(self, file_name: str) -> List[dict]:
        split_str = file_name.split("_")
        # print(split_str)
        split_split_str = split_str[4].split("-")
        conditions = {split_str[0]: float(split_str[1][0:2])/10, split_str[1][3:5]: float(split_str[2][0:2])/10, split_str[2][3:5]: float(split_str[3][0:2])/10, "t": int(split_split_str[0])}
        print(conditions)
        return conditions

    def get_conditions(self) -> dict:
        return self.conditions

    def run(self) -> None:
        self.tf2pose()
        self.path2course()


class FileNameGetter:
    def __init__(self, dir_path: str) -> None:
        self.dir_path = dir_path

    def get_file_name(self) -> List[str]:
        _file_name = os.listdir(self.dir_path)
        return _file_name


class GraphPlotter:
    def __init__(self, _data_x: [], _data_y: [], _label_x: [], _label_y: [], labels: [], _save_dir: str, graph_name) -> None:
        self.data_x = _data_x
        self.data_y = _data_y
        self.label_x = _label_x
        self.label_y = _label_y
        self.labels = labels
        self.save_dir = _save_dir
        self.graph_name = graph_name

        print(len(self.data_x))

    def make_trj_graph(self) -> None:
        fig , ax = plt.subplots()


        for i in range(len(self.data_x)):
            if i == 0:
                ax.plot(self.data_x[i], self.data_y[i], label="target", linewidth=3, linestyle="dashed")
            else :
                ax.plot(self.data_x[i], self.data_y[i], label=self.labels[i], linewidth=3)

        ax.set_xlabel(self.label_x)#x軸のラベル
        ax.set_ylabel(self.label_y)#y軸のラベル
        ax.set_xlim(-1, 7)#x軸の範囲
        ax.set_ylim(-0.75, 0.75)#y軸の範囲
        ax.set_xticks(np.arange(-1, 7.1, 1.0))#x軸の目盛り
        ax.set_yticks(np.arange(-0.75, 0.80, 0.25))#y軸の目盛り
        ax.minorticks_on()#補助目盛りを表示
        ax.grid(which="major", color="black", linestyle="-")#目盛り線を表示
        ax.grid(which="minor", color="gray", linestyle="--")#補助目盛り線を表示
        ax.legend(loc="best")#凡例を表示
        ax.set_aspect("equal")#アスペクト比を1:1に
        fig.set_figheight(6)#グラフの縦幅
        fig.set_figwidth(15)#グラフの横幅
        fig.tight_layout()#グラフの余白を調整

        # fig.savefig(self.save_dir + self.graph_name, format="png", dpi=300)

        plt.show()
        plt.clf()
        plt.cla()
        plt.close()


    def make_steer_rad_graph(self) -> None:
        fig , ax = plt.subplots()


        # print(len(self.data_x))
        # print(len(self.data_x[0]))
        # print(len(self.data_y))
        # print(len(self.data_y[0]))
        for i in range(len(self.data_y)):
            ax.plot(self.data_x, self.data_y[i], label=self.labels[i], linewidth=3)

        ax.set_xlabel(self.label_x)#x軸のラベル
        ax.set_ylabel(self.label_y)#y軸のラベル
        ax.set_xlim(2.9, 12.1)#x軸の範囲
        ax.set_ylim(-0.4, 0.4)#y軸の範囲
        ax.set_xticks(np.arange(3.0, 12.0, 1.0))#x軸の目盛り
        ax.set_yticks(np.arange(-0.5, 0.5, 0.1))#y軸の目盛り
        # ax.minorticks_on()#補助目盛りを表示
        ax.grid(which="major", color="black", linestyle="-")#目盛り線を表示
        ax.grid(which="minor", color="gray", linestyle="--")#補助目盛り線を表示
        ax.legend(loc="best")#凡例を表示
        # ax.set_aspect("equal")#アスペクト比を1:1に
        fig.set_figheight(6)#グラフの縦幅
        fig.set_figwidth(15)#グラフの横幅
        fig.tight_layout()#グラフの余白を調整

        # fig.savefig(self.save_dir + self.graph_name, format="png", dpi=300)

        plt.show()
        plt.clf()
        plt.cla()
        plt.close()

    def make_steer_deg_graph(self) -> None:
        fig , ax = plt.subplots()


        # print(len(self.data_x))
        # print(len(self.data_x[0]))
        # print(len(self.data_y))
        # print(len(self.data_y[0]))
        for i in range(len(self.data_y)):
            ax.plot(self.data_x, self.data_y[i], label=self.labels[i], linewidth=3)

        ax.set_xlabel(self.label_x)#x軸のラベル
        ax.set_ylabel(self.label_y)#y軸のラベル
        ax.set_xlim(2.9, 12.1)#x軸の範囲
        ax.set_ylim(-25, 25)#y軸の範囲
        ax.set_xticks(np.arange(3.0, 12.0, 1.0))#x軸の目盛り
        ax.set_yticks(np.arange(-25, 25.0, 5.0))#y軸の目盛り
        # ax.minorticks_on()#補助目盛りを表示
        ax.grid(which="major", color="black", linestyle="-")#目盛り線を表示
        ax.grid(which="minor", color="gray", linestyle="--")#補助目盛り線を表示
        ax.legend(loc="best")#凡例を表示
        # ax.set_aspect("equal")#アスペクト比を1:1に
        fig.set_figheight(6)#グラフの縦幅
        fig.set_figwidth(15)#グラフの横幅
        fig.tight_layout()#グラフの余白を調整

        # fig.savefig(self.save_dir + self.graph_name, format="png", dpi=300)

        plt.show()
        plt.clf()
        plt.cla()
        plt.close()

    def make_velocity_graph(self) -> None:
        fig , ax = plt.subplots()


        # print(len(self.data_x))
        # print(len(self.data_x[0]))
        # print(len(self.data_y))
        # print(len(self.data_y[0]))
        for i in range(len(self.data_y)):
            ax.plot(self.data_x, self.data_y[i], label=self.labels[i], linewidth=3)

        ax.set_xlabel(self.label_x)#x軸のラベル
        ax.set_ylabel(self.label_y)#y軸のラベル
        ax.set_xlim(0.0, 15.0)#x軸の範囲
        ax.set_ylim(-2.0, 2.4)#y軸の範囲
        ax.set_xticks(np.arange(0.0, 15.0, 1.0))#x軸の目盛り
        ax.set_yticks(np.arange(-1.8, 2.4, 0.4))#y軸の目盛り
        # ax.minorticks_on()#補助目盛りを表示
        ax.grid(which="major", color="black", linestyle="-")#目盛り線を表示
        ax.grid(which="minor", color="gray", linestyle="--")#補助目盛り線を表示
        ax.legend(loc="best")#凡例を表示
        # ax.set_aspect("equal")#アスペクト比を1:1に
        fig.set_figheight(6)#グラフの縦幅
        fig.set_figwidth(15)#グラフの横幅
        fig.tight_layout()#グラフの余白を調整

        # fig.savefig(self.save_dir + self.graph_name, format="png", dpi=300)

        plt.show()
        plt.clf()
        plt.cla()
        plt.close()

def create_label(_conditions: [], keys: []) -> []:
    labels = []
    for condition in _conditions:
        label = str()
        for key in condition.keys():
            if key in keys:
                label = label + key + "-" + str(condition[key]) + "_"
        labels.append(label[0:-1])
    return labels


if __name__ == "__main__":
    _bagfile_dir = "/home/amsl/bagfiles/trj_test0121/steer"
    # _bagfile_dir = "/mnt/c/Users/baske/master_thesis_data/trj_test0120/steer"
    _file_name_getter = FileNameGetter(_bagfile_dir)
    _file_name_list = _file_name_getter.get_file_name()
    # print(_file_name_list)
    print(len(_file_name_list))
    data_lists = []
    for _file_name in _file_name_list:
        _bagfile_path = _bagfile_dir + "/" + _file_name
        bag_reader = BagReader(_bagfile_path, _file_name)
        _poses = { "conditions": bag_reader.get_conditions(), "poses": bag_reader.tf2pose(), "steer": bag_reader.dxl2steer_angle()}
        data_lists.append(_poses)

    print("All data imported")

    data_x = []
    data_y = []
    conditions = []
    r_steer_data = []
    r_deg_steer_data = []
    l_steer_data = []
    l_deg_steer_data = []
    steer_time = []
    pose_time = []
    v_data = []
    vx_data = []
    vy_data = []

    #add target trj
    target_data = BagReader(_bagfile_dir + "/" + _file_name_list[0], _file_name_list[0])
    target_path = target_data.path2course()
    data_x.append(target_path["x"])
    # print(target_path["x"])
    data_y.append(target_path["y"])
    # print(target_path["y"])
    conditions.append(target_data.get_conditions())

    #aad data
    v = float(sys.argv[1])
    l1 = float(sys.argv[2])
    l2 = float(sys.argv[3])
    t = int(sys.argv[4])
    for data in data_lists:
        #search target
        if data["conditions"]["v"] == v and  data["conditions"]["L1"] == l1 and data["conditions"]["L2"] == l2:
        # if data["conditions"]["v"] == 1.2 and  data["conditions"]["L1"] == 0.5 and data["conditions"]["t"] == 2:
            data_x.append(data["poses"]["x"])
            data_y.append(data["poses"]["y"])
            conditions.append(data["conditions"])
            r_steer_data.append(data["steer"]["right"])
            l_steer_data.append(data["steer"]["left"])
            r_deg_steer_data.append(data["steer"]["right_deg"])
            l_deg_steer_data.append(data["steer"]["left_deg"])
            steer_time.append(data["steer"]["time"])
            pose_time.append(data["poses"]["time"])
            v_data.append(data["poses"]["v"])
            vx_data.append(data["poses"]["vx"])
            vy_data.append(data["poses"]["vy"])

    labels = create_label(conditions, ["v", "L1", "L2", "t"])
    # print(labels)

    trj_graph = GraphPlotter(data_x, data_y, "x [m]", "y [m]", labels, "/home/amsl/master_thesis/thesis/pictures/trj_test_steer/","v1.2-steer.png")
    # trj_graph.make_trj_graph()

    steer_rad_graph = GraphPlotter(steer_time[0], [r_steer_data[0], l_steer_data[0]], "t [s]", "steer angle [rad]", ["right steer", "left steer"], "/home/amsl/master_thesis/thesis/pictures/trj_test_steer/","v1.2-steer.png")
    steer_rad_graph.make_steer_rad_graph()
    steer_rad_graph = GraphPlotter(steer_time[0], [r_deg_steer_data[0], l_deg_steer_data[0]], "t [s]", "steer angle [deg]", ["right steer", "left steer"], "/home/amsl/master_thesis/thesis/pictures/trj_test_steer/","v1.2-steer.png")
    steer_rad_graph.make_steer_deg_graph()

    print(len(pose_time[0]))
    print(len(v_data[0]))
    print(len(vx_data[0]))
    print(len(vy_data[0]))
    velocity_graph = GraphPlotter(pose_time[0], [v_data[0], vx_data[0], vy_data[0]], "t [s]", "velocity [m/s]", ["v", "vx", "vy"], "/home/amsl/master_thesis/thesis/pictures/trj_test_steer/", "test_v.png")
    velocity_graph.make_velocity_graph()


    print(labels)
    print(len(labels))
    print(len(data_x))
    print(len(data_y))
    print(len(conditions))
    print(data_lists[1].keys())
    print(data_lists[1]["conditions"].keys())
    print(data_lists[1]["poses"].keys())
    print(data_lists[1]["steer"].keys())


