#! /usr/bin/env python3

from . import pheromone
import rospy
from std_msgs.msg import Float32MultiArray
import math
from gazebo_msgs.msg import ModelStates


class Node():

    def __init__(self):
        # pheromonクラスのインスタンス生成
        self.pheromone = pheromone.Pheromone(
            grid_map_size=2.0, resolution=50.0, evapolation=0.0, diffusion=0.0)

        # フェロモンの最大値と最小値を設定
        self.max_pheromone_value = 1.0
        self.min_pheromone_value = 0.0

        # pheromoneの射出を行うかどうか
        # self.is_pheromone_injection = True

        # Publisher & Subscriber
        # フェロモン値を送信
        self.publish_pheromone = rospy.Publisher('/pheromone_value',
                                                 Float32MultiArray,
                                                 queue_size=10)
        # gazeboの環境上にあるオブジェクトの状態を取得
        # 取得すると, pheromoneCallback関数が呼び出される
        rospy.Subscriber()
        self.subscribe_pose = rospy.Subscriber('/gazebo/model_states',
                                               ModelStates,
                                               self.pheromoneCallback)

    # 座標からフェロモングリッドへ変換
    def posToIndex(self, x, y):
        round_decimal_places = int(math.log10(self.pheromone.resolution))
        x = round(x, round_decimal_places)
        y = round(y, round_decimal_places)
        x = int(x*resolution)
        y = int(y*resolution)

        x_index = int(x + (self.pheromone.num_cell - 1)/2)
        y_index = int(y + (self.pheromone.num_cell - 1)/2)

        if x_index < 0 or y_index < 0 or x_index > self.pheromone.num_cell-1 \
                or y_index > self.pheromone.num_cell-1:
            raise Exception("The pheromone matrix index is out of range.")
        return x_index, y_index

    # フェロモングリッドから座標へ変換
    def indexToPos(self, x_index, y_index):
        x = x_index - (self.pheromone.num_cell-1)/2
        y = y_index - (self.pheromone.num_cell-1)/2

        x = float(x) / self.pheromone.resolution
        y = float(y) / self.pheromone.resolution

        return x, y

    def pheromoneCallback(self, model_status):
        # Reading from arguments
        print(model_status.model_name)
        # pose = message.pose[-1]
        # twist = message.twist[-1]
        # pos = pose.position
        # ori = pose.orientation
        # pheromone = cargs
        # x = pos.x
        # y = pos.y

        # angles = tf.transformations.euler_from_quaternion(
        #     (ori.x, ori.y, ori.z, ori.w))
        # if angles[2] < 0:
        #     self.theta = angles[2] + 2*math.pi
        # else:
        #     self.theta = angles[2]

        # # 9 pheromone values
        # # Position of 9 cells surrounding the robot
        # x_index, y_index = self.posToIndex(x, y)
        # phero_val = Float32MultiArray()
        # for i in range(3):
        #     for j in range(3):
        #         phero_val.data.append(
        #             self.pheromone.getPhero(x_index+i-1, y_index+j-1))
        # # print("phero_avg: {}".format(np.average(np.asarray(phero_val.data))))
        # self.publish_pheromone.publish(phero_val)
        # # # Assign pheromone value and publish it
        # # phero_val = phero.getPhero(x_index, y_index)
        # # self.pub_phero.publish(phero_val)


if __name__ == "__main__":
    rospy.init_node('pheromone')
    node1 = Node()
    rospy.spin()
