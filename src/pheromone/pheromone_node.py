#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import math
from gazebo_msgs.msg import ModelStates
import numpy as np
import time
from pathlib import Path

# import tf


class Node():

    def __init__(self):
        # pheromonクラスのインスタンス生成
        self.pheromone = Pheromone(
            grid_map_size=4, resolution=50, evaporation=0.0, diffusion=0.0)

        # フェロモンの最大値と最小値を設定
        self.max_pheromone_value = 1.0
        self.min_pheromone_value = 0.0

        # 経過時間
        self.start_time = time.process_time()

        # pheromoneの射出を行うかどうか
        # self.is_pheromone_injection = True
        # オブジェクトの周りにフェロモンを配置
        # self.pheromone.injectionCircle(
        # 0.0, 0.4, self.max_pheromone_value, 0.06)
        obstacle_pos_x = 0.0
        obstacle_pos_y = 0.4
        x_index, y_index = self.posToIndex(obstacle_pos_x, obstacle_pos_y)
        self.pheromone.injectionCircle(
            x_index, y_index, self.max_pheromone_value, 0.06)
        obstacle_pos_x = 0.0
        obstacle_pos_y = -0.4
        x_index, y_index = self.posToIndex(obstacle_pos_x, obstacle_pos_y)
        self.pheromone.injectionCircle(
            x_index, y_index, self.max_pheromone_value, 0.06)
        obstacle_pos_x = 0.4
        obstacle_pos_y = 0.0
        x_index, y_index = self.posToIndex(obstacle_pos_x, obstacle_pos_y)
        self.pheromone.injectionCircle(
            x_index, y_index, self.max_pheromone_value, 0.06)
        obstacle_pos_x = -0.4
        obstacle_pos_y = 0.0
        x_index, y_index = self.posToIndex(obstacle_pos_x, obstacle_pos_y)
        self.pheromone.injectionCircle(
            x_index, y_index, self.max_pheromone_value, 0.06)

        # Publisher & Subscriber
        # フェロモン値を送信
        self.publish_pheromone = rospy.Publisher('/pheromone_value',
                                                 Float32MultiArray,
                                                 queue_size=10)
        # gazeboの環境上にあるオブジェクトの状態を取得
        # 取得すると, pheromoneCallback関数が呼び出される
        self.subscribe_pose = rospy.Subscriber('/gazebo/model_states',
                                               ModelStates,
                                               self.pheromoneCallback)

    # 座標からフェロモングリッドへ変換
    def posToIndex(self, x, y):
        round_decimal_places = int(math.log10(self.pheromone.resolution))
        x = round(x, round_decimal_places)
        y = round(y, round_decimal_places)
        x = int(x*self.pheromone.resolution)
        y = int(y*self.pheromone.resolution)

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
        robot_index = model_status.name.index('hero_0')
        # print(model_status.pose)
        # print(model_status.twist)
        pose = model_status.pose[robot_index]
        # twist = model_status.twist[robot_index]
        pos = pose.position
        # ori = pose.orientation

        # angles = tf.transformations.euler_from_quaternion(
        #     (ori.x, ori.y, ori.z, ori.w))
        # if angles[2] < 0:
        #     self.theta = angles[2] + 2*math.pi
        # else:
        #     self.theta = angles[2]

        # 9 pheromone values
        # Position of 9 cells surrounding the robot
        # print('pos : (x, y) = (' + str(pos.x) + ', ' + str(pos.y) + ')')
        x_index, y_index = self.posToIndex(pos.x, pos.y)
        # print('pos_index : [x, y] = [' +
        #       str(x_index) + ', ' + str(y_index) + ']')
        pheromone_value = Float32MultiArray()
        for i in range(3):
            for j in range(3):
                pheromone_value.data.append(
                    self.pheromone.getPheromone(x_index+i-1, y_index+j-1))
        # print(pheromone_value)
        # print("phero_avg: {}".format(np.average(np.asarray(pheromone_value.data))))
        self.publish_pheromone.publish(pheromone_value)
        # # Assign pheromone value and publish it
        # phero_val = phero.getPhero(x_index, y_index)
        # self.pub_phero.publish(phero_val)

        # if self.start_time() >= 20:
        self.pheromone.save("experients_01")


class Pheromone():

    def __init__(self, grid_map_size=0, resolution=0, evaporation=0.0, diffusion=0.0):
        # グリッド地図の生成
        # map size = 1 m * size
        self.grid_map_size = grid_map_size
        # grid cell size = 1 m / resolution
        self.resolution = resolution
        # 1辺におけるグリッドセルの合計個数
        self.num_cell = self.resolution * self.grid_map_size + 1
        # 例外処理 : 一辺におけるグリッドセルの合計個数を必ず奇数でなければならない
        if self.num_cell % 2 == 0:
            raise Exception(
                "Number of cell is even. It needs to be an odd number")
        self.grid = np.zeros((self.num_cell, self.num_cell))
        self.grid_copy = np.zeros((self.num_cell, self.num_cell))

        # 蒸発パラメータの設定
        self.evaporation = evaporation
        if self.evaporation == 0.0:
            self.isEvaporation = False
        else:
            self.isEvaporation = True

        # 拡散パラメータの設定
        self.diffusion = diffusion
        if self.diffusion == 0.0:
            self.isDiffusion = False
        else:
            self.isDiffusion = True

        # Timers
        self.update_timer = time.process_time()
        self.step_timer = time.process_time()
        self.injection_timer = time.process_time()
        self.save_timer = time.process_time()
        self.reset_timer = time.process_time()

    # 指定した座標(x, y)からフェロモンを取得
    def getPheromone(self, x, y):
        return self.grid[x, y]

    # 指定した座標(x, y)へフェロモンを配置
    def setPheromone(self, x, y, value):
        self.grid[x, y] = value

    # 正方形の形でフェロモンの射出する
    def injectionSquare(self, x, y, pheromone_value, injection_size,
                        max_pheromone_value):
        # 例外処理 : フェロモンを射出するサイズはかならず奇数
        if injection_size % 2 == 0:
            raise Exception("Pheromone injection size must be an odd number.")
        # 現在時刻を取得
        current_time = time.process_time()
        # フェロモンを射出する間隔を0.1s間隔に設定
        if current_time - self.injection_timer > 0.1:
            for i in range(injection_size):
                for j in range(injection_size):
                    # 指定した範囲のセルにフェロモンを配置
                    self.grid[x-(injection_size-1)/2+i,
                              y-(injection_size-1)/2+j] += pheromone_value
                    # 配置するフェロモン値がフェロモンの最大値より大きければカット
                    if self.grid[x-(injection_size-1)/2+i,
                                 y-(injection_size-1)/2+j] >=\
                            max_pheromone_value:
                        self.grid[x-(injection_size-1)/2+i,
                                  y-(injection_size-1)/2+j] =\
                            max_pheromone_value
            # フェロモンを射出した時間を記録
            self.injection_timer = current_time

    def injectionCircle(self, x, y, value, radius):
        radius = int(radius*self.resolution)
        for i in range(-radius, radius):
            for j in range(-radius, radius):
                if math.sqrt(i**2+j**2) <= radius:
                    print('sqrt = ' + str(math.sqrt(i**2+j**2)) +
                          ', +radious = ' + str(radius))
                    print('injection pos_index : [x, y] = [' +
                          str(x+i) + ', ' + str(y+j) + ']')
                self.grid[x+i, y+j] = value
        # circumference = radius * 2.0 * math.pi
        # split_circle = int(circumference / self.resolution * 100)
        # for theta in range(0, 360, 360/split_circle):
        #     print(theta)

    def update(self, min_pheromone_value, max_pheromone_value):
        current_time = time.process_time()
        time_elapsed = current_time - self.update_timer
        self.update_timer = current_time

        # 拡散を行うかどうかの判定
        if self.isDiffusion is True:
            # Diffusion
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    self.grid_copy[i, j] += 0.9*self.grid[i, j]
                    if i >= 1:
                        self.grid_copy[i-1, j] += 0.025*self.grid[i, j]
                    if j >= 1:
                        self.grid_copy[i, j-1] += 0.025*self.grid[i, j]
                    if i < self.num_cell-1:
                        self.grid_copy[i+1, j] += 0.025*self.grid[i, j]
                    if j < self.num_cell-1:
                        self.grid_copy[i, j+1] += 0.025*self.grid[i, j]
            # 最大と最小を丸める
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    if self.grid_copy[i, j] < min_pheromone_value:
                        self.grid_copy[i, j] = min_pheromone_value
                    elif self.grid_copy[i, j] > max_pheromone_value:
                        self.grid_copy[i, j] = max_pheromone_value
            # グリッドセルを更新
            self.grid = np.copy(self.grid_copy)
            # 複製したグリッドセルを初期化
            self.grid_copy = np.zeros((self.num_cell, self.num_cell))

        # 蒸発を行うかどうかの判定
        if self.isEvaporation is True:
            # Evaporation
            decay = 2**(-time_elapsed/self.evaporation)
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    self.grid[i, j] = decay * self.grid[i, j]

    def save(self, file_name):
        parent = Path(__file__).resolve().parent
        with open(parent.joinpath('pheromone_saved/' +
                                  file_name + '.pheromone'), 'wb') as f:
            np.save(f, self.grid)
        # print("The pheromone matrix {} is successfully saved".
              # format(file_name))

    def load(self, file_name):
        parent=Path(__file__).resolve().parent
        with open(parent.joinpath('pheromone_saved/' +
                                  file_name + '.npy'), 'rb') as f:
            self.grid=np.load(f)
        print("The pheromone matrix {} is successfully loaded".
              format(file_name))


if __name__ == "__main__":
    rospy.init_node('pheromone')
    node1=Node()
    rospy.spin()
