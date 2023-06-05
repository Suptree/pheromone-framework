#!/usr/bin/env python3

import sys

import roslib
import os
import numpy as np
import tf
import rospy
import time
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
# from pheromone.srv import pheromone_injection, pheromone_injection_response
# from pheromone.srv import pheromone_goal, pheromone_goal_response
# from pheromone.srv import pheromone_reset, pheromone_reset_response
import math


class Node():
    def __init__(self, pheromone):
        self.pheromone = pheromone
        self.pheromone_max = 1.0
        self.pheromone_min = 0.0
        self.is_pheromone_injection = True

        # Publisher & Subscriber
        self.publish_pheromone = rospy.Publisher('/pheromone_value',
                                                 Float32MultiArray,
                                                 queue_size=10)
        self.subscribe_pose = rospy.Subscriber('/gazebo/model_states',
                                               ModelStates,
                                               self.pheromoneCallback,
                                               self.pheromone)

        # Services
        # self.srv_injection = rospy.Service('pheromone_injection',
        #                                    pheromone_injection,
        #                                    self.injectionAssign)
        # self.srv_goal = rospy.Service('pheromone_goal',
        #                               pheromone_goal,
        #                               self.next_goal)
        # self.srv_reset = rospy.Service('pheromone_test',
        #                                pheromone_reset,
        #                                self.serviceRest)

        self.is_service_requested = False
        self.theta = 0

        self.log_timer = time.process_time()
        self.log_file = open("phero_value.txt", "a+")
        self.is_saved = False
        self.is_loaded = False
        self.is_reset = True

        self.pheromone.is_diffusion = True
        self.pheromone.is_evaporation = False
        self.start_time = time.time()

    def posToIndex(self, x, y):
        pheromone = self.pheromone

        resolution = self.pheromone.resolution  # グリッドセルの解像度
        round_decimal_places = int(math.log10(resolution))
        x = round(x, round_decimal_places)
        y = round(y, round_decimal_places)
        x = int(x*resolution)
        y = int(y*resolution)

        x_index = int(x + (pheromone.num_cell - 1)/2)
        y_index = int(y + (pheromone.num_cell - 1)/2)

        if x_index < 0 or y_index < 0 or x_index > pheromone.num_cell-1 \
                or y_index > pheromone.num_cell-1:
            raise Exception("The pheromone matrix index is out of range.")
        return x_index, y_index

    def indexToPos(self, x_index, y_index):
        '''
        Convert matrix indices into 2D coordinate (x, y)
        '''
        x = x_index - (self.pheromone.num_cell-1)/2
        y = y_index - (self.pheromone.num_cell-1)/2

        x = float(x) / self.pheromone.resolution
        y = float(y) / self.pheromone.resolution

        return x, y

    def pheromoneCallback(self, message, cargs):
        # Reading from arguments
        pose = message.pose[-1]
        twist = message.twist[-1]
        pos = pose.position
        ori = pose.orientation
        pheromone = cargs
        x = pos.x
        y = pos.y

        angles = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w))
        if angles[2] < 0:
            self.theta = angles[2] + 2*math.pi
        else:
            self.theta = angles[2]

        # 9 pheromone values
        # Position of 9 cells surrounding the robot
        x_index, y_index = self.posToIndex(x, y)
        phero_val = Float32MultiArray()
        for i in range(3):
            for j in range(3):
                phero_val.data.append(
                    self.pheromone.getPhero(x_index+i-1, y_index+j-1))
        # print("phero_avg: {}".format(np.average(np.asarray(phero_val.data))))
        self.publish_pheromone.publish(phero_val)
        # # Assign pheromone value and publish it
        # phero_val = phero.getPhero(x_index, y_index)
        # self.pub_phero.publish(phero_val)


class Pheromone():

    def __init__(self, size=10, res=50, evaporation=0.0, diffusion=0.0):
        self.resolution = res  # grid cell size = 1 m / resolution
        self.size = size  # m
        self.num_cell = self.resolution * self.size + 1
        if self.num_cell % 2 == 0:
            raise Exception(
                "Number of cell is even. It needs to be an odd number")
        self.grid = np.zeros((self.num_cell, self.num_cell))
        self.grid_copy = np.zeros((self.num_cell, self.num_cell))
        self.evaporation = evaporation  # elapsed seconds for pheromone to be halved
        self.diffusion = diffusion
        self.isDiffusion = True
        self.isEvaporation = True

        # Timers
        self.update_timer = time.process_time()
        self.step_timer = time.process_time()
        self.injection_timer = time.process_time()
        self.save_timer = time.process_time()
        self.reset_timer = time.process_time()

        # self.path = path

    def getPhero(self, x, y):
        return self.grid[x, y]

    def setPhero(self, x, y, value):
        self.grid[x, y] = value

    # Inject pheromone at the robot position and nearby cells in square. Size must be an odd number.
    def injection(self, x, y, value, size, max):
        if size % 2 == 0:
            raise Exception("Pheromone injection size must be an odd number.")
        time_cur = time.process_time()
        if time_cur-self.injection_timer > 0.1:
            for i in range(size):
                for j in range(size):
                    self.grid[x-(size-1)/2+i, y-(size-1)/2+j] += value
                    if self.grid[x-(size-1)/2+i, y-(size-1)/2+j] >= max:
                        self.grid[x-(size-1)/2+i, y-(size-1)/2+j] = max
            self.injection_timer = time_cur

    def circle(self, x, y, value, radius):
        radius = int(radius*self.resolution)
        for i in range(-radius, radius):
            for j in range(-radius, radius):
                if math.sqrt(i**2+j**2) <= radius:
                    self.grid[x+i, y+j] = value

    # Update all the pheromone values depends on natural phenomena, e.g. evaporation

    def update(self, min, max):
        time_cur = time.process_time()
        time_elapsed = time_cur - self.update_timer
        self.update_timer = time_cur

        if self.isDiffusion == True:
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
            # self.grid_copy = np.clip(self.grid_copy, a_min = min, a_max = max)
            self.grid = np.copy(self.grid_copy)
            self.grid_copy = np.zeros((self.num_cell, self.num_cell))
        if self.isEvaporation == True:
            # evaporation
            decay = 2**(-time_elapsed/self.evaporation)
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    self.grid[i, j] = decay * self.grid[i, j]



if __name__ == "__main__":
    rospy.init_node('pheromone')
    pheromone = Pheromone()
    node1 = Node(pheromone=pheromone)
    rospy.spin()
