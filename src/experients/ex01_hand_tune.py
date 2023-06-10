#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

from gazebo_msgs.msg import ModelStates
import math
import tf
import time
from geometry_msgs.msg import Twist


class WaypointNavigation:
    MAX_FORWARD_SPEED = 0.1
    MAX_ROTATION_SPEED = 1.0
    cmdmsg = Twist()
    index = 0

    # Tunable parameters
    wGain = 10
    vConst = 0.3
    distThr = 0.2
    pheroThr = 0.2

    def __init__(self):
        # Initialise pheromone values
        self.pheromone_value = [0.0] * 9
        self.sum_pheromone_value = 0.0

        self.robot_theta = 0

        self.goal_pos_x = 0.8
        self.goal_pos_y = 0.0

        self.sub_phero = rospy.Subscriber(
            '/pheromone_value', Float32MultiArray, self.ReadPheromone)
        self.sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.Callback)
        self.pub = rospy.Publisher('/hero_0/cmd_vel', Twist, queue_size=1)
        self.reset_timer = time.process_time()
        self.beta_const = 1.2
        self.sensitivity = 1.2
        self.BIAS = 0.25
        self.V_COEF = 1.0  # self.v_range[0]
        self.W_COEF = 0.2  # self.w_range[0]

    def ReadPheromone(self, pheromone_message):
        pheromone_data = pheromone_message.data
        self.pheromone_value = pheromone_data
        self.sum_pheromone_value = np.sum(np.asarray(pheromone_data))

    def Callback(self, model_status):

        robot_index = model_status.name.index('hero_0')

        pose = model_status.pose[robot_index]
        # twist = model_status.twist[robot_index]

        pos = pose.position
        ori = pose.orientation

        angles = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w))

        self.theta = angles[2]

        # P controller
        v = 0
        w = 0
        # print("pos: {}".format(pos))
        # Index for # of goals
        distance = math.sqrt((pos.x-self.goal_pos_x)**2
                             + (pos.y-self.goal_pos_y)**2)
        # print('distance : ' + str(distance))
        # Reset condition reset (to prevent unwanted reset due to delay of position message subscription)
        step_timer = time.process_time()
        reset_time = step_timer - self.reset_timer

        msg = Twist()
        if (distance <= self.distThr and reset_time > 1):
            print("Goal!!")
            self.is_goal = True
            self.reset()
        elif (self.sum_pheromone_value > self.pheroThr):
            print("pheromone\n")
            msg = self.PheroResponse(self.pheromone_value)
            # msg = self.PheroOA(self.pheromone_value)
            v = msg.linear.x
            w = msg.angular.z

        # Adjust velocities
        elif (distance > self.distThr):
            # print("not pheromone\n")
            v = self.vConst
            yaw = math.atan2(self.goal_pos_y-pos.y, self.goal_pos_x-pos.x)
            u = yaw - self.theta
            bound = math.atan2(math.sin(u), math.cos(u))
            w = min(2.0, max(-2.0, self.wGain*bound))
            msg.linear.x = v
            # print(v)
            # print(" , ")
            msg.angular.z = w
            # print(w)
            self.reset_flag = False

        # distance_to_obs = [1.0]*len(self.obstacle)
        # for i in range(len(distance_to_obs)):
        #     distance_to_obs[i] = sqrt((pos.x-self.obstacle[i][0])**2+(pos.y-self.obstacle[i][1])**2)
        # if (distance_to_obs[0] < 0.3 or distance_to_obs[1] < 0.3 or distance_to_obs[2] < 0.3 or distance_to_obs[3] < 0.3) and reset_time > 1:
        #     msg = Twist()
        #     self.is_collided = True
        #     self.reset()

        if reset_time > 40.0:
            print("Times up!")
            self.is_timeout = True
            self.reset()

        if msg.linear.x > self.MAX_FORWARD_SPEED:
            msg.linear.x = self.MAX_FORWARD_SPEED
        # Publish velocity
        self.pub.publish(msg)

        self.prev_x = pos.x
        self.prev_y = pos.y

        # Reporting
        # print("Distance to goal {}".format(distance))
        # print('Callback: x=%2.2f, y=%2.2f, dist=%4.2f, cmd.v=%2.2f, cmd.w=%2.2f' %(pos.x,pos.y,distance,v,w))

    def velCoef(self, value1, value2):
        '''
        - val_avg (0, 1)
        - val_dif (-1, 1)
        - dif_coef (1, 2.714)
        - coefficient (-2.714, 2.714)
        '''
        val_avg = (value1 + value2)/2
        val_dif = value1 - value2
        dif_coef = math.exp(val_avg)

        return dif_coef*val_dif

    def PheroOA(self, phero):
        '''
        Pheromone-based obstacle avoidance algorithm
        - Input: 9 cells of pheromone
        - Output: Twist() to avoid obstacle
        '''
        # Constants:
        # Constants:
        BIAS = self.BIAS
        V_COEF = self.V_COEF
        W_COEF = self.W_COEF
        # BIAS = 0.25
        # V_COEF = 0.2
        # W_COEF = 0.3

        # Initialise values
        # values are assigned from the top left (135 deg) to the bottom right (-45 deg) ((0,1,2),(3,4,5),(6,7,8))
        avg_phero = np.average(np.asarray(phero))
        unit_vecs = np.asarray(
            [[1, 0], [math.sqrt(2)/2, math.sqrt(2)/2], [0, 1], [-math.sqrt(2)/2, math.sqrt(2)/2]])
        vec_coefs = [0.0] * 4
        twist = Twist()

        # Calculate vector weights
        vec_coefs[0] = self.velCoef(phero[5], phero[3])
        vec_coefs[1] = self.velCoef(phero[2], phero[6])
        vec_coefs[2] = self.velCoef(phero[1], phero[7])
        vec_coefs[3] = self.velCoef(phero[0], phero[8])
        vec_coefs = np.asarray(vec_coefs).reshape(4, 1)
        vel_vecs = np.multiply(unit_vecs, vec_coefs)
        vel_vec = np.sum(vel_vecs, axis=0)

        ang_vel = W_COEF*math.atan2(vel_vec[1], vel_vec[0])

        # Velocity assignment
        twist.linear.x = BIAS + V_COEF*avg_phero
        twist.angular.z = ang_vel

        return twist

    def PheroResponse(self, phero):
        # takes two pheromone input from antennae
        avg_phero = np.average(np.asarray(phero))
        beta = self.beta_const - avg_phero
        s_l = beta + (phero[0] - phero[1])*self.sensitivity
        s_r = beta + (phero[1] - phero[0])*self.sensitivity
        twist = Twist()

        twist.linear.x = (s_l + s_r)/2
        twist.angular.z = (s_l - s_r)

        return twist


if __name__ == '__main__':
    rospy.init_node('pose_reading')
    wayN = WaypointNavigation()
    rospy.spin()
