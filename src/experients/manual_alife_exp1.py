#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import time
import csv
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
# from turtlebot3_pheromone.srv import PheroReset, PheroResetResponse
import math
from std_msgs.msg import Float32MultiArray


class WaypointNavigation:

    MAX_FORWARD_SPEED = 0.5
    MAX_ROTATION_SPEED = 1.0
    cmdmsg = Twist()
    index = 0

    # Tunable parameters
    wGain = 10
    vConst = 0.3
    distThr = 0.2
    pheroThr = 0.2

    def __init__(self):

        self.num_robots = 1
        self.num_experiments = 4

        # Initialise pheromone values
        self.phero = [0.0] * 9
        self.phero_sum = 0.0

        # Initialise speed
        self.move_cmd = Twist()

        # Initialise positions
        self.goal = [1, 1]
        self.obstacle = [[2, 0], [-2, 0], [0, 2], [0, -2]]

        # Initialise ros related topics
        self.pub = rospy.Publisher('/hero_0/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.Callback, (self.pub, self.cmdmsg, self.goal))
        # self.sub_test = rospy.Subscriber('/gazebo/model_states', ModelStates, self.testCallback)
        self.sub_phero = rospy.Subscriber(
            '/pheromone_value', Float32MultiArray, self.ReadPhero)
        self.rate = rospy.Rate(300)

        self.target_x = 4
        self.target_y = 0

        self.prev_x = 0.0
        self.prev_y = 0.0

        # Initialise parameters

        self.beta_const = 1.2
        self.sensitivity = 1.2

        self.step_size = 0.05
        # self.b_range = np.arange(0, 1+self.step_size, self.step_size)
        self.v_range = np.arange(0.2, 1+self.step_size, self.step_size)
        self.w_range = np.arange(0.2, 1+self.step_size, self.step_size)

        self.BIAS = 0.25
        self.V_COEF = 1.0  # self.v_range[0]
        self.W_COEF = 0.2  # self.w_range[0]

        # self.b_size = self.b_range.size
        self.v_size = self.v_range.size
        self.w_size = self.w_range.size

        self.b_counter = 0
        self.v_counter = 0
        self.w_counter = 0

        # Initialise simulation
        self.counter_step = 0
        self.counter_collision = 0
        self.counter_success = 0
        self.arrival_time = []
        self.target_index = 0
        self.radius = 4

        # Flags
        self.is_collided = False
        self.is_goal = False
        self.is_timeout = False

        # File name
        self.time_str = time.strftime("%Y%m%d-%H%M%S")
        self.file_name = "manual_{}_{}".format(self.num_robots, self.time_str)
        self.traj_name = "{}_traj".format(self.file_name)
        print(self.file_name)

        # Initialise simulation
        self.reset_timer = time.time()
        self.reset()
        self.reset_flag = False

        # antenna movement related parameters
        self.b_range = np.arange(1.1, 1.1+self.step_size, self.step_size)
        self.s_range = np.arange(0.8, 0.8+self.step_size, self.step_size)

        self.b_size = self.b_range.size
        self.s_size = self.s_range.size

        self.b_counter = 0
        self.s_counter = 0

        # Log related
        self.log_timer = time.time()

    def ReadPhero(self, message):
        phero_data = message.data
        self.phero = phero_data
        self.phero_sum = np.sum(np.asarray(phero_data))

    def Callback(self, message, cargs):
        '''
        Main Function
        - Receive position data
        - Generate action
        '''
        # ========================================================================= #
        #                           Initialisation                                  #
        # ========================================================================= #

        pub, msg, goal = cargs
        goal = self.goal

        for i in range(len(message.name)):
            if message.name[i] == 'hero_0':
                tb3 = i
            if message.name[i] == 'unit_sphere':
                tg = i
        pose = message.pose[tb3]
        twist = message.twist[tb3]

        pos = pose.position
        ori = pose.orientation
        angles = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w))

        theta = angles[2]

        # P controller
        v = 0
        w = 0
        # print("pos: {}".format(pos))
        # Index for # of goals
        index = self.index
        distance = math.sqrt((pos.x-self.target_x)**2+(pos.y-self.target_y)**2)

        # Reset condition reset (to prevent unwanted reset due to delay of position message subscription)
        step_timer = time.time()
        reset_time = step_timer - self.reset_timer

        # Log Positions
        # if time.time() - self.log_timer > 0.5:
        #     for i in range(self.num_robots):
        #         with open('/home/swn/catkin_ws/src/Turtlebot3_Pheromone/src/log/csv/{}.csv'.format(self.traj_name), mode='a') as csv_file:
        #                 csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #                 csv_writer.writerow(['%0.1f'%reset_time, '%i'%i, '%0.2f'%pos.x, '%0.2f'%pos.y])
        #     self.log_timer = time.time()

        # ========================================================================= #
        #                          Action & State assignment                        #
        # ========================================================================= #

        if (distance <= self.distThr and reset_time > 1):
            print("wow")
            msg = Twist()
            self.is_goal = True
            self.reset()

        elif (self.phero_sum > self.pheroThr):
            print("pheromone\n")
            msg = self.PheroOA(self.pheromone_value)
            v = msg.linear.x
            w = msg.angular.z
        # Adjust velocities
        elif (distance > self.distThr):
            # print("not pheromone\n")
            v = self.vConst
            yaw = math.atan2(self.target_y-pos.y, self.target_x-pos.x)
            u = yaw - theta
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

        # Publish velocity
        self.pub.publish(msg)

        self.prev_x = pos.x
        self.prev_y = pos.y

        # Reporting
        # print("Distance to goal {}".format(distance))
        # print('Callback: x=%2.2f, y=%2.2f, dist=%4.2f, cmd.v=%2.2f, cmd.w=%2.2f' %(pos.x,pos.y,distance,v,w))

    # Angular velocity coefficient (When avg phero is high, it is more sensitive)

    def velCoef(self, value1, value2):
        '''
        - val_avg (0, 1)
        - val_dif (-1, 1)
        - dif_coef (1, 2.714)
        - coefficient (-2.714, 2.714)
        '''
        val_avg = (value1 + value2)/2
        val_dif = value1 - value2
        dif_coef = exp(val_avg)

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
            [[1, 0], [math.sqrt(2)/2, math.sqrt(2)/2], [0, 1], [-math.sqrt(2)/2, sqrt(2)/2]])
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

    def reset(self):
        '''
        Resettng the Experiment
        1. Update the counter based on the flag from step
        2. Assign next positions and reset
        3. Log the result in every selected time-step
        '''

        # ========================================================================= #
        #                           COUNTER UPDATE                                  #
        # ========================================================================= #

        # Increment Collision Counter
        if self.is_collided == True:
            print("Collision!")
            self.counter_collision += 1
            self.counter_step += 1

        # Increment Arrival Counter and store the arrival time
        if self.is_goal == True:
            print("Arrived goal!")
            self.counter_success += 1
            self.counter_step += 1
            arrived_timer = time.time()
            art = arrived_timer-self.reset_timer
            self.arrival_time.append(art)
            print("Episode time: %0.2f" % art)

        if self.is_timeout == True:
            self.counter_collision += 1
            self.counter_step += 1
            print("Timeout!")

        print("counter_step: {}, counter_success: {}, counter_collision: {}".format(
            self.counter_step, self.counter_success, self.counter_collision))

        # Reset the flags
        self.is_collided = False
        self.is_goal = False
        self.is_timeout = False

        # ========================================================================= #
        #                                  RESET                                    #
        # ========================================================================= #

        # angle_target = self.target_index*2*pi/self.num_experiments + 0.01*pi

        # self.target_x = self.radius*cos(angle_target)
        # self.target_y = self.radius*sin(angle_target)

        # if self.target_index < self.num_experiments:
        #     self.target_index += 1
        # else:
        #     self.target_index = 0

        # self.is_collided = False

        # self.theta = angle_target
        # quat = quaternion_from_euler(0,0,self.theta)

        # Reset Turtlebot position
        state_msg = ModelState()
        state_msg.model_name = 'hero_0'
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 0.1
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        state_msg.pose.orientation.w = 0.0

        # Reset Target Position
        state_target_msg = ModelState()
        # 'unit_sphere_0_0' #'unit_box_1' #'cube_20k_0'
        state_target_msg.model_name = 'unit_sphere'
        state_target_msg.pose.position.x = 4.0
        state_target_msg.pose.position.y = 0.0
        state_target_msg.pose.position.z = 0.0
        state_target_msg.pose.orientation.x = 0
        state_target_msg.pose.orientation.y = 0
        state_target_msg.pose.orientation.z = 0
        state_target_msg.pose.orientation.w = 0

        rospy.wait_for_service('gazebo/reset_simulation')

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
            resp_targ = set_state(state_target_msg)
        except rospy.ServiceException as e:
            print("Service Call Failed: %s" % e)

        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.pub.publish(self.move_cmd)
        self.pub.publish(self.move_cmd)

        rospy.wait_for_service('phero_reset')
        try:
            phero_reset = rospy.ServiceProxy('phero_reset', PheroReset)
            resp = phero_reset(True)
            print("Reset Pheromone grid successfully: {}".format(resp))
        except rospy.ServiceException as e:
            print("Service Failed %s" % e)

        # ========================================================================= #
            #                                  LOGGING                                  #
            # ========================================================================= #

        # if self.counter_step == 0:
        #     with open('/home/swn/catkin_ws/src/Turtlebot3_Pheromone/src/log/csv/{}.csv'.format(self.file_name), mode='w') as csv_file:
        #         csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #         csv_writer.writerow(['Episode', 'Beta_const', 'Sensitivity', 'Success Rate', 'Average Arrival time', 'Standard Deviation'])
        #     with open('/home/swn/catkin_ws/src/Turtlebot3_Pheromone/src/log/csv/{}.csv'.format(self.traj_name), mode='w') as csv_file:
        #         csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #         csv_writer.writerow(['time', 'ID', 'x', 'y'])

        if self.counter_step != 0:
            if (self.counter_collision != 0 or self.counter_success != 0):
                succ_percentage = 100*self.counter_success / \
                    (self.counter_success+self.counter_collision)
            else:
                succ_percentage = 0
            print("Counter: {}".format(self.counter_step))

        if (self.counter_step % 1 == 0 and self.counter_step != 0):
            print("Beta_const: {}, Sensitivity: {}".format(
                self.beta_const, self.sensitivity))
            print("Success Rate: {}%".format(succ_percentage))

        if (self.counter_step % 1 == 0 and self.counter_step != 0):
            avg_comp = np.average(np.asarray(self.arrival_time))
            std_comp = np.std(np.asarray(self.arrival_time))
            print("{} trials ended. Success rate: {}, average completion time: {}, Standard deviation: {}".format(
                self.counter_step, succ_percentage, avg_comp, std_comp))

            with open('/home/swn/catkin_ws/src/Turtlebot3_Pheromone/src/log/csv/{}.csv'.format(self.file_name), mode='a') as csv_file:
                csv_writer = csv.writer(
                    csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(['%i' % self.counter_step, '%0.2f' % self.beta_const, '%0.2f' %
                                    self.sensitivity, '%0.2f' % succ_percentage, '%0.2f' % avg_comp, '%0.2f' % std_comp])

            self.paramUpdate()
            self.arrival_time = []
            self.counter_collision = 0
            self.counter_success = 0
            self.target_index = 0

        self.reset_timer = time.time()
        self.reset_flag = True

    def paramUpdate(self):
        '''
        Parameter update after the number of experiments for a parameter set finished
        '''
        print("Parameters are updated!")
        if (self.s_counter < self.s_size-1):
            self.s_counter += 1
            self.sensitivity = self.s_range[self.s_counter]
        elif (self.b_counter < self.b_size-1):
            self.s_counter = 0
            self.b_counter += 1
            self.sensitivity = self.s_range[self.s_counter]
            self.beta_const = self.b_range[self.b_counter]
        else:
            print("Finish Iteration of parameters")
            sys.exit()


if __name__ == '__main__':
    rospy.init_node('pose_reading')
    wayN = WaypointNavigation()
    rospy.spin()
