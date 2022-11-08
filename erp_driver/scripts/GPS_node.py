#! /usr/bin/env python

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import time
from geometry_msgs.msg import Pose2D, PoseStamped, QuaternionStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16, Float32
from erp_driver.msg import erpStatusMsg, erpCmdMsg


class GPS():
    def __init__(self):
        rospy.init_node('gps_node')
        self.sub1 = rospy.Subscriber("/utm", PoseStamped, self.poseCallback)
        self.sub2 = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.Position_Callback)
        self.sub3 = rospy.Subscriber("/filter/quaternion", QuaternionStamped, self.YAWCallback)
        self.lidar_sub = rospy.Subscriber("/ob_mode",Int16, self.lidar_callback)
        self.status_sub = rospy.Subscriber("/erp42_status", erpStatusMsg, self.Status_Callback)

        self.pub1 = rospy.Publisher('/steering', erpCmdMsg, queue_size=10)
        self.pub2 = rospy.Publisher('/speed', Int16, queue_size=10)
        self.pub3 = rospy.Publisher('/mode',Int16, queue_size=1)

        rospy.on_shutdown(self.kill_function)

        self.cmd_msg_steer = erpCmdMsg()
        self.status = erpStatusMsg()

        self.ERP42_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.goal_pose2 = Pose2D()
        self.covariance_value = NavSatFix()

        self.s_angle = Float32()
        self.c_speed = Int16()
        self.mode_num = Int16()
        self.mode_num.data = 0
        self.wp_go = 0
        self.wp_len = 0
        self.lidar_data = 0
        self.lidar_cnt = 0
        self.status_speed = 1
        self.k = 49

        self.log_x = []
        self.log_y = []
        self.log_insang = []
        self.travel_dis = 0
        self.prev_time = 0

        self.pos_error_x = 10.0
        self.pos_error_y = 10.0

        self.error = 0.0
        self.error_old = 0.0
        self.error_d = 0.0
        self.error_sum = 0.0

        self.car_speed = 50
        self.yaw_deg = 0

        self.new_path = []
        self.flag = 0
        self.count = 0
        self.before1_steer = 0
        self.before2_steer = 0

        self.loop()


    def poseCallback(self, msg):
        self.ERP42_pose.x = msg.pose.position.x
        self.ERP42_pose.y = msg.pose.position.y
        self.count = -1

        print("X_coord : ", self.ERP42_pose.x)
        print("y_coord : ", self.ERP42_pose.y)

    def Position_Callback(self, msg):
        self.covariance_value = msg
        self.cvr_value = self.covariance_value.position_covariance[0] # why?

    def goalCallback(self, msg):
        self.goal_pose.x = msg.x
        self.goal_pose.y = msg.y
        # rospy.loginfo(self.goal_pose.x)

    def YAWCallback(self, msg):
        self.ERP42_pose.theta = msg.quaternion.w + 15
        # self.IMU = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
        # self.roll, self.pitch, self.yaw = euler_from_quaternion(self.IMU)
        # self.roll_deg =self.roll/pi*180
        # self.pitch_deg = self.pitch/pi*180
        # self.yaw_deg = self.yaw/pi*180
        # #print(self.yaw_deg)

    def lidar_callback(self, msg):
        self.lidar_data = msg.data

    def Status_Callback(self, msg):
        self.status.speed = msg.speed+1


    def RAD2DEG(self, x):
        a = ((x)*180 / math.pi)
        return a

    def kill_function(self):
        fig = plt.figure(figsize=(20,10))
        # plt.xlim([min(self.new_CSV['X'])-5, max(self.new_CSV['X'])+5]) # -5,+5
        # plt.ylim([min(self.new_CSV['Y'])-5, max(self.new_CSV['Y'])+5])
        # plt.xlim([315380,315391]) # -5,+5
        # plt.ylim([4071652,4071662])
        # plt.plot(self.new_CSV['X'],self.new_CSV['Y'], color='blue', linestyle='solid', linewidth=1,markersize=5)
        # plt.plot(self.log_x, self.log_y,  color='red', linestyle='solid', markersize=6)
        self.log_x = []  # reset log list for kill node(keyboard interrupt)
        self.log_y = []
        print(self.log_insang)
        # plt.show()

    def create_path(self):

        read_path = pd.read_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv', keep_default_na=False)
        car = [self.ERP42_pose.x, self.ERP42_pose.y]
        path = []
        dist = []
        new_path = []

        print('Creating New Path...')

        for i in range(len(read_path['X'])):
            path.append([read_path['X'][i],read_path['Y'][i]])
            dist.append(math.sqrt(( car[0] - read_path['X'][i] ) ** 2 + ( car[1] - read_path['Y'][i] ) ** 2 ))

        min_dist = dist.index(min(dist))  #+1

        for i in range(min_dist, len(read_path['X'])):
            new_path.append([read_path['X'][i],read_path['Y'][i]])

        self.new_path = new_path
        self.wp_len = len(new_path)

        print('Save New Path...')
        self.new_CSV = pd.DataFrame(self.new_path, columns=['X','Y'])
        self.new_CSV.to_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv',index=None)


    def loop(self):
        rate = rospy.Rate(100)
        start_time = time.time()

        while not rospy.is_shutdown():
            if(self.count == -1 and self.flag == 0):
                self.create_path()
                self.flag = 1
            if(self.flag == 1):
                # if (math.sqrt(pow(abs(self.ERP42_pose.x - 315377.7), 2) + pow(abs(self.ERP42_pose.y - 4071610.8), 2)) <=  1.0):
                #     self.mode_num.data = 1
                # elif(math.sqrt(pow(abs(self.ERP42_pose.x - 315389.8), 2) + pow(abs(self.ERP42_pose.y - 4071656.6), 2)) <=  1.5):
                #     self.mode_num.data = 2
                # elif(math.sqrt(pow(abs(self.ERP42_pose.x - 315418.7), 2) + pow(abs(self.ERP42_pose.y - 4071636.9), 2)) <=  1.5):
                #     self.mode_num.data = 1
                if(self.wp_go == 0):
                    self.goal_pose.x = self.new_path[self.wp_go][0]
                    self.goal_pose.y = self.new_path[self.wp_go][1]
                    self.goal_pose2.x = self.new_path[self.wp_go+1][0]
                    self.goal_pose2.y = self.new_path[self.wp_go+1][1]

                    self.pos_error_x = abs(self.ERP42_pose.x - self.goal_pose.x)
                    self.pos_error_y = abs(self.ERP42_pose.y - self.goal_pose.y)

                    tf_base_map_x = self.goal_pose.x - self.ERP42_pose.x
                    tf_base_map_y = self.goal_pose.y - self.ERP42_pose.y
                    self.theta_star = self.RAD2DEG(math.atan2(tf_base_map_y,tf_base_map_x))

                    if(self.theta_star < 0) :
                        self.theta_star += 360

                    self.error = self.theta_star - self.ERP42_pose.theta

                    if self.error < -180 : #-180
                        self.error += 360
                    elif self.error > 180 : #180
                        self.error += -360

                    self.change_x = self.goal_pose2.x - self.goal_pose.x
                    self.change_y = self.goal_pose2.y - self.goal_pose.y
                    self.theta_star2 = self.RAD2DEG(math.atan2(self.change_y, self.change_x))

                    if(self.theta_star2 < 0) :
                        self.theta_star2 += 360

                    self.error2 = self.theta_star2 - (self.ERP42_pose.theta)

                    if self.error2 < -180 : #-180
                        self.error2 += 360
                    elif self.error2 > 180 : #180
                        self.error2 += -360

                    self.steer_angle = (self.error + self.error2)

                else:
                    self.goal_pose.x = self.new_path[self.wp_go-1][0]
                    self.goal_pose.y = self.new_path[self.wp_go-1][1]
                    self.goal_pose2.x = self.new_path[self.wp_go][0]
                    self.goal_pose2.y = self.new_path[self.wp_go][1]

                    self.pos_error_x = abs(self.ERP42_pose.x - self.goal_pose2.x)
                    self.pos_error_y = abs(self.ERP42_pose.y - self.goal_pose2.y)

                    self.change_x = self.goal_pose2.x - self.goal_pose.x
                    self.change_y = self.goal_pose2.y - self.goal_pose.y

                    if(self.change_x == 0.0): self.change_x = 0.0000001
                    if(self.change_y == 0.0): self.change_y = 0.0000001

                    self.alpha_son = self.change_y*self.ERP42_pose.x - self.change_x*self.ERP42_pose.y + self.change_x*(self.goal_pose.y-(self.change_y/self.change_x)*self.goal_pose.x)
                    self.alpha_mom = math.sqrt(self.change_y**2 + self.change_x**2)
                    self.alpha = self.alpha_son / self.alpha_mom

                    self.error = self.RAD2DEG(math.atan(self.k*self.alpha/self.car_speed))#/self.status.speed)

                    # rospy.loginfo(self.error)


                    self.theta_star2 = self.RAD2DEG(math.atan2(self.change_y, self.change_x))

                    if(self.theta_star2 < 0) :
                        self.theta_star2 += 360

                    self.error2 = self.theta_star2 - (self.ERP42_pose.theta)

                    if self.error2 < -180 : #-180
                        self.error2 += 360
                    elif self.error2 > 180 : #180
                        self.error2 += -360

                    self.steer_angle = (self.error + self.error2)

                # int((self.steer_angle + self.before1_steer + self.before2_steer)/3) #self.steer_angle
                # self.before1_steer = self.steer_angle
                # self.before2_steer = self.before1_steer
                self.c_speed.data = self.car_speed
                # self.cmd_msg_steer.steer = int(self.s_angle.data)

                # if (math.sqrt(pow(self.pos_error_x, 2) + pow(self.pos_error_y, 2)) <=  1.5):
                #         self.wp_go += 1

                if(self.wp_go < self.wp_len-10):
                    if (math.sqrt(pow(self.pos_error_x, 2) + pow(self.pos_error_y, 2)) <=  2.2):
                        self.wp_go += 1

                    if (self.lidar_data == 1 and self.lidar_cnt == 0):
                        self.wp_go += 10
                        self.lidar_cnt = 1
                elif(self.wp_go >= self.wp_len-10 and self.wp_go < self.wp_len-2):
                    if (math.sqrt(pow(self.pos_error_x, 2) + pow(self.pos_error_y, 2)) <=  2.2):
                        self.wp_go += 1
                    # self.car_speed = 0

                elif(self.wp_go <= self.wp_len-1):
                    pass
                    # self.car_speed = 0
                    # self.steer_angle = 0

                if int((time.time() - start_time)*100) % 1 == 0: # 2Hz sampling
                    self.log_x.append(self.ERP42_pose.x)
                    self.log_y.append(self.ERP42_pose.y)

                self.cmd_msg_steer.steer = int(self.steer_angle)
                self.cmd_msg_steer.steer = np.clip(self.cmd_msg_steer.steer, -28, 28)

                # rospy.loginfo(self.wp_go)

                self.pub1.publish(self.cmd_msg_steer)
                self.pub2.publish(self.c_speed)
                self.pub3.publish(self.mode_num)

                self.travel_dis += (time.time()-self.prev_time) * (self.status.speed/36)
                self.prev_time = time.time()

                if(self.travel_dis >= 1):
                    self.travel_dis = 0
                    self.log_insang.append([self.ERP42_pose.x, self.ERP42_pose.y])


                if(self.count == -1):
                    plt.plot(self.new_CSV['X'],self.new_CSV['Y'], color='blue', linestyle='solid', linewidth=1,markersize=5)
                    plt.plot([self.ERP42_pose.x], [self.ERP42_pose.y],  color='red', marker='*', markersize=20)
                    plt.xlim([min(self.new_CSV['X'])-20, max(self.new_CSV['X'])+20]) # -5,+5
                    plt.ylim([min(self.new_CSV['Y'])-20, max(self.new_CSV['Y'])+20]) # -5, +5
                    # plt.show(block=False)
                    # plt.pause(0.1)
                    # plt.clf()

            rate.sleep()


if __name__ == '__main__':
    try:
        gps = GPS()

    except rospy.ROSInterruptException:
        pass
