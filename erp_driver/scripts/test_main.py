#! /usr/bin/env python

import ErpSerialHandler
import rospy
import time
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
from erp_driver.msg import erpStatusMsg, erpCmdMsg
from geometry_msgs.msg import Pose2D, PoseStamped, QuaternionStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16, Float32


class main_node():
    def __init__(self):
        rospy.init_node('Main_node', anonymous=True)
        self.darknet = rospy.Subscriber("/darknet_result", Int16, self.darknetCallback)
        self.steerangle = rospy.Subscriber("/SteerAngle_Int16", Int16, self.SteerAngle_vision_Callback)
        self.sub_Status = rospy.Subscriber("/erp42_status", erpStatusMsg, self.Status_Callback)
        self.sub_avoidance = rospy.Subscriber("/obstacle_avoidance",Int16, self.avoidance_Callback)
        self.sub1 = rospy.Subscriber("/steering", erpCmdMsg, self.SteerAngle_GPS_Callback)
        self.sub2 = rospy.Subscriber("/speed", Int16, self.Speed_GPS_callback)
        # self.sub3 = rospy.Subscriber("/mode", Int16, self.mode_callback)
        self.lidar_sub = rospy.Subscriber("/ob_mode",Int16, self.lidar_callback)
        self.lidar_range_sub = rospy.Subscriber("/avg_range", Int16, self.lidar_range_callback)
        self.lidar_avoidance_L = rospy.Subscriber("/stay_left", Int16, self.lidar_avoidance_L_callback)
        self.lidar_avoidance_R = rospy.Subscriber("/stay_right", Int16, self.lidar_avoidance_R_callback)
        #self.lidar = rospy.Subscriber("/lidar_offset", erpCmdMsg, self.objdetectCallback)
        #self.speed_wp = rospy.Subscriber("/Car_Control_cmd/Speed_wp_Int16", erpCmdMsg, self.Speed_WP_Callback)
        #self.stop_line = rospy.Subscriber("/stop_line_flag", erpCmdMsg, self.stopline_Callback)

        rospy.on_shutdown(self.kill_function)

        self.debug_lane_tracking = False
        self.debug_yolo = True
        self.debug_obstacle_avoidance = False
        self.debug_pure_pursuit = False

        self.speed_log = []
        self.accel_log = []
        self.brake_log = []
        self.base_speed_log = []

        self.brake_gain = 0.5
        self.gear = 1




        self.base_speed = 50
        self.lidar_data = 0
        self.avoidance = 0
        self.avoidance_L = 0
        self.avoidance_R = 0

        self.lidar_range = 0
        self.mode = 1
        self.darknet_flag = 0

        self.vision_steer = 0
        self.gps_steer = 0
        self.gps_speed = 0

        # Status Information
        self.status = erpStatusMsg()
        self.status.speed = 0
        self.status.gear = 0
        self.status.brake = 0
        self.status.steer = 0

        # Subscriber 1 is message file input
        self.cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=1)
        self.cmd = erpCmdMsg()
        self.cmd.e_stop = False #0x00
        self.cmd.gear = 1
        self.cmd.speed = 0
        self.cmd.steer = 0
        self.cmd.brake = 1

        self.goal = erpCmdMsg()
        self.goal.speed = 0
        self.goal.steer = 0
        # self.steer_wp = Float32()
        # self.speed_wp = Int16()
        self.loop()


    def darknetCallback(self, msg):
        self.darknet_flag  = msg.data
        rospy.loginfo(self.darknet_flag)

    def SteerAngle_vision_Callback(self, msg):
        self.vision_steer = msg.data * 10
        #rospy.loginfo(self.goal.steer)

    def objdetectCallback(self, msg):
        obj_detect = msg.data

    def Speed_WP_Callback(self, msg):
        speed_wp = msg.data

    def stopline_Callback(self, msg):
        stopline_flag = msg.data

    def Status_Callback(self, msg):
        if(msg.gear == 2):
            self.status.speed = -msg.speed
        else:
            self.status.speed = msg.speed

    def avoidance_Callback(self, msg):
        self.avoidance = msg.data
        rospy.loginfo(self.avoidance)
    # Steering Callback
    def SteerAngle_GPS_Callback(self, msg):
        # print(type(msg))
        self.gps_steer = -msg.steer * 71
        # rospy.loginfo("Goal Steer : %d", self.cmd.steer)


    def Speed_GPS_callback(self, msg):
        self.gps_speed = msg.data


        # rospy.loginfo("Speed : %d", self.cmd.speed)

    # def mode_callback(self, msg):
    #     self.mode = msg.data
    #     rospy.loginfo(self.mode)


    def lidar_callback(self, msg):
        self.lidar_data = msg.data
        #rospy.loginfo(self.lidar_data)

    def lidar_range_callback(self, msg):
        self.lidar_range = msg.data

    def lidar_avoidance_L_callback(self, msg):
        self.avoidance_L =msg.data

    def lidar_avoidance_R_callback(self, msg):
        self.avoidance_R =msg.data

    def kill_function(self):
        self.cmd.gear = 1
        self.cmd.brake = 1
        self.cmd.steer = 0
        self.cmd.speed = 0
        self.cmd.e_stop = False
        self.cmd_pub.publish(self.cmd)

        fig = plt.figure(figsize=(20,10))
        plt.ylim([0,200])
        # plt.axhline(self.base_speed, 0, len(self.speed_log), color="black", linestyle='--', linewidth=1)
        plt.plot(self.base_speed_log, color="black", linestyle="solid", linewidth=1)
        plt.plot(self.speed_log, color="green", linestyle="solid", linewidth=2)
        plt.plot(self.accel_log, color="red", linestyle="solid", linewidth=1)
        plt.plot(self.brake_log, color="blue", linestyle="solid", linewidth=1)
        self.speed_log = []  # reset log list for kill node(keyboard interrupt)
        self.accel_log = []
        self.brake_log = []
        plt.show()


    def sign_discrimination(self, num1, num2):
        if(num1 == 0):
            num1 = 1
        if(num2 == 0):
            num2 = 1
        return num1/abs(num1) != num2/abs(num2)


    def speed_control(self):
        if(self.cmd.brake > 1 and self.cmd.brake <= 33):
            speed = 0
        else:
            speed = abs(self.goal.speed + (self.goal.speed-self.status.speed)*1.5)
            speed = np.clip(speed, 0, 200)

        return int(speed)


    def gear_control(self):
        if(self.status.speed == 0):
            if(self.goal.speed > 0):
                self.gear = 0
            elif(self.goal.speed == 0):
                self.gear = 1
            elif(self.goal.speed < 0):
                self.gear = 2
        else:
            self.gear = self.gear


        return int(self.gear)


    def brake_control(self):
        if(self.goal.speed == 0 and self.status.speed == 0):
            brake = 33
        elif(self.sign_discrimination(self.status.speed, self.goal.speed)):
            brake = abs(self.status.speed) * self.brake_gain
        else:
            if(self.goal.speed >= 0):
                error = self.goal.speed - self.status.speed
            elif(self.goal.speed < 0):
                error = self.status.speed - self.goal.speed

            if(error < 0):
                brake = (abs(error)-5) * self.brake_gain
            elif(error >= 0):
                brake = 1

        brake = np.clip(brake, 1, 33)

        return int(brake)


    def steer_control(self):
        steer = np.clip(self.goal.steer, -2000, 2000)

        return int(steer)


    def loop(self):
        rate = rospy.Rate(100)
        start_time = time.time()
        while not rospy.is_shutdown():
            # for scenario test-----------------------
            # if(self.mode == 0):
            #     self.goal.speed = self.base_speed
            #     self.goal.steer  = self.vision_steer
            # elif(self.mode == 1):
            #     self.goal.speed = self.gps_speed
            #     self.goal.steer = self.gps_steer
            #     if(self.darknet_flag == 7):
            #         self.goal.speed = 30
            #     elif(self.darknet_flag == 5):
            #         self.goal.speed = 50

            # elif(self.mode == 2):
            #     self.goal.steer = self.gps_steer
            #     if(self.lidar_data == 0):
            #         self.goal.speed = self.gps_speed
            #         self.cmd.e_stop = False
            #         #self.goal.steer = 0
            #     elif(self.lidar_data == 1):
            #         if(self.avoidance == 1):
            #             if(self.avoidance_L == 0):
            #                 self.goal.steer = -2000
            #             elif(self.avoidance_L == 1):
            #                 self.goal.steer = 0
            #         elif(self.avoidance == 2):
            #             if(self.avoidance_R == 0):
            #                 self.goal.steer = 2000
            #             elif(self.avoidance_R == 1):
            #                 self.goal.steer = 0
            #         self.goal.speed = 50
            #         self.cmd.e_stop = False
            #     elif(self.lidar_data == 2 or self.lidar_data == 3):
            #         self.goal.speed = 0
            #         if(self.lidar_range <= 200 and self.lidar_range >= 50):
            #             self.cmd.e_stop = True
            #         elif(self.lidar_range  > 200 and self.lidar_range < 50):
            #             self.cmd.e_stop = False


            # for debug-------------------------
            if(self.debug_lane_tracking):
                self.goal.speed = self.base_speed
                self.goal.steer  = self.vision_steer

            elif(self.debug_yolo):
                self.goal.speed = self.base_speed
                self.goal.steer = 0
                if(self.darknet_flag == 7):
                    self.goal.speed = 30
                elif(self.darknet_flag == 5):
                    self.goal.speed = 50

            elif(self.debug_obstacle_avoidance):
                self.goal.steer = 0
                if(self.lidar_data == 0):
                    self.goal.speed = self.base_speed
                    self.cmd.e_stop = False
                    #self.goal.steer = 0
                elif(self.lidar_data == 1):
                    if(self.avoidance == 1):
                        if(self.avoidance_L == 0):
                            self.goal.steer = -2000
                        elif(self.avoidance_L == 1):
                            self.goal.steer = 0
                    elif(self.avoidance == 2):
                        if(self.avoidance_R == 0):
                            self.goal.steer = 2000
                        elif(self.avoidance_R == 1):
                            self.goal.steer = 0
                    self.goal.speed = 50
                    self.cmd.e_stop = False
                elif(self.lidar_data == 2 or self.lidar_data == 3):
                    self.goal.speed = 0
                    if(self.lidar_range <= 200 and self.lidar_range >= 50):
                        self.cmd.e_stop = True
                    elif(self.lidar_range  > 200 and self.lidar_range < 50):
                        self.cmd.e_stop = False

            elif(self.debug_pure_pursuit):
                self.goal.speed = self.gps_speed
                self.goal.steer = self.gps_steer

            rospy.loginfo(self.goal.speed)
            if(time.time()-start_time < 30): #and time.time()-start_time >= 30):
                self.base_speed = 70
            elif(time.time()-start_time >= 30 and time.time()-start_time < 60):
                self.base_speed = 50
            elif(time.time()-start_time >= 60):
                self.base_speed = 0



            # rospy.loginfo(self.goal.steer)
            self.cmd.gear = self.gear_control()
            self.cmd.brake = self.brake_control()
            self.cmd.steer = self.steer_control()
            self.cmd.speed = self.speed_control()

            if(self.base_speed == 0):
                self.cmd.speed = 0

            self.cmd_pub.publish(self.cmd)



            # log data
            if int((time.time() - start_time)*100) % 5 == 0: # 2Hz sampling
                self.speed_log.append(self.status.speed)
                self.accel_log.append(self.cmd.speed)
                self.brake_log.append(self.cmd.brake)
                self.base_speed_log.append(self.base_speed)

            rate.sleep()


if __name__ == '__main__':
    try:
      main_node()
    except rospy.ROSInterruptException:
      pass
