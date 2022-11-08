#! /usr/bin/env python

import ErpSerialHandler
import rospy

from erp_driver.msg import erpStatusMsg, erpCmdMsg
            
class s_drive():
    def __init__(self):
        self.
        self.cmd_pub = rospy.Publisher('/Car_Control_cmd/Speed_wp_Int16', erpCmdMsg, queue_size=1)
        self.cmd = erpCmdMsg()
        self.cmd.speed = 0
        self.steering_cmd = [-2000, 0]
        cmd_cnts = 50
        self.turn = 0
        self.steer_()
        
    # def steer_(self):
    #     print(11111111111111111111)
    #     # for i in range(2):
    #     #     self.turn = i
    #     if self.turn == 1:
    #         self.cmd.steer= self.steering_cmd[self.turn]
    #         self.turn = 0
    #         rospy.loginfo(self.cmd)
    #     else:
    #         self.cmd.steer= self.steering_cmd[self.turn]
    #         self.turn = 1
    #         rospy.loginfo(self.cmd)            
    #     self.cmd_pub.publish(self.cmd)
        
        
    def steer_(self):
        rospy.init_node('s_drive', anonymous=True)
        # for i in range(2):
        #     self.turn = i
        if self.turn == 1:
            self.cmd.steer= self.steering_cmd[self.turn]
            self.turn = 0
            rospy.loginfo(self.cmd)
            # self.cmd_pub.publish(self.cmd)
            
        else:
            self.cmd.steer= self.steering_cmd[self.turn]
            self.turn = 1
            rospy.loginfo(self.cmd)            
            # self.cmd_pub.publish(self.cmd)
            
        
        # while not rospy.is_shutdown():
        #     for i in range(2):
        #         cmd.steer= steering_cmd[i]
        #         rospy.loginfo(cmd)
                
        #         for _ in range(cmd_cnts):
        #             cmd_pub.publish(cmd)
        #             rate.sleep()

        #rospy.spin()

if __name__ == '__main__':
    try:
        cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=1)
        rate = rospy.Rate(30)
        s_d = s_drive()
        cmd = erpCmdMsg()
        while not rospy.is_shutdown():
            s_d.steer_()
            cmd_pub.publish(cmd)
            rate.sleep()
   
    except rospy.ROSInterruptException:
        pass
    
    # try:
    #     s_d = s_drive()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass


# output result   
# control_mode: 0
# e_stop: True
# gear: 1
# speed: 0
# steer: 3
# brake: 200
# encoder: 0
# alive: 28
        