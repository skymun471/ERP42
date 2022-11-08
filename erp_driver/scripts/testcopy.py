#! /usr/bin/env python

import ErpSerialHandler
import rospy
from std_msgs.msg import Int16
from erp_driver.msg import erpStatusMsg, erpCmdMsg

class s_drive():
    def __init__(self):
        rospy.init_node('Straight', anonymous=True)
        cmd_pub = rospy.Publisher('/speed', Int16, queue_size=1)
        rate = rospy.Rate(400)
        cmd = Int16()
        cmd = 50
        #steering_cmd = [0, 0]
        cmd_cnts = 50

        while not rospy.is_shutdown():
            #for i in range(2):
                #cmd.steer = steering_cmd[i]
            rospy.loginfo(cmd)
                #for _ in range(cmd_cnts):
            cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        s_d = s_drive()

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

