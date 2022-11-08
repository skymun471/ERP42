#!/usr/bin/env python
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from IPython import display
import math
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Int16

my_pose = PoseStamped()
goal_pose = Pose2D()
waypoint = 0
my_pose_x, my_pose_y = 0,0 # 302436.526494736,4123662.052556
new_path = [[0,0]]
count = 0
flag = 0

callback_count = 0
gps_list_x = []
gps_list_y = []

def WPcallback(data):
    global waypoint
    waypoint = data.data


def Posecallback(data):
    global my_pose_x, my_pose_y, count, callback_count, gps_list_y, gps_list_x
    my_pose_x = float(data.pose.position.x)
    my_pose_y = float(data.pose.position.y)
    # my_pose_x = 0
    # my_pose_y = 0

    count -= 1
    callback_count += 1
    if(callback_count % 10 == 0):
        gps_list_x.append(my_pose_x)
        gps_list_y.append(my_pose_y)
        callback_count = 0

def create_path():
    global new_path, new_CSV, flag
    read_path = pd.read_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv', keep_default_na=False)
    car = [my_pose_x, my_pose_y]
    path = []
    dist = []

    print('Creating New Path...')

    for i in range(len(read_path['X'])):
        path.append([read_path['X'][i],read_path['Y'][i]])
        dist.append(math.sqrt(( car[0] - read_path['X'][i] ) ** 2 + ( car[1] - read_path['Y'][i] ) ** 2 ))

    min_dist = dist.index(min(dist))+1

    new_path = []

    for i in range(min_dist, len(read_path['X'])):
        new_path.append([read_path['X'][i],read_path['Y'][i]])

    '''
    new_path.append([315424.61087442876,4071632.882121006])
    new_path.append([315420.38177620026,4071638.8233318003])
    new_path.append([315415.652240725,4071641.8671194073])
    new_path.append([315410.58077947935,4071639.8344328715])
    new_path.append([315406.30837257614,4071635.1093368144])
    new_path.append([315401.2104177064,4071632.67762281])
    new_path.append([315397.5473397362,4071630.114986306])
    new_path.append([315397.25094106304,4071625.5037203204])
    new_path.append([315397.1883400282,4071620.5429258714])
    new_path.append([315393.50630440505,4071617.093079625])
    new_path.append([315387.683115807,4071609.204871401])
    '''

    print('Save New Path...')
    new_CSV = pd.DataFrame(new_path, columns=['X','Y'])
    new_CSV.to_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv',index=None)
    flag = 1

rospy.init_node('Astar_control')
pub = rospy.Publisher('/goal_pose',Pose2D,queue_size=1)

rospy.Subscriber('/wp_go',Int16,WPcallback)
rospy.Subscriber('/utm', PoseStamped, Posecallback)

r = rospy.Rate(200)


while not rospy.is_shutdown():
    if count <= -1 and flag == 0: create_path()
    if flag == 1:
        goal_pose.x = new_path[waypoint][0]
        goal_pose.y = new_path[waypoint][1]
        rospy.loginfo("goal x, y: %.3f, %.3f", goal_pose.x, goal_pose.y)
        rospy.loginfo("waypoint : %d", waypoint)
        # rospy.loginfo("dist: %f", math.sqrt(( my_pose_x - new_path[waypoint][0] ) ** 2 + ( my_pose_y - new_path[waypoint][1] ) ** 2 ))
        pub.publish(goal_pose)


        plt.plot(new_CSV['X'],new_CSV['Y'], color='blue', marker='o',linestyle='solid', linewidth=1,markersize=5)
        plt.plot([my_pose_x], [my_pose_y],  color='red', marker='*', markersize=10)
        plt.xlim([min(new_CSV['X'])-20, max(new_CSV['X'])+20]) # -5,+5
        plt.ylim([min(new_CSV['Y'])-20, max(new_CSV['Y'])+20]) # -5, +5
        plt.show(block=False)
        plt.pause(0.1)
        plt.clf()

        '''
        plt.xlim([315386, 315426])
        plt.ylim([4071595,4071643])
        plt.show(block=False)
        plt.pause(0.1)
        plt.clf()
        '''

        # plt.plot(new_CSV['X'],new_CSV['Y'], color='blue', marker='o',linestyle='solid', linewidth=1,markersize=5)
        # plt.plot([my_pose_x], [my_pose_y], '8r')
        # plt.xlim([315359, 315400])
        # plt.ylim([4071610,4071665])
        # plt.set_size_inches(10,8)
        # plt.show(block=False)
        # plt.pause(0.1)
        # plt.clf()

    #r.sleep()

print(np.mean(gps_list_x))
print(np.mean(gps_list_y))
plt.plot(gps_list_x, gps_list_y,color='blue', marker='o')
plt.show()