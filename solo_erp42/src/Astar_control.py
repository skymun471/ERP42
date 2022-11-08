import pandas as pd
import math
import rospy
from geometry_msgs import Pose2D, PoseStamped
from std_msgs import Int16

my_pose = PoseStamped()
goal_pose = Pose2D()
waypoint = 0

def WPcallback(data):
    waypoint = data.data
    print('wpcallback working!!\n')

def Posecallback(data):
    my_pose.x = float(data.pose.position.x)
    my_pose.y = float(data.pose.position.y)

rospy.init_node('Astar_control')
pub = rospy.Publisher('/goal_pose',Pose2D,queue_size=1)

rospy.Subscriber('/wp_go',Int16,WPcallback)
rospy.Subscriber('/utm', PoseStamped, Posecallback)

r = rospy.rate(3)

# 한번만 실행
read_path = pd.read_csv('./path.csv', keep_default_na=False)
car = [my_pose.x, my_pose.y]
path = []
dist = []
for i in range(len(read_path['X'])):
    path.append([read_path['X'][i],read_path['Y'][i]])
    dist.append(math.sqrt(( car[0] - read_path['X'][i] ) ** 2 + ( car[1] - read_path['Y'][i] ) ** 2 ))

min_dist = dist.index(min(dist))

new_path = []

for i in range(min_dist, len(read_path['X'])):
    new_path.append([read_path['X'][i],read_path['Y'][i]])

new_CSV = pd.DataFrame(new_path, columns=['X','Y'])
new_CSV.to_csv('path.csv',index=None)

while not rospy.is_shutdown():
    goal_pose.x = new_path[waypoint][0]
    goal_pose.y = new_path[waypoint][1]
    pub.publish(goal_pose)