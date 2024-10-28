#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates

p=1

def callback(data):
    # hball_2_linkのインデックスを取得
    index = data.name.index('myrobot::base_footprint')
    
    # 位置を取得
    position = data.pose[index].position
    
    # 位置情報を表示
    rospy.loginfo("hball_2_link Position: x = {}, y = {}, z = {}".format(position.x, position.y, position.z))
    
def listener():
    rospy.init_node('link_state_listener', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback) 
    rospy.spin()

if __name__ == '__main__':
    listener()

