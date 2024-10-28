#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates

def get_hball_2_link_position():
    # /gazebo/link_statesトピックから1回だけデータを取得
    data = rospy.wait_for_message("/gazebo/link_states", LinkStates)
    
    # hball_2_linkのインデックスを取得
    index = data.name.index('myrobot::hball_2_link')
    
    # 位置を取得
    position = data.pose[index].position
    
    # 位置情報を返す
    return position

if __name__ == '__main__':
    rospy.init_node('link_state_listener', anonymous=True)
    
    # hball_2_linkの位置を取得して表示
    position = get_hball_2_link_position()
    rospy.loginfo("hball_2_link Position: x = {}, y = {}, z = {}".format(position.x, position.y, position.z))

