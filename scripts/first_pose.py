#!/usr/bin/env python3

import rospy
import math as m
from std_msgs.msg import Float64 
from gazebo_msgs.msg import LinkStates
#リンク長 mm
L0 = 79.929
L1 = 50.929
L2 = 66.929
L3 = 99.862
 
def get_hball_2_link_position():
    # /gazebo/link_statesトピックから1回だけデータを取得
    data = rospy.wait_for_message("/gazebo/link_states", LinkStates)
    
    # hball_2_linkのインデックスを取得
    index = data.name.index('myrobot::hball_2_link')
    
    # 位置を取得
    position = data.pose[index].position
    
    # 位置情報を返す
    return position
    
def inv_kin(x,y,z):
    t1 = m.atan(-x/(y-L0))
    
    t2_3_a = m.asin(  ( -x*x/(m.sin(t1))**2 -L1**2 +L2**2 -L3**2 -2*x*L1/m.sin(t1)  -z**2  )  /  ( 2*L3*m.sqrt( (x/m.sin(t1) + L1)**2 + z **2)     )     )
    
    #t2_3_a2 = m.asin(  ( x*x/(m.sin(t1))**2 +L1**2 -L2**2 +L3**2 +2*x*L1/m.sin(t1)  +z**2  )  /  ( 2*L3*m.sqrt( (x/m.sin(t1) + L1)**2 + z **2)     )     )
    
    t_a = m.asin( -(x/m.sin(t1) + L1)/ m.sqrt( (x/m.sin(t1) + L1)**2 + z **2) )
    
    t2_3 = t2_3_a - t_a
    print("t2_3:",t2_3)
    
    t2 =  m.asin((z -L3*m.sin(t2_3)) / L2) 
    
    t23 = m.asin((z-L2*m.sin(t2))/L3)
    
    t3 = t23 - t2

def talker():
    # 2つのトピックにパブリッシャを定義
    pub1 = rospy.Publisher('/myrobot/leg1_A_joint_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/myrobot/leg2_A_joint_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/myrobot/leg3_A_joint_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/myrobot/leg4_A_joint_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/myrobot/leg5_A_joint_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/myrobot/leg6_A_joint_position_controller/command', Float64, queue_size=10)

    pub7 = rospy.Publisher('/myrobot/leg1_B_joint_position_controller/command', Float64, queue_size=10)
    pub8 = rospy.Publisher('/myrobot/leg2_B_joint_position_controller/command', Float64, queue_size=10)
    pub9 = rospy.Publisher('/myrobot/leg3_B_joint_position_controller/command', Float64, queue_size=10)
    pub10 = rospy.Publisher('/myrobot/leg4_B_joint_position_controller/command', Float64, queue_size=10)
    pub11 = rospy.Publisher('/myrobot/leg5_B_joint_position_controller/command', Float64, queue_size=10)
    pub12 = rospy.Publisher('/myrobot/leg6_B_joint_position_controller/command', Float64, queue_size=10)
    
    pub13 = rospy.Publisher('/myrobot/leg1_C_joint_position_controller/command', Float64, queue_size=10)
    pub14 = rospy.Publisher('/myrobot/leg2_C_joint_position_controller/command', Float64, queue_size=10)
    pub15 = rospy.Publisher('/myrobot/leg3_C_joint_position_controller/command', Float64, queue_size=10)
    pub16 = rospy.Publisher('/myrobot/leg4_C_joint_position_controller/command', Float64, queue_size=10)
    pub17 = rospy.Publisher('/myrobot/leg5_C_joint_position_controller/command', Float64, queue_size=10)
    pub18 = rospy.Publisher('/myrobot/leg6_C_joint_position_controller/command', Float64, queue_size=10)
    # ノードの初期化
    rospy.init_node('multi_topic_talker', anonymous=True)

    rate = rospy.Rate(5)  # 10Hzでメッセージを送る
    
    x,y,z = 
    message1=0
    message2=0

    while not rospy.is_shutdown():
        message1 = min(message1 + 0.05, 0)
        message2 = max(message2 - 0.05, 0)
        
        
        
                      
        pub1.publish(message1)
        pub2.publish(message1)
        pub3.publish(message1)
        pub4.publish(message1)
        pub5.publish(message1)
        pub6.publish(message1)
        pub7.publish(message2)
        pub8.publish(message2)
        pub9.publish(message2)
        pub10.publish(message2)
        pub11.publish(message2)
        pub12.publish(message2)
        pub13.publish(message2)
        pub14.publish(message2)
        pub15.publish(message2)
        pub16.publish(message2)  
        pub17.publish(message2)
        pub18.publish(message2)      
       
        rospy.loginfo(message2)
          
        rate.sleep()
           
        if message1 == 0 and message2 == 0:
            break

if __name__ == '__main__':
    try:
        rospy.init_node('link_state_listener', anonymous=True)
        position = get_hball_2_link_position()
        inv_kin(position.x-10,position.y,position.z)
        talker()

    except rospy.ROSInterruptException:
        pass

