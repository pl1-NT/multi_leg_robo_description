#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64 # トピックに送るメッセージ型をインポート

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

    rate = rospy.Rate(20)  # 10Hzでメッセージを送る
    
    message1=0
    message2=0
    message3=0

    while not rospy.is_shutdown():
        #message1 = min(message1 + 0.01, 0.01)
        message2 = max(message2 - 0.05, -0.5) #どんなに小さくても0.5
        #message3 = min(message3 + 0.05, +1.9)
        
        
                      
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
        if message2 <= -0.5:
            message3 = min(message3 + 0.05, +1.9)
            pub13.publish(message3)
            pub14.publish(message3)
            pub15.publish(message3)        
            pub16.publish(message3)
            pub17.publish(message3)
            pub18.publish(message3)     
          
            rospy.loginfo(message2)
          
            rate.sleep()
           
            if message3 >= 1.9:
                break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

