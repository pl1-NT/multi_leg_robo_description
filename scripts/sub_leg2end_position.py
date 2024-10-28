#!/usr/bin/env python3

import rospy
from wheel_robo_description.srv import leg2end_position
from std_msgs.msg import Float64 

p = 0 
q =-0.5 
r =1.9
con = 0

def get_position_client():
    # ノード初期化
    rospy.init_node('hball_2_link_service_client', anonymous=True)
    
    # サービスが利用可能になるまで待機
    rospy.wait_for_service('get_leg2_end_position')
    
    try:
        # サービスプロキシの作成
        get_position = rospy.ServiceProxy('get_leg2_end_position', leg2end_position)
        
        # サービス呼び出し
        response = get_position()
        #rospy.loginfo("hball_2_link Position: x = {}, y = {}, z = {}".format(response.x, response.y, response.z))
        print(response.x,response.y,response.z,response.p,response.q,response.r)
        
        pub1 = rospy.Publisher('/myrobot/leg2_A_joint_position_controller/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/myrobot/leg2_B_joint_position_controller/command', Float64, queue_size=10)
        pub3 = rospy.Publisher('/myrobot/leg2_C_joint_position_controller/command', Float64, queue_size=10)
        pub4 = rospy.Publisher('/myrobot/leg3_A_joint_position_controller/command', Float64, queue_size=10)
        pub5 = rospy.Publisher('/myrobot/leg3_B_joint_position_controller/command', Float64, queue_size=10)
        pub6 = rospy.Publisher('/myrobot/leg3_C_joint_position_controller/command', Float64, queue_size=10)
        pub7 = rospy.Publisher('/myrobot/leg5_A_joint_position_controller/command', Float64, queue_size=10)
        pub8 = rospy.Publisher('/myrobot/leg5_B_joint_position_controller/command', Float64, queue_size=10)
        pub9 = rospy.Publisher('/myrobot/leg5_C_joint_position_controller/command', Float64, queue_size=10)

        pub10 = rospy.Publisher('/myrobot/leg1_A_joint_position_controller/command', Float64, queue_size=10)
        pub11 = rospy.Publisher('/myrobot/leg1_B_joint_position_controller/command', Float64, queue_size=10)
        pub12 = rospy.Publisher('/myrobot/leg1_C_joint_position_controller/command', Float64, queue_size=10)
        pub13 = rospy.Publisher('/myrobot/leg4_A_joint_position_controller/command', Float64, queue_size=10)
        pub14 = rospy.Publisher('/myrobot/leg4_B_joint_position_controller/command', Float64, queue_size=10)
        pub15 = rospy.Publisher('/myrobot/leg4_C_joint_position_controller/command', Float64, queue_size=10)
        pub16 = rospy.Publisher('/myrobot/leg6_A_joint_position_controller/command', Float64, queue_size=10)
        pub17 = rospy.Publisher('/myrobot/leg6_B_joint_position_controller/command', Float64, queue_size=10)
        pub18 = rospy.Publisher('/myrobot/leg6_C_joint_position_controller/command', Float64, queue_size=10)        
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            global p,q,r,con
            if con == 0:
                q = min(q -0.005, -0.7)
                pub2.publish(q)  
                pub5.publish(q)
                pub8.publish(q)
                rate.sleep()
                print(q)
                
            if q<= -0.7: 
               con = 1    
                      
            if con == 1:
                p = max(p - 0.01, response.p)#小さくても5
               
                pub1.publish(p) 
                pub10.publish(p) 
                pub4.publish(-p)
                pub13.publish(-p) 
                pub7.publish(-p)
                pub16.publish(-p) 
                
                if p <= response.p:
                    q = min(q + 0.03, response.q)
                    r = max(r - 0.01, response.r)
                    print(q,r)
                    pub2.publish(q)  
                    pub5.publish(q)
                    pub8.publish(q)
                    pub3.publish(r)  
                    pub6.publish(r)
                    pub9.publish(r)

                rate.sleep()

        
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)

if __name__ == '__main__':
    get_position_client()

