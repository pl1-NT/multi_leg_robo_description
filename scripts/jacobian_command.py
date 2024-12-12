#!/usr/bin/env python3

import math
import numpy as np
import sympy as sy
import rospy
import time
from std_msgs.msg import Float64 

def make_jacobian_matrix(l1, t1, l2, t2, l3, t3):

    J = np.array([
[-l1*np.cos(t1) - l2*(np.cos(t1))*np.cos(t2) - l3*(np.cos(t1))*np.cos(t2+t3) ,  l2*(np.sin(t1))*np.sin(t2) + l3*(np.sin(t1))*np.sin(t2+t3),      l3*(np.sin(t1))*np.sin(t2+t3)],
[-l1*np.sin(t1)-(l3*np.sin(t1))*np.cos(t2+t3)-l2*(np.sin(t1))*np.cos(t2)     ,  -l2*(np.cos(t1))*np.sin(t2)-l3*(np.cos(t1))*np.sin(t2+t3),      -l3*(np.cos(t1))*np.sin(t2+t3)],
[0                                                                           ,  l2*np.cos(t2)+l3*np.cos(t2*t3),                                  l3*np.cos(t2+t3)]
])
    return J


def make_inverse_matrix(mat):

    inverse_mat = np.linalg.inv(mat)
    return inverse_mat


# 最大繰り返し回数
max_loop_num = 1000
dist_x=50
dist_z=-15
# 各リンクの長さ

length0 = 80
length1 = 55.8
length2 = 66.9
length3 = 98.8
# 各リンクの初期角度（deg）
t1_deg = 0
t1_2deg = 52.14
t1_3deg = -52.14
t2_deg = -17.19
t3_deg = 97.403
#初期角度をラジアンに
t1_pre = math.radians(t1_deg)
t1_2pre = math.radians(t1_2deg)
t1_3pre = math.radians(t1_3deg)
t2_pre = math.radians(t2_deg)
t3_pre = math.radians(t3_deg)
# 支持目標手先位置 
xe_goal = -(np.sin(t1_pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) -dist_x
ye_goal = length0+np.cos(t1_pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze_goal = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)

xe2_goal = -(np.sin(t1_2pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) +dist_x
ye2_goal = length0+np.cos(t1_2pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze2_goal = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)

xe3_goal = -(np.sin(t1_3pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) +dist_x
ye3_goal = length0+np.cos(t1_3pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze3_goal = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)
#2
P_goal = np.array([[xe_goal],
                   [ye_goal],
                   [ze_goal]
                   ]) 
#3
P2_goal = np.array([[xe2_goal],
                   [ye2_goal],
                   [ze2_goal]
                   ])  
#5                   
P3_goal = np.array([[xe3_goal],
                   [ye3_goal],
                   [ze3_goal]
                   ])  
print(P3_goal)
# 遊脚目標手先位置 first
#1
xe_goal_s1 = -(np.sin(t1_pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) 
ye_goal_s1 = length0+np.cos(t1_pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze_goal_s1 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z
#6
xe2_goal_s1 = -(np.sin(t1_2pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) 
ye2_goal_s1 = length0+np.cos(t1_2pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze2_goal_s1 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z
#4
xe3_goal_s1 = -(np.sin(t1_3pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ye3_goal_s1 = length0+np.cos(t1_3pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze3_goal_s1 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z

P4_goal_s1 = np.array([[xe_goal_s1],
                   [ye_goal_s1],
                   [ze_goal_s1]
                   ]) 
P5_goal_s1 = np.array([[xe2_goal_s1],
                   [ye2_goal_s1],
                   [ze2_goal_s1]
                   ])  
P6_goal_s1 = np.array([[xe3_goal_s1],
                   [ye3_goal_s1],
                   [ze3_goal_s1]
                   ])  
# 支持脚目標手先位置 second
xe_goal_s2 = -(np.sin(t1_pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) -dist_x
ye_goal_s2 = length0+np.cos(t1_pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze_goal_s2 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z

xe2_goal_s2 = -(np.sin(t1_2pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) +dist_x
ye2_goal_s2 = length0+np.cos(t1_2pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze2_goal_s2 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z

xe3_goal_s2 = -(np.sin(t1_3pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))+dist_x
ye3_goal_s2 = length0+np.cos(t1_3pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze3_goal_s2 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)+dist_z

P4_goal_s2 = np.array([[xe_goal_s2],
                   [ye_goal_s2],
                   [ze_goal_s2]
                   ]) 
P5_goal_s2 = np.array([[xe2_goal_s2],
                   [ye2_goal_s2],
                   [ze2_goal_s2]
                   ])  
P6_goal_s2 = np.array([[xe3_goal_s2],
                   [ye3_goal_s2],
                   [ze3_goal_s2]
                   ])    
# 支持脚目標手先位置 final
xe_goal_s3 = -(np.sin(t1_pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) -dist_x
ye_goal_s3 = length0+np.cos(t1_pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze_goal_s3 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)

xe2_goal_s3 = -(np.sin(t1_2pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre)) +dist_x
ye2_goal_s3 = length0+np.cos(t1_2pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze2_goal_s3 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)-1.5

xe3_goal_s3 = -(np.sin(t1_3pre))*(length1 + length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))+dist_x
ye3_goal_s3 = length0+np.cos(t1_3pre)*(length1+length2*np.cos(t2_pre)+length3*np.cos(t2_pre+t3_pre))
ze3_goal_s3 = length2*np.sin(t2_pre)+length3*np.sin(t2_pre+t3_pre)

P4_goal_s3 = np.array([[xe_goal_s3],
                   [ye_goal_s3],
                   [ze_goal_s3]
                   ]) 
P5_goal_s3 = np.array([[xe2_goal_s3],
                   [ye2_goal_s3],
                   [ze2_goal_s3]
                   ])  
P6_goal_s3 = np.array([[xe3_goal_s3],
                   [ye3_goal_s3],
                   [ze3_goal_s3]
                   ])  
print(P_goal)
print(P2_goal)
print(P3_goal)
print(P4_goal_s3)
print(P5_goal_s3)
print(P6_goal_s3)             
# 手先位置Pを各ループごとにどの程度動かすかを決める定数
P_delta_param = 0.03
P_delta_param_s1 = P_delta_param*1.5
P_delta_param_s2 = P_delta_param*1.5
P_delta_param_s3 = P_delta_param*1.5
# ゴールしたと判定する閾値(P_currentとP_goalの距離がどの程度まで近づいたらゴールとみなすか)
goal_dis = 0.05
i=0
step=1
# ==========================================================================

# （初期）関節角度T_cur
t1_rad = t1_deg * math.pi/180
t1_2rad = t1_2deg * math.pi/180
t1_3rad = t1_3deg * math.pi/180
t2_rad = t2_deg * math.pi/180
t3_rad = t3_deg * math.pi/180
t1_cur = t1_rad
t1_2cur = t1_2rad
t1_3cur = t1_3rad
t2_cur = t2_rad
t3_cur = t3_rad
#2
T_cur = np.array([[t1_cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#3
T2_cur = np.array([[t1_2cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#5
T3_cur = np.array([[t1_3cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#1
T4_cur = np.array([[t1_cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#6
T5_cur = np.array([[t1_2cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#4
T6_cur = np.array([[t1_3cur],
                      [t2_cur],
                      [t3_cur]
                      ])
#初期手先位置P_cur
x0, y0 ,z0 = 0, 0, 0

x0_to_e, y0_to_e, z0_to_e =-length1*np.sin(T_cur[0][0])-length2*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0])-length3*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0]),length0+length1*np.cos(T_cur[0][0])+length2*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0])+length3*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0]),length2*np.sin(T_cur[1][0])+length3*np.sin(T_cur[1][0]+T_cur[2][0])

x0_to_e2, y0_to_e2, z0_to_e2 =-length1*np.sin(T2_cur[0][0])-length2*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0])-length3*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0]),length0+length1*np.cos(T2_cur[0][0])+length2*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0])+length3*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0]),length2*np.sin(T2_cur[1][0])+length3*np.sin(T2_cur[1][0]+T2_cur[2][0])

x0_to_e3, y0_to_e3, z0_to_e3 =-length1*np.sin(T3_cur[0][0])-length2*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0])-length3*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0]),length0+length1*np.cos(T3_cur[0][0])+length2*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0])+length3*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0]),length2*np.sin(T3_cur[1][0])+length3*np.sin(T3_cur[1][0]+T3_cur[2][0])
# link1の根本（原点座標）から見た時の，エンドエフェクタの位置

xe = x0 + x0_to_e
ye = y0 + y0_to_e
ze = z0 + z0_to_e
xe2 = x0 + x0_to_e2
ye2 = y0 + y0_to_e2
ze2 = z0 + z0_to_e2
xe3 = x0 + x0_to_e3
ye3 = y0 + y0_to_e3
ze3 = z0 + z0_to_e3
#2
P_cur = np.array([[xe],
                  [ye],
                  [ze]])
#3
P2_cur = np.array([[xe2],
                  [ye2],
                  [ze2]])
#5
P3_cur = np.array([[xe3],
                  [ye3],
                  [ze3]])                                    
#1
P4_cur = np.array([[xe],
                  [ye],
                  [ze]])
#6
P5_cur = np.array([[xe2],
                  [ye2],
                  [ze2]])
#4
P6_cur = np.array([[xe3],
                  [ye3],
                  [ze3]])  
pub2_A = rospy.Publisher('/myrobot/leg2_A_joint_position_controller/command', Float64, queue_size=10)
pub2_B = rospy.Publisher('/myrobot/leg2_B_joint_position_controller/command', Float64, queue_size=10)
pub2_C = rospy.Publisher('/myrobot/leg2_C_joint_position_controller/command', Float64, queue_size=10)
pub3_A = rospy.Publisher('/myrobot/leg3_A_joint_position_controller/command', Float64, queue_size=10)
pub3_B = rospy.Publisher('/myrobot/leg3_B_joint_position_controller/command', Float64, queue_size=10)
pub3_C = rospy.Publisher('/myrobot/leg3_C_joint_position_controller/command', Float64, queue_size=10)
pub5_A = rospy.Publisher('/myrobot/leg5_A_joint_position_controller/command', Float64, queue_size=10)
pub5_B = rospy.Publisher('/myrobot/leg5_B_joint_position_controller/command', Float64, queue_size=10)
pub5_C = rospy.Publisher('/myrobot/leg5_C_joint_position_controller/command', Float64, queue_size=10)
pub1_A = rospy.Publisher('/myrobot/leg1_A_joint_position_controller/command', Float64, queue_size=10)
pub1_B = rospy.Publisher('/myrobot/leg1_B_joint_position_controller/command', Float64, queue_size=10)
pub1_C = rospy.Publisher('/myrobot/leg1_C_joint_position_controller/command', Float64, queue_size=10)
pub4_A = rospy.Publisher('/myrobot/leg4_A_joint_position_controller/command', Float64, queue_size=10)
pub4_B = rospy.Publisher('/myrobot/leg4_B_joint_position_controller/command', Float64, queue_size=10)
pub4_C = rospy.Publisher('/myrobot/leg4_C_joint_position_controller/command', Float64, queue_size=10)
pub6_A = rospy.Publisher('/myrobot/leg6_A_joint_position_controller/command', Float64, queue_size=10)
pub6_B = rospy.Publisher('/myrobot/leg6_B_joint_position_controller/command', Float64, queue_size=10)
pub6_C = rospy.Publisher('/myrobot/leg6_C_joint_position_controller/command', Float64, queue_size=10)
rospy.init_node('multi_topic_talker', anonymous=True)
rate = rospy.Rate(20)
def commander():
    global i ,max_loop_num, length0, length1, length2, length3, P_goal, P_delta_param,P_delta_param_s1,P_delta_param_s2,P_delta_param_s3, goal_dis, T_cur, P_cur,T2_cur, P2_cur,T3_cur, P3_cur,P2_goal,P3_goal,T4_cur, T5_cur,T6_cur, P4_cur,P5_cur, P6_cur,P2_goal,P3_goal,P4_goal_s1,P5_goal_s1,P6_goal_s1 ,P4_goal_s2,P5_goal_s2,P6_goal_s2 ,P4_goal_s3,P5_goal_s3,P6_goal_s3 ,step 
    while not rospy.is_shutdown():

        jacobian = make_jacobian_matrix(length1, T_cur[0][0], length2, T_cur[1][0], length3, T_cur[2][0]) # ヤコビ行列J
        jacobian2 = make_jacobian_matrix(length1, T2_cur[0][0], length2, T2_cur[1][0], length3, T2_cur[2][0])
        jacobian3 = make_jacobian_matrix(length1, T3_cur[0][0], length2, T3_cur[1][0], length3, T3_cur[2][0]) 
        jacobian4 = make_jacobian_matrix(length1, T4_cur[0][0], length2, T4_cur[1][0], length3, T4_cur[2][0]) 
        jacobian5 = make_jacobian_matrix(length1, T5_cur[0][0], length2, T5_cur[1][0], length3, T5_cur[2][0])
        jacobian6 = make_jacobian_matrix(length1, T6_cur[0][0], length2, T6_cur[1][0], length3, T6_cur[2][0])  
        
        jacobian_inverse = make_inverse_matrix(jacobian) 
        jacobian_inverse2 = make_inverse_matrix(jacobian2)
        jacobian_inverse3 = make_inverse_matrix(jacobian3)
        jacobian_inverse4 = make_inverse_matrix(jacobian4) 
        jacobian_inverse5 = make_inverse_matrix(jacobian5)
        jacobian_inverse6 = make_inverse_matrix(jacobian6)
    # 現在の手先P_current→目標手先P_goal 方向のベクトル
        P_cur_to_P_goal = P_goal - P_cur
        P2_cur_to_P_goal = P2_goal - P2_cur
        P3_cur_to_P_goal = P3_goal - P3_cur

    # 現在の手先P_current→目標手先P_goal 方向に向かう微小量ΔP
        P_delta_param = P_delta_param*1.05
        P_delta = P_cur_to_P_goal * P_delta_param
        P2_delta = P2_cur_to_P_goal * P_delta_param
        P3_delta = P3_cur_to_P_goal * P_delta_param

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
        T_delta = jacobian_inverse @ P_delta
        T2_delta = jacobian_inverse2 @ P2_delta
        T3_delta = jacobian_inverse3 @ P3_delta
        
        if step == 1:
            P4_cur_to_P_goal = P4_goal_s1 - P4_cur
            P5_cur_to_P_goal = P5_goal_s1 - P5_cur
            P6_cur_to_P_goal = P6_goal_s1 - P6_cur
            
            P_delta_param_s1 = P_delta_param_s1*1.5
            P4_delta = P4_cur_to_P_goal * P_delta_param_s1
            P5_delta = P5_cur_to_P_goal * P_delta_param_s1
            P6_delta = P6_cur_to_P_goal * P_delta_param_s1

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
            if  (P6_goal_s1[0][0]-P6_cur[0][0])**2  + (P6_goal_s1[1][0]-P6_cur[1][0])**2  + (P6_goal_s1[2][0]-P6_cur[2][0])**2 <  0.2**2 :
                print("↑first-step")
                step = 2
        if step == 2:
            P4_cur_to_P_goal = P4_goal_s2 - P4_cur
            P5_cur_to_P_goal = P5_goal_s2 - P5_cur
            P6_cur_to_P_goal = P6_goal_s2 - P6_cur
            
            P_delta_param_s2 = P_delta_param_s2*1.3
            P4_delta = P4_cur_to_P_goal * P_delta_param_s2
            P5_delta = P5_cur_to_P_goal * P_delta_param_s2
            P6_delta = P6_cur_to_P_goal * P_delta_param_s2

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
            if  (P6_goal_s2[0][0]-P6_cur[0][0])**2  + (P6_goal_s2[1][0]-P6_cur[1][0])**2  + (P6_goal_s2[2][0]-P6_cur[2][0])**2 <  0.2**2 :
                print("↑second-tstep")
                step = 3       
        if step == 3:
            P4_cur_to_P_goal = P4_goal_s3 - P4_cur
            P5_cur_to_P_goal = P5_goal_s3 - P5_cur
            P6_cur_to_P_goal = P6_goal_s3 - P6_cur
            
            P_delta_param_s3 = P_delta_param_s3*1.12
            P4_delta = P4_cur_to_P_goal * P_delta_param_s3
            P5_delta = P5_cur_to_P_goal * P_delta_param_s3
            P6_delta = P6_cur_to_P_goal * P_delta_param_s3

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
            print("1:",P4_cur[2][0])
            print("4:",P6_cur[2][0])
            print("6:",P5_cur[2][0])
            #print("4:",P3_cur[2][0])
            if  (P6_goal_s3[0][0]-P6_cur[0][0])**2  + (P6_goal_s3[1][0]-P6_cur[1][0])**2  + (P6_goal_s3[2][0]-P6_cur[2][0])**2 <  0.007**2 :
                #print("1: ",P4_cur[2][0])  
                #print("4: ",P6_cur[2][0])  
                #print("6: ",P5_cur[2][0]) 
                #print("3rd-done")
                T4_new = T4_cur
                T5_new = T5_cur 
                T6_new = T6_cur              
    # ΔQだけ各関節を動かす
        T_new = T_cur + T_delta
        T2_new = T2_cur + T2_delta
        T3_new = T3_cur + T3_delta
        t2_1=T_new[0][0]
        t2_2=T_new[1][0]
        t2_3=T_new[2][0]
        t3_1=T2_new[0][0]
        t3_2=T2_new[1][0]
        t3_3=T2_new[2][0]
        t5_1=T3_new[0][0]
        t5_2=T3_new[1][0]
        t5_3=T3_new[2][0]
        t1_1=T4_new[0][0]
        t1_2=T4_new[1][0]
        t1_3=T4_new[2][0]
        t4_1=T6_new[0][0]
        t4_2=T6_new[1][0]
        t4_3=T6_new[2][0]
        t6_1=T5_new[0][0]
        t6_2=T5_new[1][0]
        t6_3=T5_new[2][0]

        pub1_A.publish(t1_1)
        pub4_A.publish(t4_1)
        pub6_A.publish(t6_1)
        pub1_B.publish(t1_2)
        pub4_B.publish(t4_2)
        pub6_B.publish(t6_2)
        pub1_C.publish(t1_3)
        pub4_C.publish(t4_3)
        pub6_C.publish(t6_3)  
        pub2_A.publish(t2_1)
        pub3_A.publish(t3_1)
        pub5_A.publish(t5_1)
        pub2_B.publish(t2_2)
        pub3_B.publish(t3_2)
        pub5_B.publish(t5_2)
        pub2_C.publish(t2_3)
        pub3_C.publish(t3_3)
        pub5_C.publish(t5_3)
        rate.sleep()
        
    # 更新後の関節角度T_newを用いて、手先位置P_newを計算（三角関数による順運動学）
    # link1の根本から見た時の，link1の先端位置
        x0_to_e, y0_to_e, z0_to_e =-length1*np.sin(T_new[0][0])-length2*(np.sin(T_new[0][0]))*np.cos(T_new[1][0])-length3*(np.sin(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length0+length1*np.cos(T_new[0][0])+length2*(np.cos(T_new[0][0]))*np.cos(T_new[1][0])+length3*(np.cos(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length2*np.sin(T_new[1][0])+length3*np.sin(T_new[1][0]+T_new[2][0])

        x0_to_e2, y0_to_e2, z0_to_e2 =-length1*np.sin(T2_new[0][0])-length2*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0])-length3*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length0+length1*np.cos(T2_new[0][0])+length2*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0])+length3*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length2*np.sin(T2_new[1][0])+length3*np.sin(T2_new[1][0]+T2_new[2][0])

        x0_to_e3, y0_to_e3, z0_to_e3 =-length1*np.sin(T3_new[0][0])-length2*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0])-length3*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length0+length1*np.cos(T3_new[0][0])+length2*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0])+length3*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length2*np.sin(T3_new[1][0])+length3*np.sin(T3_new[1][0]+T3_new[2][0])
 
        x0_to_e4, y0_to_e4, z0_to_e4 =-length1*np.sin(T4_new[0][0])-length2*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0])-length3*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length0+length1*np.cos(T4_new[0][0])+length2*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0])+length3*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length2*np.sin(T4_new[1][0])+length3*np.sin(T4_new[1][0]+T4_new[2][0])

        x0_to_e5, y0_to_e5, z0_to_e5 =-length1*np.sin(T5_new[0][0])-length2*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0])-length3*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length0+length1*np.cos(T5_new[0][0])+length2*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0])+length3*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length2*np.sin(T5_new[1][0])+length3*np.sin(T5_new[1][0]+T5_new[2][0])

        x0_to_e6, y0_to_e6, z0_to_e6 =-length1*np.sin(T6_new[0][0])-length2*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0])-length3*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length0+length1*np.cos(T6_new[0][0])+length2*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0])+length3*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length2*np.sin(T6_new[1][0])+length3*np.sin(T6_new[1][0]+T6_new[2][0])
    # link1の根本（原点座標）から見た時の，エンドエフェクタの位置
        xe = x0 + x0_to_e
        ye = y0 + y0_to_e
        ze = z0 + z0_to_e
        xe2 = x0 + x0_to_e2
        ye2 = y0 + y0_to_e2
        ze2 = z0 + z0_to_e2
        xe3 = x0 + x0_to_e3
        ye3 = y0 + y0_to_e3
        ze3 = z0 + z0_to_e3
        xe4 = x0 + x0_to_e4
        ye4 = y0 + y0_to_e4
        ze4 = z0 + z0_to_e4
        xe5 = x0 + x0_to_e5
        ye5 = y0 + y0_to_e5
        ze5 = z0 + z0_to_e5
        xe6 = x0 + x0_to_e6
        ye6 = y0 + y0_to_e6
        ze6 = z0 + z0_to_e6
        P_new = np.array([[xe],
                          [ye],
                          [ze]])
        P2_new = np.array([[xe2],
                           [ye2],
                           [ze2]])
        P3_new = np.array([[xe3],
                           [ye3],
                           [ze3]]) 
        P4_new = np.array([[xe4],
                           [ye4],
                          [ ze4]])
        P5_new = np.array([[xe5],
                           [ye5],
                           [ze5]])
        P6_new = np.array([[xe6],
                           [ye6],
                           [ze6]]) 
    # 現在関節角度、現在手先位置を更新
        T_cur = T_new
        P_cur = P_new
        T2_cur = T2_new
        P2_cur = P2_new
        T3_cur = T3_new
        P3_cur = P3_new
        T4_cur = T4_new
        P4_cur = P4_new
        T5_cur = T5_new
        P5_cur = P5_new
        T6_cur = T6_new
        P6_cur = P6_new
        i=i+1
        #print("5-x:",P3_cur[0][0])
    # 目標手先P_goalに到達したら終了
        if  (P3_goal[0][0]-P3_cur[0][0])**2  + (P3_goal[1][0]-P3_cur[1][0])**2  + (P3_goal[2][0]-P3_cur[2][0])**2 <  0.01**2 :   
            print("2: ",P_cur[2][0])  
            print("3: ",P2_cur[2][0])  
            print("5: ",P3_cur[2][0])           
            break
        
        if i == max_loop_num-1:
            print("位置を計算できませんでした（特異点，もしくは実現不可能な座標の可能性があります）")
            break
    #time.sleep(1.5)
    #支遊入れ替え
    dist_x = 100
    dist_z = -10
    step=1
    P_delta_param = 0.01
    P_delta_param_s1 = P_delta_param*1.5
    P_delta_param_s2 = P_delta_param*1.5
    P_delta_param_s3 = P_delta_param*1.5
    #tn_m = 第n関節m本目
    t1_1pre = T4_cur[0][0]
    t2_1pre = T4_cur[1][0]
    t3_1pre = T4_cur[2][0]
    t1_2pre = T6_cur[0][0]
    t2_2pre = T6_cur[1][0]
    t3_2pre = T6_cur[2][0]
    t1_3pre = T5_cur[0][0]
    t2_3pre = T5_cur[1][0]
    t3_3pre = T5_cur[2][0]
    t1_4pre = T_cur[0][0]
    t2_4pre = T_cur[1][0]
    t3_4pre = T_cur[2][0]
    t1_5pre = T2_cur[0][0]
    t2_5pre = T2_cur[1][0]
    t3_5pre = T2_cur[2][0]
    t1_6pre = T3_cur[0][0]
    t2_6pre = T3_cur[1][0]
    t3_6pre = T3_cur[2][0]    
    T_cur = np.array([[t1_1pre],
                      [t2_1pre],
                      [t3_1pre]
                      ])
#3
    T2_cur = np.array([[t1_2pre],
                      [t2_2pre],
                      [t3_2pre]
                      ])
#5
    T3_cur = np.array([[t1_3pre],
                      [t2_3pre],
                      [t3_3pre]
                      ])
#1
    T4_cur = np.array([[t1_4pre],
                      [t2_4pre],
                      [t3_4pre]
                      ])
#6
    T5_cur = np.array([[t1_5pre],
                      [t2_5pre],
                      [t3_5pre]
                      ])
#4
    T6_cur = np.array([[t1_6pre],
                      [t2_6pre],
                      [t3_6pre]
                      ])
    P_curl = P_cur
    P2_curl = P2_cur
    P3_curl = P3_cur
    P4_curl = P4_cur
    P5_curl = P5_cur
    P6_curl = P6_cur
    P_cur = P4_curl
    P2_cur = P6_curl
    P3_cur = P5_curl
    P4_cur = P_curl
    P5_cur = P2_curl
    P6_cur = P3_curl   
    # 支持目標手先位置 
    xe_goal = -(np.sin(t1_1pre))*(length1 + length2*np.cos(t2_1pre)+length3*np.cos(t2_1pre+t3_1pre)) +dist_x
    ye_goal = length0+np.cos(t1_1pre)*(length1+length2*np.cos(t2_1pre)+length3*np.cos(t2_1pre+t3_1pre))
    ze_goal = length2*np.sin(t2_1pre)+length3*np.sin(t2_1pre+t3_1pre)

    xe2_goal = -(np.sin(t1_2pre))*(length1 + length2*np.cos(t2_2pre)+length3*np.cos(t2_2pre+t3_2pre)) -dist_x
    ye2_goal = length0+np.cos(t1_2pre)*(length1+length2*np.cos(t2_2pre)+length3*np.cos(t2_2pre+t3_2pre))
    ze2_goal = length2*np.sin(t2_2pre)+length3*np.sin(t2_2pre+t3_2pre)

    xe3_goal = -(np.sin(t1_3pre))*(length1 + length2*np.cos(t2_3pre)+length3*np.cos(t2_3pre+t3_3pre)) -dist_x
    ye3_goal = length0+np.cos(t1_3pre)*(length1+length2*np.cos(t2_3pre)+length3*np.cos(t2_3pre+t3_3pre))
    ze3_goal = length2*np.sin(t2_3pre)+length3*np.sin(t2_3pre+t3_3pre)
#1
    P_goal = np.array([[xe_goal],
                   [ye_goal],
                   [ze_goal]
                   ]) 
#4
    P2_goal = np.array([[xe2_goal],
                   [ye2_goal],
                   [ze2_goal]
                   ])  
#6                   
    P3_goal = np.array([[xe3_goal],
                   [ye3_goal],
                   [ze3_goal]
                   ])  
    # 遊脚目標手先位置 first
#2
    xe_goal_s1 = -(np.sin(t1_4pre))*(length1 + length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre)) 
    ye_goal_s1 = length0+np.cos(t1_4pre)*(length1+length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre))
    ze_goal_s1 = length2*np.sin(t2_4pre)+length3*np.sin(t2_4pre+t3_4pre)+dist_z
#3
    xe2_goal_s1 = -(np.sin(t1_5pre))*(length1 + length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre)) 
    ye2_goal_s1 = length0+np.cos(t1_5pre)*(length1+length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre))
    ze2_goal_s1 = length2*np.sin(t2_5pre)+length3*np.sin(t2_5pre+t3_5pre)+dist_z
#5
    xe3_goal_s1 = -(np.sin(t1_6pre))*(length1 + length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))
    ye3_goal_s1 = length0+np.cos(t1_6pre)*(length1+length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))
    ze3_goal_s1 = length2*np.sin(t2_6pre)+length3*np.sin(t2_6pre+t3_6pre)+dist_z

    P4_goal_s1 = np.array([[xe_goal_s1],
                   [ye_goal_s1],
                   [ze_goal_s1]
                   ]) 
    P5_goal_s1 = np.array([[xe2_goal_s1],
                   [ye2_goal_s1],
                   [ze2_goal_s1]
                   ])  
    P6_goal_s1 = np.array([[xe3_goal_s1],
                   [ye3_goal_s1],
                   [ze3_goal_s1]
                   ])  
# 遊脚目標手先位置 second
    xe_goal_s2 = -(np.sin(t1_4pre))*(length1 + length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre)) +dist_x
    ye_goal_s2 = length0+np.cos(t1_4pre)*(length1+length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre))
    ze_goal_s2 = length2*np.sin(t2_4pre)+length3*np.sin(t2_4pre+t3_4pre)+dist_z

    xe2_goal_s2 = -(np.sin(t1_5pre))*(length1 + length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre))-dist_x
    ye2_goal_s2 = length0+np.cos(t1_5pre)*(length1+length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre))
    ze2_goal_s2 = length2*np.sin(t2_5pre)+length3*np.sin(t2_5pre+t3_5pre)+dist_z

    xe3_goal_s2 = -(np.sin(t1_6pre))*(length1 + length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))-dist_x
    ye3_goal_s2 = length0+np.cos(t1_6pre)*(length1+length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))
    ze3_goal_s2 = length2*np.sin(t2_6pre)+length3*np.sin(t2_6pre+t3_6pre)+dist_z

    P4_goal_s2 = np.array([[xe_goal_s2],
                   [ye_goal_s2],
                   [ze_goal_s2]
                   ]) 
    P5_goal_s2 = np.array([[xe2_goal_s2],
                   [ye2_goal_s2],
                   [ze2_goal_s2]
                   ])  
    P6_goal_s2 = np.array([[xe3_goal_s2],
                   [ye3_goal_s2],
                   [ze3_goal_s2]
                   ])    
# 遊脚目標手先位置 final
    xe_goal_s3 = -(np.sin(t1_4pre))*(length1 + length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre)) +dist_x
    ye_goal_s3 = length0+np.cos(t1_4pre)*(length1+length2*np.cos(t2_4pre)+length3*np.cos(t2_4pre+t3_4pre))
    ze_goal_s3 = length2*np.sin(t2_4pre)+length3*np.sin(t2_4pre+t3_4pre)

    xe2_goal_s3 = -(np.sin(t1_5pre))*(length1 + length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre))-dist_x
    ye2_goal_s3 = length0+np.cos(t1_5pre)*(length1+length2*np.cos(t2_5pre)+length3*np.cos(t2_5pre+t3_5pre))
    ze2_goal_s3 = length2*np.sin(t2_5pre)+length3*np.sin(t2_5pre+t3_5pre)

    xe3_goal_s3 = -(np.sin(t1_6pre))*(length1 + length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))-dist_x
    ye3_goal_s3 = length0+np.cos(t1_6pre)*(length1+length2*np.cos(t2_6pre)+length3*np.cos(t2_6pre+t3_6pre))
    ze3_goal_s3 = length2*np.sin(t2_6pre)+length3*np.sin(t2_6pre+t3_6pre)

    P4_goal_s3 = np.array([[xe_goal_s3],
                   [ye_goal_s3],
                   [ze_goal_s3]
                   ]) 
    P5_goal_s3 = np.array([[xe2_goal_s3],
                   [ye2_goal_s3],
                   [ze2_goal_s3]
                   ])  
    P6_goal_s3 = np.array([[xe3_goal_s3],
                   [ye3_goal_s3],
                   [ze3_goal_s3]
                   ])         
    """
    print(P_goal)
    print(P2_goal)
    print(P3_goal)
    print(P4_goal_s3)
    print(P5_goal_s3)
    """
    print("ima",P6_cur)
    print("6goal_s1:",P6_goal_s1 )
    print("6goal_s2:",P6_goal_s2 )
    print("6goal_s3:",P6_goal_s3 )
    while not rospy.is_shutdown():

        jacobian = make_jacobian_matrix(length1, T_cur[0][0], length2, T_cur[1][0], length3, T_cur[2][0]) # ヤコビ行列J
        jacobian2 = make_jacobian_matrix(length1, T2_cur[0][0], length2, T2_cur[1][0], length3, T2_cur[2][0])
        jacobian3 = make_jacobian_matrix(length1, T3_cur[0][0], length2, T3_cur[1][0], length3, T3_cur[2][0]) 
        jacobian4 = make_jacobian_matrix(length1, T4_cur[0][0], length2, T4_cur[1][0], length3, T4_cur[2][0]) 
        jacobian5 = make_jacobian_matrix(length1, T5_cur[0][0], length2, T5_cur[1][0], length3, T5_cur[2][0])
        jacobian6 = make_jacobian_matrix(length1, T6_cur[0][0], length2, T6_cur[1][0], length3, T6_cur[2][0])  
        
        jacobian_inverse = make_inverse_matrix(jacobian) 
        jacobian_inverse2 = make_inverse_matrix(jacobian2)
        jacobian_inverse3 = make_inverse_matrix(jacobian3)
        jacobian_inverse4 = make_inverse_matrix(jacobian4) 
        jacobian_inverse5 = make_inverse_matrix(jacobian5)
        jacobian_inverse6 = make_inverse_matrix(jacobian6)
    # 現在の手先P_current→目標手先P_goal 方向のベクトル
        P_cur_to_P_goal = P_goal - P_cur
        P2_cur_to_P_goal = P2_goal - P2_cur
        P3_cur_to_P_goal = P3_goal - P3_cur

    # 現在の手先P_current→目標手先P_goal 方向に向かう微小量ΔP
        P_delta_param = P_delta_param*1.05
        print("delta",P_delta_param)
        P_delta = P_cur_to_P_goal * P_delta_param
        P2_delta = P2_cur_to_P_goal * P_delta_param
        P3_delta = P3_cur_to_P_goal * P_delta_param

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
        T_delta = jacobian_inverse @ P_delta
        T2_delta = jacobian_inverse2 @ P2_delta
        T3_delta = jacobian_inverse3 @ P3_delta
        
        if step == 1:
            P4_cur_to_P_goal = P4_goal_s1 - P4_cur
            P5_cur_to_P_goal = P5_goal_s1 - P5_cur
            P6_cur_to_P_goal = P6_goal_s1 - P6_cur
            P_delta_param_s1 = P_delta_param_s1*1.12

            P4_delta = P4_cur_to_P_goal * P_delta_param_s1
            P5_delta = P5_cur_to_P_goal * P_delta_param_s1
            P6_delta = P6_cur_to_P_goal * P_delta_param_s1

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta

            if  (P6_goal_s1[0][0]-P6_cur[0][0])**2  + (P6_goal_s1[1][0]-P6_cur[1][0])**2  + (P6_goal_s1[2][0]-P6_cur[2][0])**2 <  0.05**2 :
                print("↑first-step")
                step = 2
        if step == 2:
            P4_cur_to_P_goal = P4_goal_s2 - P4_cur
            P5_cur_to_P_goal = P5_goal_s2 - P5_cur
            P6_cur_to_P_goal = P6_goal_s2 - P6_cur
            P_delta_param_s2 = P_delta_param_s2*1.3
            P4_delta = P4_cur_to_P_goal * P_delta_param_s2
            P5_delta = P5_cur_to_P_goal * P_delta_param_s2
            P6_delta = P6_cur_to_P_goal * P_delta_param_s2

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
            if  (P6_goal_s2[0][0]-P6_cur[0][0])**2  + (P6_goal_s2[1][0]-P6_cur[1][0])**2  + (P6_goal_s2[2][0]-P6_cur[2][0])**2 <  0.1**2 :
                print("↑second-tstep")
                step = 3       
        if step == 3:
            P4_cur_to_P_goal = P4_goal_s3 - P4_cur
            P5_cur_to_P_goal = P5_goal_s3 - P5_cur
            P6_cur_to_P_goal = P6_goal_s3 - P6_cur
            P_delta_param_s3 = P_delta_param_s3*1.12
            P4_delta = P4_cur_to_P_goal * P_delta_param_s3
            P5_delta = P5_cur_to_P_goal * P_delta_param_s3
            P6_delta = P6_cur_to_P_goal * P_delta_param_s3

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta

            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
            if  (P6_goal_s3[0][0]-P6_cur[0][0])**2  + (P6_goal_s3[1][0]-P6_cur[1][0])**2  + (P6_goal_s3[2][0]-P6_cur[2][0])**2 <  0.01**2 :
                print("2: ",P4_cur[2][0])  
                print("3: ",P5_cur[2][0])  
                print("5: ",P6_cur[2][0]) 
                T4_new = T4_cur
                T5_new = T5_cur 
                T6_new = T6_cur              
    # ΔQだけ各関節を動かす
        T_new = T_cur + T_delta
        T2_new = T2_cur + T2_delta
        T3_new = T3_cur + T3_delta
        
        t2_1=T4_new[0][0]
        t2_2=T4_new[1][0]
        t2_3=T4_new[2][0]
        t3_1=T5_new[0][0]
        t3_2=T5_new[1][0]
        t3_3=T5_new[2][0]
        t5_1=T6_new[0][0]
        t5_2=T6_new[1][0]
        t5_3=T6_new[2][0]
        t1_1=T_new[0][0]
        t1_2=T_new[1][0]
        t1_3=T_new[2][0]
        t4_1=T2_new[0][0]
        t4_2=T2_new[1][0]
        t4_3=T2_new[2][0]
        t6_1=T3_new[0][0]
        t6_2=T3_new[1][0]
        t6_3=T3_new[2][0]

        pub1_A.publish(t1_1)
        pub4_A.publish(t4_1)
        pub6_A.publish(t6_1)
        pub1_B.publish(t1_2)
        pub4_B.publish(t4_2)
        pub6_B.publish(t6_2)
        pub1_C.publish(t1_3)
        pub4_C.publish(t4_3)
        pub6_C.publish(t6_3)  
        
        pub2_A.publish(t2_1)
        pub3_A.publish(t3_1)
        pub5_A.publish(t5_1)
        pub2_B.publish(t2_2)
        pub3_B.publish(t3_2)
        pub5_B.publish(t5_2)
        pub2_C.publish(t2_3)
        pub3_C.publish(t3_3)
        pub5_C.publish(t5_3)
        
        rate.sleep()
        
    # 更新後の関節角度T_newを用いて、手先位置P_newを計算（三角関数による順運動学）
    # link1の根本から見た時の，link1の先端位置
        x0_to_e, y0_to_e, z0_to_e =-length1*np.sin(T_new[0][0])-length2*(np.sin(T_new[0][0]))*np.cos(T_new[1][0])-length3*(np.sin(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length0+length1*np.cos(T_new[0][0])+length2*(np.cos(T_new[0][0]))*np.cos(T_new[1][0])+length3*(np.cos(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length2*np.sin(T_new[1][0])+length3*np.sin(T_new[1][0]+T_new[2][0])

        x0_to_e2, y0_to_e2, z0_to_e2 =-length1*np.sin(T2_new[0][0])-length2*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0])-length3*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length0+length1*np.cos(T2_new[0][0])+length2*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0])+length3*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length2*np.sin(T2_new[1][0])+length3*np.sin(T2_new[1][0]+T2_new[2][0])

        x0_to_e3, y0_to_e3, z0_to_e3 =-length1*np.sin(T3_new[0][0])-length2*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0])-length3*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length0+length1*np.cos(T3_new[0][0])+length2*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0])+length3*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length2*np.sin(T3_new[1][0])+length3*np.sin(T3_new[1][0]+T3_new[2][0])
 
        x0_to_e4, y0_to_e4, z0_to_e4 =-length1*np.sin(T4_new[0][0])-length2*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0])-length3*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length0+length1*np.cos(T4_new[0][0])+length2*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0])+length3*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length2*np.sin(T4_new[1][0])+length3*np.sin(T4_new[1][0]+T4_new[2][0])

        x0_to_e5, y0_to_e5, z0_to_e5 =-length1*np.sin(T5_new[0][0])-length2*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0])-length3*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length0+length1*np.cos(T5_new[0][0])+length2*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0])+length3*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length2*np.sin(T5_new[1][0])+length3*np.sin(T5_new[1][0]+T5_new[2][0])

        x0_to_e6, y0_to_e6, z0_to_e6 =-length1*np.sin(T6_new[0][0])-length2*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0])-length3*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length0+length1*np.cos(T6_new[0][0])+length2*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0])+length3*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length2*np.sin(T6_new[1][0])+length3*np.sin(T6_new[1][0]+T6_new[2][0])
    # link1の根本（原点座標）から見た時の，エンドエフェクタの位置
        xe = x0 + x0_to_e
        ye = y0 + y0_to_e
        ze = z0 + z0_to_e
        xe2 = x0 + x0_to_e2
        ye2 = y0 + y0_to_e2
        ze2 = z0 + z0_to_e2
        xe3 = x0 + x0_to_e3
        ye3 = y0 + y0_to_e3
        ze3 = z0 + z0_to_e3
        xe4 = x0 + x0_to_e4
        ye4 = y0 + y0_to_e4
        ze4 = z0 + z0_to_e4
        xe5 = x0 + x0_to_e5
        ye5 = y0 + y0_to_e5
        ze5 = z0 + z0_to_e5
        xe6 = x0 + x0_to_e6
        ye6 = y0 + y0_to_e6
        ze6 = z0 + z0_to_e6
        P_new = np.array([[xe],
                          [ye],
                          [ze]])
        P2_new = np.array([[xe2],
                           [ye2],
                           [ze2]])
        P3_new = np.array([[xe3],
                           [ye3],
                           [ze3]]) 
        P4_new = np.array([[xe4],
                           [ye4],
                          [ ze4]])
        P5_new = np.array([[xe5],
                           [ye5],
                           [ze5]])
        P6_new = np.array([[xe6],
                           [ye6],
                           [ze6]]) 
    # 現在関節角度Q_current、現在手先位置P_currentを更新
        T_cur = T_new
        P_cur = P_new
        T2_cur = T2_new
        P2_cur = P2_new
        T3_cur = T3_new
        P3_cur = P3_new
        T4_cur = T4_new
        P4_cur = P4_new
        T5_cur = T5_new
        P5_cur = P5_new
        T6_cur = T6_new
        P6_cur = P6_new
        i=i+1
    # 目標手先P_goalに到達したら終了
        if  (P3_goal[0][0]-P3_cur[0][0])**2  + (P3_goal[1][0]-P3_cur[1][0])**2  + (P3_goal[2][0]-P3_cur[2][0])**2 <  goal_dis**2 :   
            print("1: ",P_cur[2][0])  
            print("4: ",P2_cur[2][0])  
            print("6: ",P3_cur[2][0])           
            break
        
        if i == max_loop_num-1:
            print("位置を計算できませんでした（特異点，もしくは実現不可能な座標の可能性があります）")
            break

    while True:
        print("3rd-phase")
        dist_x = 100
        dist_z = -10
        step=1
        P_delta_param = 0.01
        P_delta_param_s1 = P_delta_param*1.5
        P_delta_param_s2 = P_delta_param*1.5
        P_delta_param_s3 = P_delta_param*1.5
    #tn_m = 第n関節m本目
  
    # 支持目標手先位置 
        xe_goal =-length1*np.sin(T4_cur[0][0])-length2*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0])-length3*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])-dist_x
        ye_goal = length0+length1*np.cos(T4_cur[0][0])+length2*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0])+length3*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])
        ze_goal = length2*np.sin(T4_cur[1][0])+length3*np.sin(T4_cur[1][0]+T4_cur[2][0])
    
        xe2_goal =-length1*np.sin(T5_cur[0][0])-length2*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0])-length3*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])+dist_x
        ye2_goal = length0+length1*np.cos(T5_cur[0][0])+length2*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0])+length3*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])
        ze2_goal = length2*np.sin(T5_cur[1][0])+length3*np.sin(T5_cur[1][0]+T5_cur[2][0])
    
        xe3_goal =-length1*np.sin(T6_cur[0][0])-length2*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0])-length3*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])+dist_x
        ye3_goal = length0+length1*np.cos(T6_cur[0][0])+length2*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0])+length3*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])
        ze3_goal = length2*np.sin(T6_cur[1][0])+length3*np.sin(T6_cur[1][0]+T6_cur[2][0])
#1
        P4_goal = np.array([[xe_goal],
                   [ye_goal],
                   [ze_goal]
                   ]) 
#4
        P5_goal = np.array([[xe2_goal],
                   [ye2_goal],
                   [ze2_goal]
                   ])  
#6                   
        P6_goal = np.array([[xe3_goal],
                   [ye3_goal],
                   [ze3_goal]
                   ])  
    # 遊脚目標手先位置 first
#2
        xe_goal_s1  =-length1*np.sin(T_cur[0][0])-length2*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0])-length3*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])
        ye_goal_s1  = length0+length1*np.cos(T_cur[0][0])+length2*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0])+length3*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])
        ze_goal_s1  = length2*np.sin(T_cur[1][0])+length3*np.sin(T_cur[1][0]+T_cur[2][0])+dist_z
    
        xe2_goal_s1  =-length1*np.sin(T2_cur[0][0])-length2*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0])-length3*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])
        ye2_goal_s1  = length0+length1*np.cos(T2_cur[0][0])+length2*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0])+length3*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])
        ze2_goal_s1  = length2*np.sin(T2_cur[1][0])+length3*np.sin(T2_cur[1][0]+T2_cur[2][0])+dist_z

        xe3_goal_s1  =-length1*np.sin(T3_cur[0][0])-length2*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0])-length3*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])
        ye3_goal_s1  = length0+length1*np.cos(T3_cur[0][0])+length2*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0])+length3*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])
        ze3_goal_s1  = length2*np.sin(T3_cur[1][0])+length3*np.sin(T3_cur[1][0]+T3_cur[2][0])+dist_z

        P_goal_s1 = np.array([[xe_goal_s1],
                   [ye_goal_s1],
                   [ze_goal_s1]
                   ]) 
        P2_goal_s1 = np.array([[xe2_goal_s1],
                   [ye2_goal_s1],
                   [ze2_goal_s1]
                   ])  
        P3_goal_s1 = np.array([[xe3_goal_s1],
                   [ye3_goal_s1],
                   [ze3_goal_s1]
                   ])  
# 遊脚目標手先位置 second
        xe_goal_s2  =-length1*np.sin(T_cur[0][0])-length2*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0])-length3*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])-dist_x
        ye_goal_s2  = length0+length1*np.cos(T_cur[0][0])+length2*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0])+length3*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])
        ze_goal_s2  = length2*np.sin(T_cur[1][0])+length3*np.sin(T_cur[1][0]+T_cur[2][0])+dist_z
    
        xe2_goal_s2  =-length1*np.sin(T2_cur[0][0])-length2*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0])-length3*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])+dist_x
        ye2_goal_s2  = length0+length1*np.cos(T2_cur[0][0])+length2*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0])+length3*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])
        ze2_goal_s2  = length2*np.sin(T2_cur[1][0])+length3*np.sin(T2_cur[1][0]+T2_cur[2][0])+dist_z

        xe3_goal_s2  =-length1*np.sin(T3_cur[0][0])-length2*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0])-length3*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])+dist_x
        ye3_goal_s2  = length0+length1*np.cos(T3_cur[0][0])+length2*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0])+length3*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])
        ze3_goal_s2  = length2*np.sin(T3_cur[1][0])+length3*np.sin(T3_cur[1][0]+T3_cur[2][0])+dist_z

        P_goal_s2 = np.array([[xe_goal_s2],
                   [ye_goal_s2],
                   [ze_goal_s2]
                   ]) 
        P2_goal_s2 = np.array([[xe2_goal_s2],
                   [ye2_goal_s2],
                   [ze2_goal_s2]
                   ])  
        P3_goal_s2 = np.array([[xe3_goal_s2],
                   [ye3_goal_s2],
                   [ze3_goal_s2]
                   ])    
# 遊脚目標手先位置 final
        xe_goal_s3  =-length1*np.sin(T_cur[0][0])-length2*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0])-length3*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])-dist_x
        ye_goal_s3  = length0+length1*np.cos(T_cur[0][0])+length2*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0])+length3*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])
        ze_goal_s3  = length2*np.sin(T_cur[1][0])+length3*np.sin(T_cur[1][0]+T_cur[2][0])
        
        xe2_goal_s3  =-length1*np.sin(T2_cur[0][0])-length2*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0])-length3*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])+dist_x
        ye2_goal_s3  = length0+length1*np.cos(T2_cur[0][0])+length2*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0])+length3*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])
        ze2_goal_s3  = length2*np.sin(T2_cur[1][0])+length3*np.sin(T2_cur[1][0]+T2_cur[2][0])

        xe3_goal_s3  =-length1*np.sin(T3_cur[0][0])-length2*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0])-length3*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])+dist_x
        ye3_goal_s3  = length0+length1*np.cos(T3_cur[0][0])+length2*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0])+length3*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])
        ze3_goal_s3  = length2*np.sin(T3_cur[1][0])+length3*np.sin(T3_cur[1][0]+T3_cur[2][0])


        P_goal_s3 = np.array([[xe_goal_s3],
                   [ye_goal_s3],
                   [ze_goal_s3]
                   ]) 
        P2_goal_s3 = np.array([[xe2_goal_s3],
                   [ye2_goal_s3],
                   [ze2_goal_s3]
                   ])  
        P3_goal_s3 = np.array([[xe3_goal_s3],
                       [ye3_goal_s3],
                       [ze3_goal_s3]
                       ])            
    
        print("ima",P3_cur)
        print("3goal:",P3_goal_s1 )
        print("3goal_s2:",P3_goal_s2 )
        print("3goal_s3:",P3_goal_s3 )
        while not rospy.is_shutdown():

            jacobian = make_jacobian_matrix(length1, T_cur[0][0], length2, T_cur[1][0], length3, T_cur[2][0]) # ヤコビ行列J
            jacobian2 = make_jacobian_matrix(length1, T2_cur[0][0], length2, T2_cur[1][0], length3, T2_cur[2][0])
            jacobian3 = make_jacobian_matrix(length1, T3_cur[0][0], length2, T3_cur[1][0], length3, T3_cur[2][0]) 
            jacobian4 = make_jacobian_matrix(length1, T4_cur[0][0], length2, T4_cur[1][0], length3, T4_cur[2][0]) 
            jacobian5 = make_jacobian_matrix(length1, T5_cur[0][0], length2, T5_cur[1][0], length3, T5_cur[2][0])
            jacobian6 = make_jacobian_matrix(length1, T6_cur[0][0], length2, T6_cur[1][0], length3, T6_cur[2][0])  
        
            jacobian_inverse = make_inverse_matrix(jacobian) 
            jacobian_inverse2 = make_inverse_matrix(jacobian2)
            jacobian_inverse3 = make_inverse_matrix(jacobian3)
            jacobian_inverse4 = make_inverse_matrix(jacobian4) 
            jacobian_inverse5 = make_inverse_matrix(jacobian5)
            jacobian_inverse6 = make_inverse_matrix(jacobian6)
    # 現在の手先P_current→目標手先P_goal 方向のベクトル
            P4_cur_to_P_goal = P4_goal - P4_cur
            P5_cur_to_P_goal = P5_goal - P5_cur
            P6_cur_to_P_goal = P6_goal - P6_cur

    # 現在の手先P_current→目標手先P_goal 方向に向かう微小量ΔP
            P_delta_param = P_delta_param*1.05
            P4_delta = P4_cur_to_P_goal * P_delta_param
            P5_delta = P5_cur_to_P_goal * P_delta_param
            P6_delta = P6_cur_to_P_goal * P_delta_param

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T4_delta = jacobian_inverse4 @ P4_delta
            T5_delta = jacobian_inverse5 @ P5_delta
            T6_delta = jacobian_inverse6 @ P6_delta
        
            if step == 1:
                P_cur_to_P_goal = P_goal_s1 - P_cur
                P2_cur_to_P_goal = P2_goal_s1 - P2_cur
                P3_cur_to_P_goal = P3_goal_s1 - P3_cur
                P_delta_param_s1 = P_delta_param_s1*1.12

                P_delta = P_cur_to_P_goal * P_delta_param_s1
                P2_delta = P2_cur_to_P_goal * P_delta_param_s1
                P3_delta = P3_cur_to_P_goal * P_delta_param_s1

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T_delta = jacobian_inverse @ P_delta
                T2_delta = jacobian_inverse2 @ P2_delta
                T3_delta = jacobian_inverse3 @ P3_delta

                T_new = T_cur + T_delta
                T2_new = T2_cur + T2_delta
                T3_new = T3_cur + T3_delta

                if  (P3_goal_s1[0][0]-P3_cur[0][0])**2  + (P3_goal_s1[1][0]-P3_cur[1][0])**2  + (P3_goal_s1[2][0]-P3_cur[2][0])**2 <  0.05**2 :
                    print("↑first-step")
                    step = 2
            if step == 2:
                P_cur_to_P_goal = P_goal_s2 - P_cur
                P2_cur_to_P_goal = P2_goal_s2 - P2_cur
                P3_cur_to_P_goal = P3_goal_s2 - P3_cur
                P_delta_param_s2 = P_delta_param_s2*1.12

                P_delta = P_cur_to_P_goal * P_delta_param_s2
                P2_delta = P2_cur_to_P_goal * P_delta_param_s2
                P3_delta = P3_cur_to_P_goal * P_delta_param_s2

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T_delta = jacobian_inverse @ P_delta
                T2_delta = jacobian_inverse2 @ P2_delta
                T3_delta = jacobian_inverse3 @ P3_delta

                T_new = T_cur + T_delta
                T2_new = T2_cur + T2_delta
                T3_new = T3_cur + T3_delta
    
                if  (P3_goal_s2[0][0]-P3_cur[0][0])**2  + (P3_goal_s2[1][0]-P3_cur[1][0])**2  + (P3_goal_s2[2][0]-P3_cur[2][0])**2 <  0.05**2 :
                    print("↑second-step")
                    step = 3  
            if step == 3:
                P_cur_to_P_goal = P_goal_s1 - P_cur
                P2_cur_to_P_goal = P2_goal_s3 - P2_cur
                P3_cur_to_P_goal = P3_goal_s3 - P3_cur
                P_delta_param_s3 = P_delta_param_s3*1.12
        
                P_delta = P_cur_to_P_goal * P_delta_param_s3
                P2_delta = P2_cur_to_P_goal * P_delta_param_s3
                P3_delta = P3_cur_to_P_goal * P_delta_param_s3

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T_delta = jacobian_inverse @ P_delta
                T2_delta = jacobian_inverse2 @ P2_delta
                T3_delta = jacobian_inverse3 @ P3_delta

                T_new = T_cur + T_delta
                T2_new = T2_cur + T2_delta
                T3_new = T3_cur + T3_delta

                if  (P3_goal_s3[0][0]-P3_cur[0][0])**2  + (P3_goal_s3[1][0]-P3_cur[1][0])**2  + (P3_goal_s3[2][0]-P3_cur[2][0])**2 <  0.05**2 :
                #print("1: ",P1_cur[2][0])  
                #print("4: ",P2_cur[2][0])  
                #print("6: ",P3_cur[2][0]) 
                    T_new = T_cur
                    T2_new = T2_cur 
                    T3_new = T3_cur              
        # ΔQだけ各関節を動かす
            T4_new = T4_cur + T4_delta
            T5_new = T5_cur + T5_delta
            T6_new = T6_cur + T6_delta
        
            t2_1=T4_new[0][0]
            t2_2=T4_new[1][0]
            t2_3=T4_new[2][0]
            t3_1=T5_new[0][0]
            t3_2=T5_new[1][0]
            t3_3=T5_new[2][0]
            t5_1=T6_new[0][0]
            t5_2=T6_new[1][0]
            t5_3=T6_new[2][0]
            t1_1=T_new[0][0]
            t1_2=T_new[1][0]
            t1_3=T_new[2][0]
            t4_1=T2_new[0][0]
            t4_2=T2_new[1][0]
            t4_3=T2_new[2][0]
            t6_1=T3_new[0][0]
            t6_2=T3_new[1][0]
            t6_3=T3_new[2][0]

            pub1_A.publish(t1_1)
            pub4_A.publish(t4_1)
            pub6_A.publish(t6_1)
            pub1_B.publish(t1_2)
            pub4_B.publish(t4_2)
            pub6_B.publish(t6_2)
            pub1_C.publish(t1_3)
            pub4_C.publish(t4_3)
            pub6_C.publish(t6_3)  
        
            pub2_A.publish(t2_1)
            pub3_A.publish(t3_1)
            pub5_A.publish(t5_1)
            pub2_B.publish(t2_2)
            pub3_B.publish(t3_2)
            pub5_B.publish(t5_2)
            pub2_C.publish(t2_3)
            pub3_C.publish(t3_3)
            pub5_C.publish(t5_3)
        
            rate.sleep()
        
    # 更新後の関節角度T_newを用いて、手先位置P_newを計算（三角関数による順運動学）
    # link1の根本から見た時の，link1の先端位置
            x0_to_e, y0_to_e, z0_to_e =-length1*np.sin(T_new[0][0])-length2*(np.sin(T_new[0][0]))*np.cos(T_new[1][0])-length3*(np.sin(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length0+length1*np.cos(T_new[0][0])+length2*(np.cos(T_new[0][0]))*np.cos(T_new[1][0])+length3*(np.cos(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length2*np.sin(T_new[1][0])+length3*np.sin(T_new[1][0]+T_new[2][0])

            x0_to_e2, y0_to_e2, z0_to_e2 =-length1*np.sin(T2_new[0][0])-length2*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0])-length3*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length0+length1*np.cos(T2_new[0][0])+length2*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0])+length3*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length2*np.sin(T2_new[1][0])+length3*np.sin(T2_new[1][0]+T2_new[2][0])

            x0_to_e3, y0_to_e3, z0_to_e3 =-length1*np.sin(T3_new[0][0])-length2*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0])-length3*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length0+length1*np.cos(T3_new[0][0])+length2*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0])+length3*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length2*np.sin(T3_new[1][0])+length3*np.sin(T3_new[1][0]+T3_new[2][0])
 
            x0_to_e4, y0_to_e4, z0_to_e4 =-length1*np.sin(T4_new[0][0])-length2*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0])-length3*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length0+length1*np.cos(T4_new[0][0])+length2*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0])+length3*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length2*np.sin(T4_new[1][0])+length3*np.sin(T4_new[1][0]+T4_new[2][0])

            x0_to_e5, y0_to_e5, z0_to_e5 =-length1*np.sin(T5_new[0][0])-length2*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0])-length3*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length0+length1*np.cos(T5_new[0][0])+length2*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0])+length3*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length2*np.sin(T5_new[1][0])+length3*np.sin(T5_new[1][0]+T5_new[2][0])

            x0_to_e6, y0_to_e6, z0_to_e6 =-length1*np.sin(T6_new[0][0])-length2*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0])-length3*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length0+length1*np.cos(T6_new[0][0])+length2*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0])+length3*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length2*np.sin(T6_new[1][0])+length3*np.sin(T6_new[1][0]+T6_new[2][0])
    # link1の根本（原点座標）から見た時の，エンドエフェクタの位置
            xe = x0 + x0_to_e
            ye = y0 + y0_to_e
            ze = z0 + z0_to_e
            xe2 = x0 + x0_to_e2
            ye2 = y0 + y0_to_e2
            ze2 = z0 + z0_to_e2
            xe3 = x0 + x0_to_e3
            ye3 = y0 + y0_to_e3
            ze3 = z0 + z0_to_e3
            xe4 = x0 + x0_to_e4
            ye4 = y0 + y0_to_e4
            ze4 = z0 + z0_to_e4
            xe5 = x0 + x0_to_e5
            ye5 = y0 + y0_to_e5
            ze5 = z0 + z0_to_e5
            xe6 = x0 + x0_to_e6
            ye6 = y0 + y0_to_e6
            ze6 = z0 + z0_to_e6
            P_new = np.array([[xe],
                          [ye],
                          [ze]])
            P2_new = np.array([[xe2],
                           [ye2],
                           [ze2]])
            P3_new = np.array([[xe3],
                           [ye3],
                           [ze3]]) 
            P4_new = np.array([[xe4],
                           [ye4],
                          [ ze4]])
            P5_new = np.array([[xe5],
                           [ye5],
                           [ze5]])
            P6_new = np.array([[xe6],
                           [ye6],
                           [ze6]]) 
    # 現在関節角度Q_current、現在手先位置P_currentを更新
            T_cur = T_new
            P_cur = P_new
            T2_cur = T2_new
            P2_cur = P2_new
            T3_cur = T3_new
            P3_cur = P3_new
            T4_cur = T4_new
            P4_cur = P4_new
            T5_cur = T5_new
            P5_cur = P5_new
            T6_cur = T6_new
            P6_cur = P6_new
            i=i+1
    #     目標手先P_goalに到達したら終了
            if  (P6_goal[0][0]-P6_cur[0][0])**2  + (P6_goal[1][0]-P6_cur[1][0])**2  + (P6_goal[2][0]-P6_cur[2][0])**2 <  goal_dis**2 :   
                print("2: ",P4_cur[2][0])  
                print("3: ",P5_cur[2][0])  
                print("5: ",P6_cur[2][0])           
                break
            
        print("4th-phase")    
        step=1
        P_delta_param = 0.01
        P_delta_param_s1 = P_delta_param*1.5
        P_delta_param_s2 = P_delta_param*1.5
        P_delta_param_s3 = P_delta_param*1.5
    #tn_m = 第n関節m本目
  
    # 支持目標手先位置 
        xe_goal_s1 =-length1*np.sin(T4_cur[0][0])-length2*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0])-length3*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])
        ye_goal_s1 = length0+length1*np.cos(T4_cur[0][0])+length2*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0])+length3*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])
        ze_goal_s1 = length2*np.sin(T4_cur[1][0])+length3*np.sin(T4_cur[1][0]+T4_cur[2][0])+dist_z
    
        xe2_goal_s1 =-length1*np.sin(T5_cur[0][0])-length2*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0])-length3*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])
        ye2_goal_s1 = length0+length1*np.cos(T5_cur[0][0])+length2*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0])+length3*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])
        ze2_goal_s1 = length2*np.sin(T5_cur[1][0])+length3*np.sin(T5_cur[1][0]+T5_cur[2][0])+dist_z

        xe3_goal_s1 =-length1*np.sin(T6_cur[0][0])-length2*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0])-length3*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])
        ye3_goal_s1 = length0+length1*np.cos(T6_cur[0][0])+length2*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0])+length3*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])
        ze3_goal_s1 = length2*np.sin(T6_cur[1][0])+length3*np.sin(T6_cur[1][0]+T6_cur[2][0])+dist_z
#1
        P4_goal_s1 = np.array([[xe_goal_s1],
                   [ye_goal_s1],
                   [ze_goal_s1]
                   ]) 
#4
        P5_goal_s1 = np.array([[xe2_goal_s1],
                   [ye2_goal_s1],
                   [ze2_goal_s1]
                   ])  
#6                   
        P6_goal_s1 = np.array([[xe3_goal_s1],
                   [ye3_goal_s1],
                   [ze3_goal_s1]
                   ]) 
        xe_goal_s2 =-length1*np.sin(T4_cur[0][0])-length2*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0])-length3*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])+dist_x
        ye_goal_s2 = length0+length1*np.cos(T4_cur[0][0])+length2*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0])+length3*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])
        ze_goal_s2 = length2*np.sin(T4_cur[1][0])+length3*np.sin(T4_cur[1][0]+T4_cur[2][0])+dist_z
    
        xe2_goal_s2 =-length1*np.sin(T5_cur[0][0])-length2*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0])-length3*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])-dist_x
        ye2_goal_s2 = length0+length1*np.cos(T5_cur[0][0])+length2*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0])+length3*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])
        ze2_goal_s2 = length2*np.sin(T5_cur[1][0])+length3*np.sin(T5_cur[1][0]+T5_cur[2][0])+dist_z

        xe3_goal_s2 =-length1*np.sin(T6_cur[0][0])-length2*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0])-length3*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])-dist_x
        ye3_goal_s2 = length0+length1*np.cos(T6_cur[0][0])+length2*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0])+length3*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])
        ze3_goal_s2 = length2*np.sin(T6_cur[1][0])+length3*np.sin(T6_cur[1][0]+T6_cur[2][0])+dist_z
#1
        P4_goal_s2 = np.array([[xe_goal_s2],
                   [ye_goal_s2],
                   [ze_goal_s2]
                   ]) 
#4
        P5_goal_s2 = np.array([[xe2_goal_s2],
                   [ye2_goal_s2],
                   [ze2_goal_s2]
                   ])  
#6                   
        P6_goal_s2 = np.array([[xe3_goal_s2],
                   [ye3_goal_s2],
                   [ze3_goal_s2]
                   ])  
        xe_goal_s3 =-length1*np.sin(T4_cur[0][0])-length2*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0])-length3*(np.sin(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])+dist_x
        ye_goal_s3 = length0+length1*np.cos(T4_cur[0][0])+length2*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0])+length3*(np.cos(T4_cur[0][0]))*np.cos(T4_cur[1][0]+T4_cur[2][0])
        ze_goal_s3 = length2*np.sin(T4_cur[1][0])+length3*np.sin(T4_cur[1][0]+T4_cur[2][0])
    
        xe2_goal_s3 =-length1*np.sin(T5_cur[0][0])-length2*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0])-length3*(np.sin(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])-dist_x
        ye2_goal_s3 = length0+length1*np.cos(T5_cur[0][0])+length2*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0])+length3*(np.cos(T5_cur[0][0]))*np.cos(T5_cur[1][0]+T5_cur[2][0])
        ze2_goal_s3 = length2*np.sin(T5_cur[1][0])+length3*np.sin(T5_cur[1][0]+T5_cur[2][0])

        xe3_goal_s3 =-length1*np.sin(T6_cur[0][0])-length2*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0])-length3*(np.sin(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])-dist_x
        ye3_goal_s3 = length0+length1*np.cos(T6_cur[0][0])+length2*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0])+length3*(np.cos(T6_cur[0][0]))*np.cos(T6_cur[1][0]+T6_cur[2][0])
        ze3_goal_s3 = length2*np.sin(T6_cur[1][0])+length3*np.sin(T6_cur[1][0]+T6_cur[2][0])
#1
        P4_goal_s3 = np.array([[xe_goal_s3],
                   [ye_goal_s3],
                   [ze_goal_s3]
                   ]) 
#4
        P5_goal_s3 = np.array([[xe2_goal_s3],
                   [ye2_goal_s3],
                   [ze2_goal_s3]
                   ])  
#6                   
        P6_goal_s3 = np.array([[xe3_goal_s3],
                   [ye3_goal_s3],
                   [ze3_goal_s3]
                   ]) 
    # 支持目標 
#2
        xe_goal  =-length1*np.sin(T_cur[0][0])-length2*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0])-length3*(np.sin(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])+dist_x
        ye_goal  = length0+length1*np.cos(T_cur[0][0])+length2*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0])+length3*(np.cos(T_cur[0][0]))*np.cos(T_cur[1][0]+T_cur[2][0])
        ze_goal  = length2*np.sin(T_cur[1][0])+length3*np.sin(T_cur[1][0]+T_cur[2][0])
     
        xe2_goal  =-length1*np.sin(T2_cur[0][0])-length2*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0])-length3*(np.sin(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])-dist_x
        ye2_goal  = length0+length1*np.cos(T2_cur[0][0])+length2*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0])+length3*(np.cos(T2_cur[0][0]))*np.cos(T2_cur[1][0]+T2_cur[2][0])
        ze2_goal  = length2*np.sin(T2_cur[1][0])+length3*np.sin(T2_cur[1][0]+T2_cur[2][0])

        xe3_goal  =-length1*np.sin(T3_cur[0][0])-length2*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0])-length3*(np.sin(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])-dist_x
        ye3_goal  = length0+length1*np.cos(T3_cur[0][0])+length2*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0])+length3*(np.cos(T3_cur[0][0]))*np.cos(T3_cur[1][0]+T3_cur[2][0])
        ze3_goal  = length2*np.sin(T3_cur[1][0])+length3*np.sin(T3_cur[1][0]+T3_cur[2][0])

        P_goal = np.array([[xe_goal],
                   [ye_goal],
                   [ze_goal]
                   ]) 
        P2_goal = np.array([[xe2_goal],
                   [ye2_goal],
                   [ze2_goal]
                   ])  
        P3_goal = np.array([[xe3_goal],
                   [ye3_goal],
                   [ze3_goal]
                   ])  
        
    #print("ima",P3_cur)
    #print("3goal:",P3_goal_s1 )
    #print("3goal_s2:",P3_goal_s2 )
    #print("3goal_s3:",P3_goal_s3 )
        while not rospy.is_shutdown():
       
            jacobian = make_jacobian_matrix(length1, T_cur[0][0], length2, T_cur[1][0], length3, T_cur[2][0]) # ヤコビ行列J
            jacobian2 = make_jacobian_matrix(length1, T2_cur[0][0], length2, T2_cur[1][0], length3, T2_cur[2][0])
            jacobian3 = make_jacobian_matrix(length1, T3_cur[0][0], length2, T3_cur[1][0], length3, T3_cur[2][0]) 
            jacobian4 = make_jacobian_matrix(length1, T4_cur[0][0], length2, T4_cur[1][0], length3, T4_cur[2][0]) 
            jacobian5 = make_jacobian_matrix(length1, T5_cur[0][0], length2, T5_cur[1][0], length3, T5_cur[2][0])
            jacobian6 = make_jacobian_matrix(length1, T6_cur[0][0], length2, T6_cur[1][0], length3, T6_cur[2][0])  
        
            jacobian_inverse = make_inverse_matrix(jacobian) 
            jacobian_inverse2 = make_inverse_matrix(jacobian2)
            jacobian_inverse3 = make_inverse_matrix(jacobian3)
            jacobian_inverse4 = make_inverse_matrix(jacobian4) 
            jacobian_inverse5 = make_inverse_matrix(jacobian5)
            jacobian_inverse6 = make_inverse_matrix(jacobian6)
    # 現在の手先P_current→目標手先P_goal 方向のベクトル
            P_cur_to_P_goal = P_goal - P_cur
            P2_cur_to_P_goal = P2_goal - P2_cur
            P3_cur_to_P_goal = P3_goal - P3_cur

    # 現在の手先P_current→目標手先P_goal 方向に向かう微小量ΔP
            P_delta_param = P_delta_param*1.05
            P_delta = P_cur_to_P_goal * P_delta_param
            P2_delta = P2_cur_to_P_goal * P_delta_param
            P3_delta = P3_cur_to_P_goal * P_delta_param
        
    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
            T_delta = jacobian_inverse @ P_delta
            T2_delta = jacobian_inverse2 @ P2_delta
            T3_delta = jacobian_inverse3 @ P3_delta
        
            if step == 1:
                P4_cur_to_P_goal = P4_goal_s1 - P4_cur
                P5_cur_to_P_goal = P5_goal_s1 - P5_cur
                P6_cur_to_P_goal = P6_goal_s1 - P6_cur
                P_delta_param_s1 = P_delta_param_s1*1.12

                P4_delta = P4_cur_to_P_goal * P_delta_param_s1
                P5_delta = P5_cur_to_P_goal * P_delta_param_s1
                P6_delta = P6_cur_to_P_goal * P_delta_param_s1

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T4_delta = jacobian_inverse4 @ P4_delta
                T5_delta = jacobian_inverse5 @ P5_delta
                T6_delta = jacobian_inverse6 @ P6_delta

                T4_new = T4_cur + T4_delta
                T5_new = T5_cur + T5_delta
                T6_new = T6_cur + T6_delta

                if  (P6_goal_s1[0][0]-P6_cur[0][0])**2  + (P6_goal_s1[1][0]-P6_cur[1][0])**2  + (P6_goal_s1[2][0]-P6_cur[2][0])**2 <  0.05**2 :
                    print("↑first-step")
                    step = 2
            if step == 2:
                P4_cur_to_P_goal = P4_goal_s2 - P4_cur
                P5_cur_to_P_goal = P5_goal_s2 - P5_cur
                P6_cur_to_P_goal = P6_goal_s2 - P6_cur
                P_delta_param_s2 = P_delta_param_s2*1.12

                P4_delta = P4_cur_to_P_goal * P_delta_param_s2
                P5_delta = P5_cur_to_P_goal * P_delta_param_s2
                P6_delta = P6_cur_to_P_goal * P_delta_param_s2

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T4_delta = jacobian_inverse4 @ P4_delta
                T5_delta = jacobian_inverse5 @ P5_delta
                T6_delta = jacobian_inverse6 @ P6_delta

                T4_new = T4_cur + T4_delta
                T5_new = T5_cur + T5_delta
                T6_new = T6_cur + T6_delta

                if  (P6_goal_s2[0][0]-P6_cur[0][0])**2  + (P6_goal_s2[1][0]-P6_cur[1][0])**2  + (P6_goal_s2[2][0]-P6_cur[2][0])**2 <  0.05**2 :
                    print("↑second-step")
                    step = 3
            if step == 3:
                P4_cur_to_P_goal = P4_goal_s3 - P4_cur
                P5_cur_to_P_goal = P5_goal_s3 - P5_cur
                P6_cur_to_P_goal = P6_goal_s3 - P6_cur
                P_delta_param_s3 = P_delta_param_s3*1.12

                P4_delta = P4_cur_to_P_goal * P_delta_param_s3
                P5_delta = P5_cur_to_P_goal * P_delta_param_s3
                P6_delta = P6_cur_to_P_goal * P_delta_param_s3

    # ΔP移動するために必要な「各関節の、微小角度ΔQ」を計算
                T4_delta = jacobian_inverse4 @ P4_delta
                T5_delta = jacobian_inverse5 @ P5_delta
                T6_delta = jacobian_inverse6 @ P6_delta

                T4_new = T4_cur + T4_delta
                T5_new = T5_cur + T5_delta
                T6_new = T6_cur + T6_delta

                if  (P6_goal_s3[0][0]-P6_cur[0][0])**2  + (P6_goal_s3[1][0]-P6_cur[1][0])**2  + (P6_goal_s3[2][0]-P6_cur[2][0])**2 <  0.05**2 :
                    print("↑third-step")
                    T4_new = T4_cur
                    T5_new = T5_cur 
                    T6_new = T6_cur              
    # ΔQだけ各関節を動かす
            T_new = T_cur + T_delta
            T2_new = T2_cur + T2_delta
            T3_new = T3_cur + T3_delta
        
            t2_1=T4_new[0][0]
            t2_2=T4_new[1][0]
            t2_3=T4_new[2][0]
            t3_1=T5_new[0][0]
            t3_2=T5_new[1][0]
            t3_3=T5_new[2][0]
            t5_1=T6_new[0][0]
            t5_2=T6_new[1][0]
            t5_3=T6_new[2][0]
            t1_1=T_new[0][0]
            t1_2=T_new[1][0]
            t1_3=T_new[2][0]
            t4_1=T2_new[0][0]
            t4_2=T2_new[1][0]
            t4_3=T2_new[2][0]
            t6_1=T3_new[0][0]
            t6_2=T3_new[1][0]
            t6_3=T3_new[2][0]

            pub1_A.publish(t1_1)
            pub4_A.publish(t4_1)
            pub6_A.publish(t6_1)
            pub1_B.publish(t1_2)
            pub4_B.publish(t4_2)
            pub6_B.publish(t6_2)
            pub1_C.publish(t1_3)
            pub4_C.publish(t4_3)
            pub6_C.publish(t6_3)  
        
            pub2_A.publish(t2_1)
            pub3_A.publish(t3_1)
            pub5_A.publish(t5_1)
            pub2_B.publish(t2_2)
            pub3_B.publish(t3_2)
            pub5_B.publish(t5_2)
            pub2_C.publish(t2_3)
            pub3_C.publish(t3_3)
            pub5_C.publish(t5_3)
        
            rate.sleep()
        
    # 更新後の関節角度T_newを用いて、手先位置P_newを計算（三角関数による順運動学）
    # link1の根本から見た時の，link1の先端位置
            x0_to_e, y0_to_e, z0_to_e =-length1*np.sin(T_new[0][0])-length2*(np.sin(T_new[0][0]))*np.cos(T_new[1][0])-length3*(np.sin(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length0+length1*np.cos(T_new[0][0])+length2*(np.cos(T_new[0][0]))*np.cos(T_new[1][0])+length3*(np.cos(T_new[0][0]))*np.cos(T_new[1][0]+T_new[2][0]),length2*np.sin(T_new[1][0])+length3*np.sin(T_new[1][0]+T_new[2][0])

            x0_to_e2, y0_to_e2, z0_to_e2 =-length1*np.sin(T2_new[0][0])-length2*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0])-length3*(np.sin(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length0+length1*np.cos(T2_new[0][0])+length2*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0])+length3*(np.cos(T2_new[0][0]))*np.cos(T2_new[1][0]+T2_new[2][0]),length2*np.sin(T2_new[1][0])+length3*np.sin(T2_new[1][0]+T2_new[2][0])

            x0_to_e3, y0_to_e3, z0_to_e3 =-length1*np.sin(T3_new[0][0])-length2*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0])-length3*(np.sin(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length0+length1*np.cos(T3_new[0][0])+length2*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0])+length3*(np.cos(T3_new[0][0]))*np.cos(T3_new[1][0]+T3_new[2][0]),length2*np.sin(T3_new[1][0])+length3*np.sin(T3_new[1][0]+T3_new[2][0])
 
            x0_to_e4, y0_to_e4, z0_to_e4 =-length1*np.sin(T4_new[0][0])-length2*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0])-length3*(np.sin(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length0+length1*np.cos(T4_new[0][0])+length2*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0])+length3*(np.cos(T4_new[0][0]))*np.cos(T4_new[1][0]+T4_new[2][0]),length2*np.sin(T4_new[1][0])+length3*np.sin(T4_new[1][0]+T4_new[2][0])

            x0_to_e5, y0_to_e5, z0_to_e5 =-length1*np.sin(T5_new[0][0])-length2*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0])-length3*(np.sin(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length0+length1*np.cos(T5_new[0][0])+length2*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0])+length3*(np.cos(T5_new[0][0]))*np.cos(T5_new[1][0]+T5_new[2][0]),length2*np.sin(T5_new[1][0])+length3*np.sin(T5_new[1][0]+T5_new[2][0])

            x0_to_e6, y0_to_e6, z0_to_e6 =-length1*np.sin(T6_new[0][0])-length2*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0])-length3*(np.sin(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length0+length1*np.cos(T6_new[0][0])+length2*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0])+length3*(np.cos(T6_new[0][0]))*np.cos(T6_new[1][0]+T6_new[2][0]),length2*np.sin(T6_new[1][0])+length3*np.sin(T6_new[1][0]+T6_new[2][0])
    # link1の根本（原点座標）から見た時の，エンドエフェクタの位置
            xe = x0 + x0_to_e
            ye = y0 + y0_to_e
            ze = z0 + z0_to_e
            xe2 = x0 + x0_to_e2
            ye2 = y0 + y0_to_e2
            ze2 = z0 + z0_to_e2
            xe3 = x0 + x0_to_e3
            ye3 = y0 + y0_to_e3
            ze3 = z0 + z0_to_e3
            xe4 = x0 + x0_to_e4
            ye4 = y0 + y0_to_e4
            ze4 = z0 + z0_to_e4
            xe5 = x0 + x0_to_e5
            ye5 = y0 + y0_to_e5
            ze5 = z0 + z0_to_e5
            xe6 = x0 + x0_to_e6
            ye6 = y0 + y0_to_e6
            ze6 = z0 + z0_to_e6
            P_new = np.array([[xe],
                          [ye],
                          [ze]])
            P2_new = np.array([[xe2],
                           [ye2],
                           [ze2]])
            P3_new = np.array([[xe3],
                           [ye3],
                           [ze3]]) 
            P4_new = np.array([[xe4],
                           [ye4],
                          [ ze4]])
            P5_new = np.array([[xe5],
                           [ye5],
                           [ze5]])
            P6_new = np.array([[xe6],
                           [ye6],
                           [ze6]]) 
    # 現在関節角度Q_current、現在手先位置P_currentを更新
            T_cur = T_new
            P_cur = P_new
            T2_cur = T2_new
            P2_cur = P2_new
            T3_cur = T3_new
            P3_cur = P3_new
            T4_cur = T4_new
            P4_cur = P4_new
            T5_cur = T5_new
            P5_cur = P5_new
            T6_cur = T6_new
            P6_cur = P6_new
            i=i+1
    # 目標手先P_goalに到達したら終了
            if  (P3_goal[0][0]-P3_cur[0][0])**2  + (P3_goal[1][0]-P3_cur[1][0])**2  + (P3_goal[2][0]-P3_cur[2][0])**2 <  goal_dis**2 :   
                print("2: ",P4_cur[2][0])  
                print("3: ",P5_cur[2][0])  
                print("5: ",P6_cur[2][0])           
                break
        print("loop-end") 
if __name__ == '__main__':
    time.sleep(1.5)
    commander()
