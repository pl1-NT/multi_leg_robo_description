#!/usr/bin/env python3

import rospy
import math as m
import numpy as np
from gazebo_msgs.msg import LinkStates
from wheel_robo_description.srv import leg2end_position, leg2end_positionResponse # サービスファイルのインポート

L0 = 79.929
L1 = 50.929
L2 = 66.929
L3 = 99.862
L3_1 = 84.638
L3_2 = 52.999
# グローバル変数としてhball_2_linkの位置を保持
hball_2_position = None

    
def link_states_callback(data):
    global po_xr2, po_yr2, bobj_po, fobj_po, leg2_end_position ,body_position, aim_x, aim_y
    
    # インデックスを取得
    indexr = data.name.index('myrobot::base_footprint')
    indexb = data.name.index('myrobot::backobject_link')
    indexf = data.name.index('myrobot::frontobject_link')
    index1 = data.name.index('myrobot::hball_1_link')
    index2 = data.name.index('myrobot::hball_2_link')
    index3 = data.name.index('myrobot::hball_3_link')
    index4 = data.name.index('myrobot::hball_4_link')
    index5 = data.name.index('myrobot::hball_5_link')
    index6 = data.name.index('myrobot::hball_6_link')    

        
    # 位置を取得
    body_position = data.pose[indexr].position
    bobj_po = data.pose[indexb].position
    fobj_po = data.pose[indexf].position
    leg1_end_position = data.pose[index1].position
    leg2_end_position = data.pose[index2].position
    leg3_end_position = data.pose[index3].position    
    leg4_end_position = data.pose[index4].position
    leg5_end_position = data.pose[index5].position
    leg6_end_position = data.pose[index6].position  
    #傾き計算
    theta = m.atan((bobj_po.y-fobj_po.y)/(bobj_po.x-fobj_po.x)  )
    #body中心との距離    
    po_xw2 = leg2_end_position.x-body_position.x
    po_yw2 = leg2_end_position.y-body_position.y
    #bodyとの相対座標に変換
    po_xr2 = po_xw2 * m.cos(-theta) - po_yw2 * m.sin(-theta)
    po_yr2 = po_xw2 * m.sin(-theta) + po_yw2 * m.cos(-theta)  
    
    #前進位置
    aim_x = po_xr2*1000 +25*m.cos(theta)
    aim_y = po_yr2*1000 +25*m.sin(theta)
        
def inv_kin(x,y,z):
    t1 = m.atan(-x/(y-L0))
    
    t2_3_a = m.asin(  ( -x*x/(m.sin(t1))**2 -L1**2 +L2**2 -L3**2 -2*x*L1/m.sin(t1)  -z**2  )  /  ( 2*L3*m.sqrt( (x/m.sin(t1) + L1)**2 + z **2)     )     )

    t_a = m.asin( -(x/m.sin(t1) + L1)/ m.sqrt( (x/m.sin(t1) + L1)**2 + z **2) )
    
    t2_3 = t2_3_a - t_a

    t2 =  m.asin((z -L3*m.sin(t2_3)) / L2) 
    
    t23 = m.asin((z-L2*m.sin(t2))/L3)
    
    t3 = t23 - t2 
    return t1,t2,t3

def handle_get_link_position(req):
    # サービスにリクエストが来たときにhball_2_linkの位置を返す
    if leg2_end_position is not None:
        t1, t2, t3 =  inv_kin(aim_x, aim_y, leg2_end_position.z-body_position.z*1000)
        return leg2end_positionResponse(x=aim_x, y = aim_y, z=leg2_end_position.z-body_position.z, p=t1,q=-t2,r=-t3)
    else:
        return leg2end_positionResponse(x=0.0, y=0.0, z=0.0)  # デフォルト値

def service_server():
    # ノード初期化
    rospy.init_node('hball_2_link_service_server', anonymous=True)
    
    # /gazebo/link_statesのサブスクライバ
    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)
    
    # サービスの作成
    rospy.Service('get_leg2_end_position', leg2end_position, handle_get_link_position)
    
    rospy.spin()

if __name__ == '__main__':
    service_server()

