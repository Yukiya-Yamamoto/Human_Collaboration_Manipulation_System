#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，排出位置検出サブモジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/09/14
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
from std_msgs.msg import *
from geometry_msgs.msg import *
import rospy
from discharge_position_detect_module.srv import *

def dis_pos_detect_client():
    rospy.init_node('client')
    rospy.wait_for_service('discharge_position_detect_module')
    set_area = DischargePositionDetectionResultRequest()
    set_area.discharge_position_list.x = 1.0
    set_area.discharge_position_list.y = 1.0
    set_area.discharge_position_list.z = 1.0

    try:
        discharge_position_detect = rospy.ServiceProxy('discharge_position_detect_module', DischargePositionDetectionResult)
        response = discharge_position_detect(set_area)
        print("Service response:", response)

    except rospy.ServiceException:
        print ("hoge")


if __name__ == "__main__":
    dis_pos_detect_client()