#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，排出位置検出サブモジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/08/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from discharge_position_detect_module.srv import *

def response_data(req):
    print(req)
    srv= DischargePositionDetectionResultResponse()
    srv.task_command_id = 1
    return srv

def dis_pos_detect_server():
    rospy.init_node('discharge_position_detect_server')
    s = rospy.Service('discharge_position_detect_module', DischargePositionDetectionResult, response_data)
    print(DischargePositionDetectionResultResponse())
    rospy.spin()

if __name__ == "__main__":
   dis_pos_detect_server()