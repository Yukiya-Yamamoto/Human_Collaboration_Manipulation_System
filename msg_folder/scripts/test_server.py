#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　
#SystemManagementNode.pyの単体検証用のコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

import rospy
from rospy.topics import Publisher, Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from system_management_module.msg import *
from system_management_module.srv import * 

def callback(req):
    # リクエストの確認
    print("")
    print("Request Data:")
    print(req)
    srv = SystemManagementResponse()

    srv.task_result_list.task_command_id = 1
    srv.task_result_list.number_of_items_picked = 1
    srv.task_result_list.task_result = True
    
    return srv

def main():
    rospy.init_node("test_server_node")
    s = rospy.Service('sys_manage_service',SystemManagement,callback)

    rospy.spin()
    

if __name__ == "__main__":

    main()
    
