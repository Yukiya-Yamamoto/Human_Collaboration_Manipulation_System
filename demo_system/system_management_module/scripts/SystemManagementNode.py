#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

import rospy
from system_management_module.msg import *
from system_management_module.srv import *

# class SYSTEM_MANAGE():#ここの意味が分からん，なぜこうした？
#     def __init__(self):
#         # set server
#         rospy.Service('sys_manage_service',SystemManagement,self.callback)

#     def callback(self, req):
#         # リクエストの確認
#         print("")
#         print("Request Data:")
#         print(req)
#         srv = SystemManagementResponse()

#         srv.task_result_list.task_command_id = 1
#         srv.task_result_list.number_of_items_picked = 1
#         srv.task_result_list.task_result = True
        
#         return srv

def main():
    rospy.init_node("system_management_node")
    # SYSTEM_MANAGE()

    while not rospy.is_shutdown():
        set_task_command_list = TaskCommandList()
        set_task_command = TaskCommand()

        # print(type(set_task_command))
        # print(type(srv.task_info_list.task_command_list))
    
        rospy.wait_for_service('sys_manage_service')

        set_task_command.task_command_id = 1
        set_task_command.work_type_id = 1
        set_task_command.number_of_items_picked = 1
        
        set_task_command.work_presence_area = WorkPresenceArea()
        set_task_command.work_presence_area.start_point.x = -2.0
        set_task_command.work_presence_area.start_point.y = -2.0
        set_task_command.work_presence_area.start_point.z = 0.0
        set_task_command.work_presence_area.end_point.x = 2.0
        set_task_command.work_presence_area.end_point.y = 2.0
        set_task_command.work_presence_area.end_point.z = 2.0

        set_task_command.candidate_discharge_location = CandidateDischargeLocationAreaList()

        area_list = CandidateDischargeLocationArea()
        area_list.start_point.x = -2.0
        area_list.start_point.y = -2.0
        area_list.start_point.z = 0.0 
        area_list.end_point.x = 2.0 
        area_list.end_point.y = 2.0
        area_list.end_point.z = 2.0  
        
        set_task_command.candidate_discharge_location.candidate_discharge_location_area_list.append(area_list)

        print(set_task_command)
        set_task_command_list.task_command_list.append(set_task_command)

    
        srv = SystemManagementRequest()
        srv.task_info_list = set_task_command_list
        print("")
        print(srv.task_info_list.task_command_list[0].task_command_id)
        
        print("")
        print(srv)

        try:
            rospy.loginfo(set_task_command.task_command_id)
            print(srv)
            proxy = rospy.ServiceProxy('sys_manage_service', SystemManagement)
            result = proxy(srv)
            print(result)
            
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)

        # for subscribe response
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
