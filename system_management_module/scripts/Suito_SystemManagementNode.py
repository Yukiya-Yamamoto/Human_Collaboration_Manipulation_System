#!/usr/bin/env python
# coding: UTF-8

import rospy
import pprint
from system_management_module.msg import *
from system_management_module.srv import *

def main():
    rospy.init_node("system_management_node")
    
    while(1):#永遠ループの元（10/05）
        rospy.wait_for_service('sys_manage_service')

        ############################################
        set_task_command = TaskCommand()
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

        area_list = CandidateDischargeLocationArea()
        area_list.start_point.x = -2.0
        area_list.start_point.y = -2.0
        area_list.start_point.z = 0.0 
        area_list.end_point.x = 2.0 
        area_list.end_point.y = 2.0
        area_list.end_point.z = 2.0  
        
        set_task_command.candidate_discharge_location = CandidateDischargeLocationAreaList()
        set_task_command.candidate_discharge_location.candidate_discharge_location_area_list.append(area_list)

        print("set_task_command", set_task_command)

        set_task_command_list = TaskCommandList()
        set_task_command_list.task_command_list.append(set_task_command)

        srv = SystemManagementRequest()
        srv.task_info_list = set_task_command_list

        print("")
        print("aaa", srv.task_info_list.task_command_list[0].task_command_id)
        

        rospy.loginfo(set_task_command.task_command_id)
        print(srv)
        proxy = rospy.ServiceProxy('sys_manage_service', SystemManagement)
        result = proxy(srv)
        print("result=")
        print("")
        print(result)
            
        # except rospy.ServiceException as e:
        #     rospy.loginfo("ServiceException : %s" % e)

        # rospy.spin()

if __name__ == "__main__":
    main()
