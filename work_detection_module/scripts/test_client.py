#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，ワーク検出モジュールを実装するために作成したコードです．　　
#WorkDetectionNode.pyの単体検証用のコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
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
from work_detection_module.msg import *
from work_detection_module.srv import *

def main():
    rospy.init_node("test_client_node")

    rospy.wait_for_service('work_det_service')

    task_command_id = 1
    work_type_id = 1
    set_area = TargetArea()
    set_area.start_point.x = -2.0
    set_area.start_point.y = -2.0
    set_area.start_point.z = 0.0
    set_area.end_point.x = 2.0
    set_area.end_point.y = 2.0
    set_area.end_point.z = 2.0

    try:
        proxy = rospy.ServiceProxy('work_det_service', WorkDetection)

        result = proxy(task_command_id, work_type_id, set_area)

        print("Pose Result")
        print(result.work_detection_result_list)
    except rospy.ServiceException as e:
        rospy.loginfo("ServiceException : %s" % e)

    rospy.spin()

if __name__ == "__main__":
    main()
