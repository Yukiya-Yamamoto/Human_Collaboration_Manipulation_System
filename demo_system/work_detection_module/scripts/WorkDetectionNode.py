#!/usr/bin/env python
# coding: UTF-8
#####################################################################################################################
#このノードは，ワーク検出モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
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
import yaml
import rospkg
from rospy.topics import Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from aruco_msgs.msg import *
from work_detection_module.msg import *
from work_detection_module.srv import *

class WorkDetect:
    def __init__(self):
        self.length_sub = Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.ArucoCallback)

        self.setup_req = rospy.Service('work_det_service', WorkDetection, self.pose_request)

        self.aruco_data = MarkerArray()

        print("Initialization done")

    def ArucoCallback(self, aruco_):
        self.aruco_data = aruco_.markers

    def pose_request(self, req_):
        # リクエストの確認
        print("")
        print("Request Data:")
        print(req_)

        rospack = rospkg.RosPack()
        rospack.list() 
        srv = WorkDetectionResponse()
        set_data = WorkDetectionResult()
        if req_.task_command_id == 1:
            if req_.work_type_id == 1:
                path = rospack.get_path('work_detection_module')
                with open(path + '/config/sandwich_id.yaml') as f:
                    config = yaml.load(f)

            if req_.work_type_id == 2:
                path = rospack.get_path('work_detection_module')
                with open(path + '/config/drink_id.yaml') as f:
                    config = yaml.load(f)

            if config and self.aruco_data:
                for i in range(len(self.aruco_data)):
                    if self.aruco_data[i].id == config[i]:
                        if req_.target_area.start_point.x < self.aruco_data[i].pose.pose.position.x < req_.target_area.end_point.x and\
                           req_.target_area.start_point.y < self.aruco_data[i].pose.pose.position.y < req_.target_area.end_point.y and\
                           req_.target_area.start_point.z < self.aruco_data[i].pose.pose.position.z < req_.target_area.end_point.z:
                                set_data.pose.position.x = self.aruco_data[i].pose.pose.position.x
                                set_data.pose.position.y = self.aruco_data[i].pose.pose.position.y
                                set_data.pose.position.z = self.aruco_data[i].pose.pose.position.z
                                set_data.pose.orientation.x = self.aruco_data[i].pose.pose.orientation.x
                                set_data.pose.orientation.y = self.aruco_data[i].pose.pose.orientation.y
                                set_data.pose.orientation.z = self.aruco_data[i].pose.pose.orientation.z
                                set_data.pose.orientation.w = self.aruco_data[i].pose.pose.orientation.w
                                print(set_data.pose)
            
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_result_list.work_info_list.append(set_data)
            print(srv.work_detection_result_list.work_info_list)
                
        elif req_.task_command_id == 2:
            # 未実装
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_result_list.work_info_list.append(set_data)

        else:
            # 定義されていない作業指令IDの場合
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_result_list.work_info_list.append(set_data)  
        return srv

if __name__ == "__main__":
    rospy.init_node("work_detection_node")
    wd = WorkDetect()
    rospy.spin()
  