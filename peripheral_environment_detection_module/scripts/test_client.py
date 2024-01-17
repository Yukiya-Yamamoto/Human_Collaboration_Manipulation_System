#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは，周辺環境計測モジュールを実装するために作成したコードです．
#PerEnvDetectNode.pyの単体検証用のコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
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
from peripheral_environment_detection_module.msg import *
from peripheral_environment_detection_module.srv import *

# メイン
def main():
    # ノードの初期化
    rospy.init_node("test_client_node")

    # サービスが利用可能になるまで待機
    rospy.wait_for_service('per_env_det_service')

    set_data = AreaSetup()
    set_data.target_area.start_point.x = 1.0
    set_data.target_area.start_point.y = 2.0
    set_data.target_area.start_point.z = 3.0
    set_data.target_area.end_point.x = 4.0
    set_data.target_area.end_point.y = 5.0
    set_data.target_area.end_point.z = 6.0
    set_data.area_division.cooperative_work_range = 8
    set_data.area_division.non_cooperative_work_range = 4
    set_list = AreaSetupList()
    set_list.area_setup_info.append(set_data)

    try:
        # サービスプロクシの生成
        proxy = rospy.ServiceProxy('per_env_det_service', MonitoringAreaSetup)

        # リクエストの送信
        result = proxy(set_list)

        # ログ出力
        print("Setup Result")
        print(result.setup_result)
    except rospy.ServiceException as e:
        rospy.loginfo("ServiceException : %s" % e)

    # ノード終了まで待機
    rospy.spin()


if __name__ == "__main__":
    main()
