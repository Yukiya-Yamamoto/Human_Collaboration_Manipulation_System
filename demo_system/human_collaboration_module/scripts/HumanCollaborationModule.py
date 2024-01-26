#!/usr/bin/env python
# coding: UTF-8

#####################################################################################################################
#このノードは．人協働マニピュレーションモジュールを機能単位で作成したモジュールです．　
#ハンド部分のモジュールと人検知部分の追加を行っています．　　　　　　　　　　　　　　 #
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/11/07
#08/30 排出位置検出システム編集
#09/21 排出位置検出システム　統合
#09/28 WS環境認識システム作成
#10/05 排出位置検出システム　統合確認　
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
import rospy,tf, tf2_ros
from rospy.topics import Publisher, Subscriber

#smachの使用
from smach import State, StateMachine, Concurrence
import smach_ros
import moveit_commander
from geometry_msgs.msg import *
from std_msgs.msg import String, Bool

#周辺環境認識用のsrvとmsg
from peripheral_environment_detection_module.msg import *
from peripheral_environment_detection_module.srv import *

#ワーク検知用のsrvとmsg
from work_detection_module.msg import *
from work_detection_module.srv import *

#上位アプリ用のsrvとmsg
from system_management_module.msg import *
from system_management_module.srv import * 

#排出位置検出のsrv
from discharge_position_detect_module.srv import *

## original
import time



##################################################################
#　　　　　　　　　　　 クラス定義　                            #
##################################################################
class HumanCollaboration:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
    
        self.group = moveit_commander.MoveGroupCommander("motoman_gp8")
        self.group.set_pose_reference_frame("base_link")
        self.group.set_end_effector_link("grasp_point")
        self.group.set_planner_id( "RRTConnectkConfigDefault" )
        self.group.allow_replanning( True )
        
        self.area_setup_result = SetupResult()
        self.work_detect_result = WorkDetectionResultList()
        self.system_management = SystemManagementResponse()
        


#把持点情報取得
    def set_grasp_position(self, x, y, z, vel=1.0):
  
      quat = tf.transformations.quaternion_from_euler(0, -3.14, -1.57)
  
      target_pose = PoseStamped()
      target_pose.header.frame_id = "base_link"
      target_pose.pose.orientation.x = quat[0]
      target_pose.pose.orientation.y = quat[1]
      target_pose.pose.orientation.z = quat[2]
      target_pose.pose.orientation.w = quat[3]
      target_pose.pose.position.x = x
      target_pose.pose.position.y = y
      target_pose.pose.position.z = z
  
      self.group.set_pose_target(target_pose)
      self.group.set_max_velocity_scaling_factor(vel)
      plan = self.group.plan()
  
      if type(plan) is tuple: # for noetic
          plan = plan[1]
  
      if(len(plan.joint_trajectory.points)==0):
        rospy.logwarn("IK can't be solved")
        self.group.clear_pose_targets()
        return 'retry'
      else: 
        self.group.execute(plan)
        return 'succeeded'


#関節情報取得
    def set_joint_value(self, joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t, vel=1.0):
  
      joint_goal = self.group.get_current_joint_values()
  
      joint_goal[0] = joint_1_s
      joint_goal[1] = joint_2_l
      joint_goal[2] = joint_3_u
      joint_goal[3] = joint_4_r
      joint_goal[4] = joint_5_b
      joint_goal[5] = joint_6_t
  
      self.group.set_joint_value_target(joint_goal)
      self.group.set_max_velocity_scaling_factor(vel)
      plan = self.group.plan()

      if type(plan) is tuple: # for noetic
          plan = plan[1]
  
      if(len(plan.joint_trajectory.points)==0):
        rospy.logwarn("can't be solved lifter ik")
        self.group.clear_pose_targets()
        return 'retry'
      else: 
        self.group.execute(plan)
        return 'succeeded'
      

#現在の関節情報の取得
    def set_current_position(self, offset_x, offset_y, offset_z, vel=1.0):
  
      quat = tf.transformations.quaternion_from_euler(0, -3.14, -1.57)
  
      target_pose = PoseStamped()
      target_pose.header.frame_id = "base_link"
      target_pose.pose.orientation.x = quat[0]
      target_pose.pose.orientation.y = quat[1]
      target_pose.pose.orientation.z = quat[2]
      target_pose.pose.orientation.w = quat[3]
      target_pose.pose.position.x = self.group.get_current_pose().pose.position.x + offset_x
      target_pose.pose.position.y = self.group.get_current_pose().pose.position.y + offset_y
      target_pose.pose.position.z = self.group.get_current_pose().pose.position.z + offset_z
  
      self.group.set_pose_target(target_pose)
      self.group.set_max_velocity_scaling_factor(vel)
      plan = self.group.plan()
  
      if type(plan) is tuple: # for noetic
          plan = plan[1]
  
      if(len(plan.joint_trajectory.points)==0):
        rospy.logwarn("IK can't be solved")
        self.group.clear_pose_targets()
        return 'retry'
      else: 
        self.group.execute(plan)
        return 'succeeded'
  


#tf座標変換
    def set_tf_position(self, offset_x, offset_y, offset_z, vel=1.0, tf_pose="grasp_point"):
  
      quat = tf.transformations.quaternion_from_euler(0, -3.14, -1.57)
  
      print(tf_pose)
  
      listener = tf.TransformListener()
      try:
        listener.waitForTransform("/base_link", tf_pose, rospy.Time(0), rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform('/base_link', tf_pose, rospy.Time(0))
      except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Not found frame...")
        return 'retry'
  
  
      print(trans[0])
      print(trans[1])
      print(trans[2])
  
      target_pose = PoseStamped()
      target_pose.header.frame_id = "base_link"
      target_pose.pose.orientation.x = quat[0]
      target_pose.pose.orientation.y = quat[1]
      target_pose.pose.orientation.z = quat[2]
      target_pose.pose.orientation.w = quat[3]
      target_pose.pose.position.x = trans[0] + offset_x
      target_pose.pose.position.y = trans[1] + offset_y
      target_pose.pose.position.z = offset_z
    
      self.group.set_pose_target(target_pose)
      self.group.set_max_velocity_scaling_factor(vel)
      plan = self.group.plan()
  
      if type(plan) is tuple: # for noetic
          plan = plan[1]
  
      if(len(plan.joint_trajectory.points)==0):
        rospy.logwarn("IK can't be solved")
        self.group.clear_pose_targets()
        return 'retry'
      else: 
        self.group.execute(plan)
        return 'succeeded'

#初期姿勢   
    def set_initial_pose(self):
  
      joint_goal = self.group.get_current_joint_values()
     
      for i in range(0,len(joint_goal)):
        joint_goal[i] = 0
      joint_goal[4] = -1.57
  
      self.group.set_joint_value_target(joint_goal)
      plan = self.group.plan()
  
      if type(plan) is tuple: # for noetic
          plan = plan[1]
      self.group.go()
  
      if(len(plan.joint_trajectory.points)==0):
        rospy.logwarn("IK can't be solved")
        self.group.clear_pose_targets()
        return 'retry'
      else: 
        self.group.execute(plan)
        return 'succeeded'
    
#ワーク検知用
    def work_detection(self, req_task_id, req_work_id, req_target_area):
        try:
            proxy = rospy.ServiceProxy('work_det_service', WorkDetection)

            result = proxy(req_task_id, req_work_id, req_target_area)

            rospy.loginfo("Detection Result")
            rospy.loginfo(result.work_detection_result_list)
            self.work_detect_result = result.work_detection_result_list
            return True
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False
    
  
#排出位置検出用 0830編集
    def discharge_position(self, req_discharge_position_list):
        try:
            proxy = rospy.ServiceProxy('discharge_position_detect_module', DischargePositionDetectionResult)

            result = proxy(req_discharge_position_list)

            rospy.loginfo("Detection Result")
            rospy.loginfo(result)
            self.task_id = result
            return True
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False
        
#上位アプリ用
    def callback(self, task_command_id, number_of_items_picked,task_result):
           print("")
           print("Request Data:")
           print(SystemManagementRequest())

       
           self.system_management.task_result_list.task_command_id = 1
           self.system_management.task_result_list.number_of_items_picked = 1
           self.system_management.task_result_list.task_result = True
           return self.system_management
    
#人検知用
    def peripheral_environment_area_set(self, req_area_data):
        req_list = AreaSetupList()
        req_list.area_setup_info.append(req_area_data)

        try:
            proxy = rospy.ServiceProxy('per_env_det_service', MonitoringAreaSetup)

            result = proxy(req_list)

            rospy.loginfo("Area Setup Result")
            rospy.loginfo(result.setup_result)
            self.area_setup_result = result.setup_result
            return True
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False
    
#座標変換
    def tf_broadcast(self):
        if self.work_detect_result:
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "cam_frame"
            static_transformStamped.child_frame_id = "marker_frame"

            static_transformStamped.transform.translation.x = self.work_detect_result.work_info_list[0].pose.position.x
            static_transformStamped.transform.translation.y = self.work_detect_result.work_info_list[0].pose.position.y
            static_transformStamped.transform.translation.z = self.work_detect_result.work_info_list[0].pose.position.z

            static_transformStamped.transform.rotation.x = self.work_detect_result.work_info_list[0].pose.orientation.x
            static_transformStamped.transform.rotation.y = self.work_detect_result.work_info_list[0].pose.orientation.y
            static_transformStamped.transform.rotation.z = self.work_detect_result.work_info_list[0].pose.orientation.z
            static_transformStamped.transform.rotation.w = self.work_detect_result.work_info_list[0].pose.orientation.w

            broadcaster.sendTransform(static_transformStamped)
            return True
        else:
            return False


    def set_wait_time(self, time):
        rospy.sleep(time)
        return 'succeeded'
###########################################
#グリッパー部分
class GripperControlCommand:
  def __init__(self):
        # publisher
        self.control_pub = Publisher('/gripper_judge', String, queue_size=100)
        # subscriber
        self.result_sub = Subscriber('/hand_result', String, self.ResultCallback)

        self.send_judge = String()

        self.recieve_result = String()

        self.GRIPPER_RETRY_CNT = 100

        self.GRIPPER_MOVE_WAIT = 5.0

  def gripper_control(self, command="open"):
    wait_cnt = 0
    if(command == "open"):
      while True:
        self.send_judge = "open"
        self.control_pub.publish(self.send_judge)
        
        if(self.recieve_result.data == "done"):
          self.send_judge = "done"
          self.control_pub.publish(self.send_judge)
          break
        elif (wait_cnt > self.GRIPPER_RETRY_CNT):
          print("gripper retry...")
          break

        rospy.sleep(0.1)
        wait_cnt += 1

    if(command == "close"):
      while True:
        self.send_judge = "close"
        self.control_pub.publish(self.send_judge)
        
        if(self.recieve_result.data == "done"):
          self.send_judge = "done"
          self.control_pub.publish(self.send_judge)
          break
        elif (wait_cnt > self.GRIPPER_RETRY_CNT):
          print("gripper retry...")
          break

        rospy.sleep(0.1)
        wait_cnt += 1

    rospy.sleep(self.GRIPPER_MOVE_WAIT)
  
    if (wait_cnt > self.GRIPPER_RETRY_CNT):
      return 'retry'
    else: 
      return 'succeeded'

  def ResultCallback(self, result_):
    self.recieve_result = result_

class GRIPPER_COM(State):
  def __init__(self, command="open"):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.command = command

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
        rospy.loginfo('State GRIPPER_COM is being preempted!!!')
        self.service_preempt()
        return 'preempted'
      self.counter += 1
      if(gc.gripper_control(self.command) == 'succeeded'):
        return 'succeeded'
      else:
        return 'retry'
    else:
      return 'aborted'

###########################################


##################################################################
#　　　　　　　　　　　 smach実行動作                            #
##################################################################

#ロボット関節情報の取得を実行
class JOINT_MANIP(State):
  def __init__(self, joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t, vel=1.0):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.joint_1_s = joint_1_s
    self.joint_2_l = joint_2_l
    self.joint_3_u = joint_3_u
    self.joint_4_r = joint_4_r
    self.joint_5_b = joint_5_b
    self.joint_6_t = joint_6_t
    self.vel = vel

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
          rospy.loginfo('State JOINT_MANIP is being preempted!!!')
          self.service_preempt()
          return 'preempted'
      self.counter += 1
      rospy.loginfo('Manipulate at ({},{},{},{},{},{}) in scale velocity {}'\
              .format(self.joint_1_s, self.joint_2_l, self.joint_3_u, self.joint_4_r, self.joint_5_b, self.joint_6_t, self.vel))
      if(hc.set_joint_value(self.joint_1_s, self.joint_2_l, self.joint_3_u, \
          self.joint_4_r, self.joint_5_b, self.joint_6_t, self.vel) == 'succeeded'):
        return 'succeeded'
      else: 
        return 'retry'
    else:
      return 'aborted'

#ロボット現在姿勢取得を実行
class CURRENT_MANIP(State):
  def __init__(self, offset_x, offset_y, offset_z, vel=1.0, direction="side"):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.offset_x = offset_x
    self.offset_y = offset_y
    self.offset_z = offset_z
    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
          rospy.loginfo('State CURRENT_MANIP is being preempted!!!')
          self.service_preempt()
          return 'preempted'
      self.counter += 1
      rospy.loginfo('Manipulate at ({},{},{}) in scale velocity {}'.format(self.offset_x, self.offset_y, self.offset_z, self.vel))
      if(hc.set_current_position(self.offset_x, self.offset_y, self.offset_z, self.vel) == 'succeeded'):
        return 'succeeded'
      else:
        return 'retry'
    else:
      return 'aborted'


class TF_MANIP(State):
  def __init__(self, offset_x, offset_y, offset_z, vel=1.0, direction="side", tf_pose="grasp_point"):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.offset_x = offset_x
    self.offset_y = offset_y
    self.offset_z = offset_z
    self.vel = vel
    self.direction = direction
    self.tf_pose = tf_pose

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
          rospy.loginfo('State TF_MANIP is being preempted!!!')
          self.service_preempt()
          return 'preempted'
      self.counter += 1
      rospy.loginfo('Manipulate at ({},{},{}) in scale velocity {}'.format(self.offset_x, self.offset_y, self.offset_z, self.vel))
      if(hc.set_tf_position(self.offset_x, self.offset_y, self.offset_z, self.vel, self.tf_pose) == 'succeeded'):
        return 'succeeded'
      else: 
        return 'retry'
    else:
      return 'aborted'
    
#ロボット手先位置座標の取得を実行
class NORMAL_MANIP(State):
  def __init__(self,x,y,z,vel=1.0,direction="side"):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.x = x
    self.y = y
    self.z = z
    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
          rospy.loginfo('State NORMAL_MANIP is being preempted!!!')
          self.service_preempt()
          return 'preempted'
      self.counter += 1
      rospy.loginfo('Manipulate at ({},{},{}) in scale velocity {}'.format(self.x,self.y,self.z,self.vel))
      if(hc.set_grasp_position(self.x,self.y,self.z,self.vel) == 'succeeded'):
        return 'succeeded'
      else: 
        return 'retry'
    else:
      return 'aborted'

#ロボット初期姿勢への動作実行
class INIT_POSE(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0

  def execute(self, userdata):
    if self.counter < 3:
      if self.preempt_requested():
          rospy.loginfo('State INIT_POSE is being preempted!!!')
          self.service_preempt()
          return 'preempted'
      self.counter += 1
      rospy.loginfo('Initialize Pose')
      if(hc.set_initial_pose() == 'succeeded'):
        return 'succeeded'
      else: 
        return 'retry'
    else:
      return 'aborted'

#座標変換実行
class TF_BROADCAST(State):
  def __init__(self):
      State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
      self.counter = 0

  def execute(self, userdata):
      if self.counter < 3:
          if self.preempt_requested():
              rospy.loginfo('State AREASET is being preempted!!!')
              self.service_preempt()
              return 'preempted'
          self.counter += 1
          if(hc.tf_broadcast()): 
              return 'succeeded'
          else: 
              return 'retry'
      else:
          return 'aborted'


#ワーク検知実行
class WORK_DETECT(State):
  def __init__(self, req_task_id, req_work_id, req_target_area):
    State.__init__(self, outcomes=['succeeded','retry','aborted','preempted'])
    self.counter = 0
    self.req_task_id = req_task_id
    self.req_work_id = req_work_id
    self.req_target_area = req_target_area

  def execute(self, userdata):
    rospy.loginfo("waiting work_det_service")
    rospy.wait_for_service('work_det_service')
    rospy.loginfo("work_det_service comes up")
    if self.counter < 3:
        if self.preempt_requested():
            rospy.loginfo('State AREASET is being preempted!!!')
            self.service_preempt()
            return 'preempted'
        self.counter += 1
        if(hc.work_detection(self.req_task_id, self.req_work_id, self.req_target_area)): 
            return 'succeeded'
        else: 
            return 'retry'
    else:
        return 'aborted'
        

#排出位置検出実行 0830編集
class DISCHARGE_DETECT(State):
  def __init__(self, req_discharge_position_list):
    State.__init__(self, outcomes=['succeeded','retry','aborted'])
    self.counter = 0
    self.req_discharge_position_list = req_discharge_position_list

  def execute(self, userdata):
    rospy.loginfo("waiting discharge_position")
    rospy.wait_for_service('discharge_position_detect_module')
    rospy.loginfo("discharge_position comes up")
    if self.counter < 3:
        # if self.preempt_requested():
        #     rospy.loginfo('State POSITION is being preempted!!!')
        #     self.service_preempt()
        #     return 'preempted'
        # self.counter += 1
        if(hc.discharge_position(self.req_discharge_position_list)): 
            return 'succeeded'
        else: 
            return 'retry'
    else:
        return 'aborted'
    

#周辺環境認識実行
class AREA_SET(State):
  def __init__(self, req_area_data):
    State.__init__(self, outcomes=['succeeded','retry','aborted'])
    self.counter = 0
    self.req_area_data = req_area_data

  def execute(self, userdata):
    rospy.loginfo("waiting per_env_det_service")
    rospy.wait_for_service('per_env_det_service')
    rospy.loginfo("per_env_det_service comes up")
    if self.counter < 3:
        self.counter += 1
        if(hc.peripheral_environment_area_set(self.req_area_data)): 
            return 'succeeded'
        else: 
            return 'retry'
    else:
        return 'aborted'
        

#上位アプリ実行       
class SYSTEM_CALL(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.req = False

    ## service define
    self.server = rospy.Service('sys_manage_service', SystemManagement, self.callback)

  def callback(self, req):
    # リクエストの確認
    print("")
    print("Request Data:")
    print(self.req)
    self.srv = SystemManagementResponse()

    self.srv.task_result_list.task_command_id = 1
    self.srv.task_result_list.number_of_items_picked = 1
    self.srv.task_result_list.task_result = True

    self.req = True

    return self.srv
       
  def execute(self,userdata):

      # 開始の時刻を保存
      start_time = time.time()
      # 経過した時刻を取得
      end_time = time.time()

      # ５秒間サービス通信を確認
      rospy.loginfo('Wait 5 seconds for service')
      while(end_time-start_time <= 5):
          end_time = time.time()
          if (self.req == True):
              return 'succeeded'        
          
      return 'aborted'

#作業結果の送信
class RESULT_RECEIVE(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self,userdata):
        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()

        # ５秒間サービス通信を確認
        rospy.loginfo('Wait 5 seconds for service')
        rospy.wait_for_service('sys_manage_service_2')
        while(end_time-start_time <= 5):
            end_time = time.time()

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

            set_task_command_list = TaskCommandList()
            set_task_command_list.task_command_list.append(set_task_command)

            srv = SystemManagementRequest()
            srv.task_info_list = set_task_command_list
            ############################################

            try:
                proxy = rospy.ServiceProxy('sys_manage_service_2', SystemManagement)
                result = proxy(srv)
                print("result = ", result)
            
            except rospy.ServiceException as e:
                rospy.loginfo("ServiceException : %s" % e)

            return 'succeeded'
        
        return 'aborted'

##############################################################################################

#人検知実行
def child_manip_cb(outcome_map):
    if outcome_map['HUMAN DETECT'] == 'invalid':
        return True
    elif outcome_map['RUNNING'] == 'succeeded':
        return True
    elif outcome_map['RUNNING'] == 'aborted':
        return True
    else:
        return False

def out_manip_cb(outcome_map):
    if outcome_map['HUMAN DETECT'] == 'invalid':
        return 'motion_stop'
    elif outcome_map['RUNNING'] == 'aborted':
        return 'aborted'
    else:
        return 'succeeded'
    
def humandetect_cb(ud, msg):
    if msg.data == True:
      return False
    else:
      return True

def resumemove_cb(ud, msg):
    if msg.data == False:
      return False
    else:
      return True


class WAIT_TIME(State):
  def __init__(self, time):
     State.__init__(self, outcomes=['succeeded','aborted','preempted'])
     self.time = time

  def execute(self, userdata):
    rospy.loginfo('Waiting ...')
    if self.preempt_requested():
        rospy.loginfo('State WAIT_TIME is being preempted!!!')
        self.service_preempt()
        return 'preempted'
    if(hc.set_wait_time(self.time) == 'succeeded'):
      return 'succeeded'
    else: 
      return 'aborted'



##################################################################
#　　　　　　　　　　　 以下実行処理内容                         #
##################################################################   
if __name__ == '__main__':
  hc = HumanCollaboration()
  rospy.init_node('human_collaboration_node')

  set_data = AreaSetup()
  set_data.target_area.start_point.x = 1.0
  set_data.target_area.start_point.y = 2.0
  set_data.target_area.start_point.z = 3.0
  set_data.target_area.end_point.x = 4.0
  set_data.target_area.end_point.y = 5.0
  set_data.target_area.end_point.z = 6.0
  set_data.area_division.cooperative_work_range = 8
  set_data.area_division.non_cooperative_work_range = 4

  task_command_id = 1
  work_type_id = 1

  discharge_position_list=Point()
  discharge_position_list.x = 1.0
  discharge_position_list.y = 1.0
  discharge_position_list.z = 1.0

  set_area = TargetArea()
  set_area.start_point.x = -2.0
  set_area.start_point.y = -2.0
  set_area.start_point.z = 0.0
  set_area.end_point.x = 2.0
  set_area.end_point.y = 2.0
  set_area.end_point.z = 2.0



##################################################################
#　　　　　　　　　　　  作業中動作                      　　　  #
##################################################################
  work_detection = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with work_detection:
    StateMachine.add('PICK MOTION 1', TF_MANIP(0.03, 0.015, 0.20, tf_pose="marker_frame"), \
        transitions={'succeeded':'PICK MOTION 2','retry':'PICK MOTION 1','aborted':'aborted'})
    StateMachine.add('PICK MOTION 2', CURRENT_MANIP(0.0, 0.0, -0.20), \
        transitions={'succeeded':'PICK MOTION 3','retry':'PICK MOTION 2','aborted':'aborted'})
    StateMachine.add('PICK MOTION 3', CURRENT_MANIP(0.0, 0.0, 0.20), \
        transitions={'succeeded':'INIT POSE 1','retry':'PICK MOTION 3','aborted':'aborted'})
    StateMachine.add('INIT POSE 1', INIT_POSE(), \
        transitions={'succeeded':'PLACE MOTION 1','retry':'INIT POSE 1','aborted':'aborted'})
    StateMachine.add('PLACE MOTION 1', NORMAL_MANIP(0.415, -0.320, 0.415), \
        transitions={'succeeded':'PLACE MOTION 2','retry':'PLACE MOTION 1','aborted':'aborted'})
    StateMachine.add('PLACE MOTION 2', CURRENT_MANIP(0.0, 0.0, -0.10), \
        transitions={'succeeded':'PLACE MOTION 3','retry':'PLACE MOTION 2','aborted':'aborted'})
    StateMachine.add('PLACE MOTION 3', CURRENT_MANIP(0.0, 0.0, 0.05), \
        transitions={'succeeded':'INIT POSE 2','retry':'PLACE MOTION 3','aborted':'aborted'})
    StateMachine.add('INIT POSE 2', INIT_POSE(), \
        transitions={'succeeded':'succeeded','retry':'INIT POSE 2','aborted':'aborted'})



##################################################################
#　　　　　　　　　　 人検知後の停止動作          　　           #
##################################################################
  stop_motion = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with stop_motion:
    StateMachine.add('WAIT', smach_ros.MonitorState("/stop_judge", String, resumemove_cb), \
      transitions={'invalid':'MOVE WAIT', 'valid':'WAIT', 'preempted':'WAIT'})
    StateMachine.add('MOVE WAIT', WAIT_TIME(2.0), \
      transitions={'succeeded':'INIT POSE','aborted':'aborted','preempted':'preempted'})
#初期姿勢
    StateMachine.add('INIT POSE', INIT_POSE(), \
      transitions={'succeeded':'succeeded','retry':'INIT POSE','aborted':'aborted','preempted':'preempted'})



##################################################################
#　　　　　　　　　　　 待機中動作               　　           #
##################################################################
  wait_command = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with wait_command:
    StateMachine.add('WAIT 1', WAIT_TIME(5), \
        transitions={'succeeded':'AREA SET','aborted':'aborted'})
#周辺環境計測
    StateMachine.add('AREA SET',  AREA_SET(set_data), \
        transitions={'succeeded':'DISCHARGE DETECT','retry':'AREA SET','aborted':'aborted'})
    
#排出位置検出
#10/05変更
    StateMachine.add('DISCHARGE DETECT',  DISCHARGE_DETECT(discharge_position_list), \
        transitions={'succeeded':'WORK DETECTION 1','retry':'DISCHARGE DETECT','aborted':'aborted'})
    
#ワーク検知１    
    StateMachine.add('WORK DETECTION 1', WORK_DETECT(task_command_id, work_type_id, set_area), \
        transitions={'succeeded':'BROADCAST 1','retry':'WORK DETECTION 1','aborted':'aborted'})
    
#ワーク検知２   
    StateMachine.add('BROADCAST 1', TF_BROADCAST(), \
        transitions={'succeeded':'succeeded','retry':'BROADCAST 1','aborted':'aborted'})
    


##################################################################
#　　　　　　　　　　　 実行中動作               　　           #
##################################################################
  running = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with running:
    StateMachine.add('TASK PREPARATION', wait_command, \
        transitions={'succeeded':'TASK IN PROGRESS','preempted':'preempted','aborted':'aborted'})
#作業実行中
    StateMachine.add('TASK IN PROGRESS', work_detection, \
        transitions={'succeeded':'RESULT RECEIVE','preempted':'preempted','aborted':'TASK PREPARATION'})
#作業結果の送信
    StateMachine.add('RESULT RECEIVE', RESULT_RECEIVE(), \
        transitions={'succeeded':'succeeded','aborted':'aborted'})
#人検知
    StateMachine.add('STOP OPERATION', stop_motion, \
      transitions={'succeeded':'TASK IN PROGRESS','aborted':'aborted','preempted':'preempted'})

#待機中動作
#状態遷移先の修正（retryを消去）10/04
#"WAIT"STATE  追加 1/4
  preparation = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with preparation:  
    StateMachine.add('SYSTEM CALL',  SYSTEM_CALL(), \
        transitions={'succeeded':'WAIT','aborted':'aborted'})
    StateMachine.add('WAIT', WAIT_TIME(2), \
        transitions={'succeeded':'succeeded','aborted':'aborted'})



##################################################################
#　　　　　　　　　　　 並列処理内容　                          #
##################################################################
  manip_concurrence = Concurrence(outcomes=['succeeded','aborted','motion_stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=child_manip_cb,
                                        outcome_cb=out_manip_cb)
  with manip_concurrence:
    Concurrence.add('RUNNING', running)
    Concurrence.add('HUMAN DETECT', smach_ros.MonitorState('/intrusion_result', PerEnvDetectResult, humandetect_cb))

##################################################################
#　　　　　　　　　　　 メイン処理内容                           #
##################################################################

  module_play = StateMachine(outcomes=['succeeded','aborted','preempted'])
  with module_play:
    #初期化動作
    StateMachine.add('INITIALIZATION', INIT_POSE(), \
        transitions={'succeeded':'PREPARATION','retry':'INITIALIZATION','aborted':'aborted'})
    #待機中
    StateMachine.add('PREPARATION',  preparation, \
        transitions={'succeeded':'RUNNING','preempted':'preempted','aborted':'aborted'})
    #実行中
    StateMachine.add('RUNNING', running, \
        transitions={'succeeded':'CLOSE TASK','aborted':'PREPARATION','preempted':'preempted'})
    #終了処理中
    StateMachine.add('CLOSE TASK', INIT_POSE(), \
        transitions={'succeeded':'succeeded','retry':'CLOSE TASK','aborted':'aborted','preempted':'preempted'})

 #smach_viwerによって状態遷移を可視化する
  sis = smach_ros.IntrospectionServer('server_name', module_play, 'Human Collaboration module Start Point!')
  sis.start()

#smachの実行
  module_play.execute()
  sis.stop()
