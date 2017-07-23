#!/usr/bin/env python
# encoding:utf8
from xm_smach.target_gpsr import target
import rospy
import tf
import actionlib
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
        

# state decide the smach of gpsr ,it is used for choosing the meta-smach due to the order of speech
class GetTask(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                    outcomes=['succeeded','aborted','go','find','follow','pick','talk','place','tell','error'],
                    input_keys =['action','current_task_in','task_num','goal'],
                    output_keys =['sentences','curent_task_out'])
        
    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task_in
            task_num = userdata.task_num
            goal = userdata.goal
            getattr(userdata, 'sentences')
            getattr(userdata, 'current_task_out')
        except:
            rospy.logerr('No param specified')
            return 'error'
        current_task +=1
        userdata.current_task_out = current_task
        # tasks have been executed successfully
        if current_task ==  task_num:
            return 'succeeded'
        current_action =  action[current_task]
        if current_action == 'go':
            return 'go'
        elif current_action == 'find':
            return 'find'
        elif current_action == 'follow':
            return 'follow'
        elif current_action == 'pick':
            return 'grasp'
        elif current_action == 'talk':
            return 'talk'
        elif current_action == 'place':
            return 'place'
        elif current_action == 'tell':
             userdata.sentences =  goal[current_task]
            return 'tell'
        else:
            # current_task_out -1
            userdata.current_task_out = userdata.current_task_in
            return 'aborted'
  
#   function as the name of state -_-
class PersonOrPosition(State):### switch the goal among person , speaker, thing, position
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error','person','position','thing'],
                        input_keys=['goal','current_task'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'goal')
            getattr(userdata, 'current_task')
        except:
            rospy.logerr('No param specified')
            return 'error'
        self.goal = userdata.goal[userdata.current_task]
        if self.goal=='person' or self.goal=='speaker':
            return 'person'
        else :
            return 'position'


# state used for find the people to follow 
# use for the position of people and xm
# xm must face to the people
# this state may last for 5 turns because the camera may not find the people fast
'''@param max_checks the number of messages to receive and evaluate. If cond_cb returns False for any
               of them, the state will finish with outcome 'invalid'. If cond_cb returns True for 
               all of them, the outcome will be 'valid' '''
# here we want to use the features of MonitorState mentioned above
class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm',])

        self.tf_listener = tf.TransformListener()
# if camrea find the people ,this state will return False
# else, if after 5 times camera still cannot find people, will return True
    def people_cb(self,userdata,msg):
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified')
            rospy.logerr('No error state use ,so I raise the KeyError')
            raise KeyError
        if msg is not None:
            self.tmp_pos = msg.position
            # rospy.logwarn(self.tmp_pos)
            self.tmp_pos.header.frame_id = 'camera_link'
            self.tmp_pos.header.stamp = rospy.Time()
            self.tf_listener.waitForTransform('map',self.tmp_pos.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
            # people_bf is the position of person in map
            people_bf = self.tf_listener.transformPoint('map', self.tmp_pos)
            # the below snippet is no longer used , because we can simple the interface of NavStack state
            # self.tf_listener.waitForTransform('base_footprint',self.tmp_pos.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
            # self.tmp_pos = self.tf_listener.transformPoint('base_footprint', self.tmp_pos)
            # angle = atan2(self.tmp_pos.point.y,self.tmp_pos.point.x)
            # self.tmp_pos.point.x -= 0.6*cos(angle)##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
            # self.tmp_pos.point.y -= 0.6*sin(angle)
            # self.tf_listener.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(60.0))
            # people_af is the position of xm in the map
            # people_af = self.tf_listener.transformPoint('map', self.tmp_pos)
            rospy.logwarn(people_bf)
            userdata.pos_xm = people_bf
            
            return False
        else:
            return True


 
# simple state used for get meaning from the speech node
# command mean invoke the speech node to return the meaning from the order
class GetMeaning(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['goal','action','task_num'])
                        
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
                
    def execute(self,userdata,response):
        try:
            getattr(userdata, 'goal')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
        except:
            rospy.logerr('No param specified')
            return 'error'

        response =self.speech_client.call(command =1)
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = responce.action
            userdata.goal= response.target
            print(userdata.task_num)
            print(userdata.action)
            print(userdata.goal)
            return 'succeeded'
        else:
            return 'aborted'
# this state used for dealing the data that from the /follow topic
# which input the pos_xm and output the Point type
class FollowData(State):
    def __init__(self):
        State.__init__(self,outcome=['succeeded','aborted','error'],
                            input_keys =['pos_xm']
                            output_keys =['point'])
        self.tf_listener = tf.TransformListener()

    def execute(self,userdata):
        try:
            getattr(userdata,'pos_xm')
            getattr(userdata,'point')
        except:
            rospy.logerr('No params specified')
            return 'error'

        # people_bf is the position of person in map
        try:
            self.tf_listener.waitForTransform('map',userdata.pos_xm.header.frame_id,rospy.Time(0),rospy.Duration(60.0)) 
            userdata.pos_xm = self.tf_listener.transformPoint('map', userdata.pos_xm)
        
            self.tf_listener.waitForTransform('base_footprint',.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
            self.tmp_pos = self.tf_listener.transformPoint('base_footprint', userdata.pos_xm)
            angle = atan2(self.tmp_pos.point.y,self.tmp_pos.point.x)
        except:
            rospy.logerr('Tf throw the error')
            return 'aborted'
        userdata.point =Point()
        userdata.point.x =0.5
        userdata.point.y =0.0
        userdata.point.z =angle
        return 'succeeded'

# this state is quite foolish ,so if you have good idea, please replace the below snippet
class GetGoal(State):
    def __init__(self):
        State.__init__(self,outcome =['succeeded','aborted','error'],
                            input_keys =['goal','current_task'],
                            output_keys =['current_goal'])
    def execute(self,userdata):
        try:
            getattr(userdata,'goal')
            getattr(userdata,'current_task')
            getattr(userdata,'current_goal')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        self.current_goal = userdata.goal[userdata.current_task]
        self.current_goal = target[self.current_goal]['pos']
        if self.current_goal is not None:
            userdata.current_goal =self.current_goal
            return 'succeeded'
        else:
            return 'aborted'

# I donnot want to say anything ....
class GetName(State):
    def __init__(self):
        State.__init__(self,outcome =['succeeded','aborted','error'],
                        input_keys =['goal','current_task'],
                        output_keys =['name'])
    
    def execute(self, userdata):
        try:
            getattr(userdata,'goal')
            getattr(userdata,'current_task')
            getattr(userdata,'name')
        except :
            rospy.logerr('No params specified')
            return 'error'
        else:
            name = userdata.goal[userdata.curent_task]
            if name is not None:
                userdata.name =name 
                return 'succeeded'
            else:
                return 'aborted'