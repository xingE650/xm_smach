#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import State, UserData
from smach_ros import SimpleActionState, MonitorState,ServiceState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from xm_msgs.srv import *
import subprocess
# the vital state used in whoiswho smach 
# the face_reco state 
# check_people use when the first see the crowds
# you may want to change this snippet 
#  0 get_position 1 remember 2 recognize
# the position is a list of PointStamped()
# we cannot do any job for reading the output_keys
# io_keys can avoid the problem of reading or writing only of keys
class FaceReco(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys = ['command','name_id'],
                        io_keys = ['position'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('people_reco',xm_People)
        self.send_id  =0
    
    def execute(self,userdata):
        rospy.loginfo('face recongize begin')
        req  = xm_PeopleRequest()
        try:
            command = userdata.command
        except:
            rospy.logerr('No param specified')
            return 'error'
        req.command= command
        # paramm checked
        # 0 get_position return all people position
        if command ==0:
            pass
        # 1 remember need send_id but no need of receive_id
        elif command ==1:
            req.send_id = self.send_id
            self.send_id+=1
        # 2 recognize need receive_id but no need of send_id
        elif command ==2:
            try:
                name_id =userdata.name_id
            except:
                rospy.logerr('Name_id is not specified')
                return 'error'
            else:
                req.send_id = name_id
        try:
            res= self.face_reco_client.call(req)
        except:
            return 'aborted'
        else:
            if command ==0:
                userdata.position.extend(res.pos_person) 
            elif command ==2:
                userdata.position =res.pos_person.pop()
            return 'succeeded'
        
# state return the name and the thing heard , use for remember face and the information
# serivice name 'name_get'
# serivice type xm_SpeechName
# mode ==false mean that is the 2nd time reco
class NameAndThing(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['name','target'])
        self.client = rospy.ServiceProxy('name_get',xm_SpeechName)
        self.mode  =True
    def execute(self,userdata):
        try:
            rospy.loginfo("I donnot know to say what here")
        except:
            rospy.logerr('ros is error, please buy a new computer')
            return 'error'
        if self.mode ==False:
            self.string_ ='I cannot understand what you are saying ,can you speak again? thank you'
            subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(self.string_)} , shell = True)
        res =self.client.call(command =0)
        if res.result ==False:
            self.mode = False
            return 'aborted'
        else :
            userdata.name = res.name
            userdata.target = res.target
            self.string_ ='I know that you are '+str(res.name)+'and you want '+str(res.target)
            subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(self.string_)} , shell = True)
            return 'succeeded'

# simple state check the people_position if empty
class CheckEmpty(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['list'])
    
    def execute(self,userdata):
        try:
            length = len(userdata.list)
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            if length ==0 :
                return 'aborted'
            else:
                return 'succeeded'

# this state is used for generating the sentences that shows xm understands the speaker
class CheckName(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted','error'],
                        input_keys =['name_list','name_id','name'])
    def execute(self, userdata):
        try:
            getattr(userdata,'name_list')
            getattr(userdata,'name_id')
            getattr(userdata,'name')
        except:
            rospy.logerr('No params specified')
            return 'error'
        if userdata.name ==userdata.name_list[userdata.name_id]:
            return 'succeeded'
        else:
            return 'aborted'

# this state used for extract the position of each person
class GetValue(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list'],
                        output_keys =['element'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'


# this state used for joining the name and target in a list
class NameInList(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys=['name','target'],
                        output_keys=['name_list','target_list'])
        self.name_list = list()
        self.target_list = list()
    def execute(self, userdata):
        try:
            getattr(userdata,'name')
            getattr(userdata,'target')

        except :
            rospy.logerr('No params specified')
            return 'error'
        else:
            self.name_list.append(userdata.name)
            self.target_list.append(userdata.target)
            userdata.name_list =self.name_list
            userdata.target_list =self.target_list
            return 'succeeded'
        # oh hehe 
        if False:
            return 'aborted'

# this state may look quitly silly , so if you find good way please rewrite 
class GetId(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['input_list'],
                        output_keys =['output_id'])
        self.input_list = list()
        self.mode = True
    
    def execute(self,userdata):
        try:
            getattr(userdata,'input_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            if self.mode ==True:
                self.mode =False
                self.input_list = userdata.input_list
            else:
                pass
            output_id = len(self.input_list)-1
            try:
                self.input_list.pop()
            except :
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'