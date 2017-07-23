#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
import rospy
from smach import State, UserData
# ServiceState is used for req fixed without data through input_keys
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
# simple state for nav_stack 
# may usually use for whoiswho
# we may notice the very important detail:
# if you are nav to the front of person, the data must from the cv ,so it must be PointStamped,
# but if you are just nav to place, the waypoints usually specified, so it must be Pose
class NavStack(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['pos_xm','mode'])
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        # wait 60s for the action server to become available 
        self.nav_client.wait_for_server(rospy.Duration(60))

    def execute(self, userdata):
        # mode ==1 mean that xm is going the place of people, which require xm face the person
        try:
            getattr(userdata, 'pos_xm')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            # mode ==1 mean that xm need to nav to the front of people
            if userdata.mode ==1:
                print userdata.pos_xm
                return self.nav_people(userdata.pos_xm)
            # mode ==0 mean that xm is just nav to the place
            elif userdata.mode ==0:
                return self.nav_thing(userdata.pos_xm)
            else :
                return 'aborted'
# this function will deal the data from the cv ,which is the PointStamped()
    def nav_people(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id ='map'
        # this point is in the frame of robot 
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.6*cos(angle)
        person_y = person_y -0.6*sin(angle)
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header.frame_id  =pos_xm.header.frame_id
        qs.quaternion = self.q
        # we need to transform these data into the map frame
        self.tf_listener.transformPoint('map',pos_xm)
        # the angle should also transform to the map frame
        self.tf_listener.transformQuaternion('map',qs)
        goal.target_pose.pose = Pose(pos_xm.point,qs.quaternion)
        self.nav_client.send_goal(goal)
        nav_counter = 0
        # if nav stack is failed, we can plan again
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter<50:
            nav_counter +=1
            rospy.sleep(0.5)
        if self.nav_client.get_goal_status_text() == 'Goal reached.' :
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'

    def nav_thing(self,pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        self.nav_client.send_goal(goal)
        nav_counter = 0
        # if nav stack is failed, we can plan again
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter<50:
            nav_counter +=1
            rospy.sleep(0.5)
        if self.nav_client.get_goal_status_text() == 'Goal reached.' :
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'

#  simple state used for xm_speak ,we use the espeak ,but very un-lovely -_-
# if no sentences provided, this state should be skipped
class Speak(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['sentences'])
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try :
            self.string_ = str(userdata.sentences)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(self.string_)} , shell = True)
            except:
                return 'aborted'
            else:
                return 'succeeded'

# simple state used to invoke the /move service to justfy the xm position
# it contains all the /move service call state

class SimpleMove(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        input_keys=['point'])
        self.move_client = rospy.ServiceProxy('/move',xm_Move)
        self.move_client.wait_for_service(timeout= 25.0)

    def execute(self,userdata):
        move_value = xm_MoveRequest()
        try:
            move_value.position = userdata.point
        except:
            rospy.logerr('No param specified')
            return 'error'
        print str(move_value)
        result =self.move_client.call(move_value)
        
        if result.arrived ==True :
            return 'succeeded'
        else:
            return 'aborted'

# armcmd should be written into a unified state for 3 different poses
# nav_ps is just used for making the arm understand this action is nav_pose instead of error,
# so it must is specified as true
class ArmCmd(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=[ 'succeeded', 'aborted', 'error'],
                        io_keys=['mode','arm_ps'])
        self.xm_arm_client = rospy.ServiceProxy('xm_pick_or_place', xm_PickOrPlace)
        # make sure that the PointStamped used for arm pick_or_place is in the frame base_footprint
        self.tf_listener = tf.TransformListener()
        
    def execute(self, userdata):
        rospy.loginfo('Moving arm')
        try:
            getattr(userdata,'mode')
            getattr(userdata,'arm_ps')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            req = xm_PickOrPlaceRequest()
            req.action = userdata.mode
            self.arm_ps_ =PointStamped()
            if userdata.arm_ps.header.frame_id is not '':
                rospy.loginfo('frame checked')
                self.arm_ps_ = self.tf_listener.transformPoint('base_footprint',userdata.arm_ps)
            else:
                pass
            req.goal_position = self.arm_ps_
            res =self.xm_arm_client.call(req)
            if res.result ==True:
                return 'succeeded'
            else:
                return 'aborted'

# simple state use for delay some time
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"


# monitor state used for detect if the door is open and xm want to go to there ^_^
class DoorDetect():
    def __init__(self):
        self.door_detect_ = MonitorState('DoorState', Bool, self.door_cb,max_checks =1)

    def door_cb(self,userdata,msg):
        if msg.data == True:
            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
            req = EmptyRequest()
            res = clear_client.call(req)
            return False
        else:
            return True

class Anwser(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted'])
        self.anwser_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    
    def execute(self):
        res =self.anwser_client.call(command =0)
        if res.result ==True:
            return 'succeeded'
        else:
            return 'aborted'


# state use for object reco
# because in different task smach ,this state may make different tasks,so donnot 
# write this state class in the common_lib.py
class FindObject(State):
    def __init__(self):
        State.__init__(self,
                         outcomes=['succeeded','aborted','error'],
                         input_keys=['current_goal'],
                         output_keys =['object_pos'])
        self.xm_findobject = rospy.ServiceProxy('get_object', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        goal = Point()
        try:
            current_goal = userdata.current_goal
        except:
            rospy.logerr('No param specified')
            return 'error'
        for i in range(3):
            try:
                # find object by invoking the service(command=2)
                req = xm_ObjectDetectRequest()
                req.object_name = current_goal
                self.object_info = self.xm_findobject.call(req)
                
            except:
                return 'aborted'
        #   object_pos is PointStamped
        userdata.object_pos = self.object_info.object.pos
        # output_keys cannot be read     
        # print userdata.object_pos
        return 'succeeded'


