#!/usr/bin/env python
# encoding:utf8

# you need not to import the rospy module ,but import it here you can enjoy the vscode Tab fun -_-
# should use my modules as much as possible ^_^
# and the output_keys cannot be read
# the input_keys cannot be write
# this task is only 8 minutes so when xm return the room the cv should give me the position 
# unless the time must be beyond limit
# if userdata no instance , it must not be used
import rospy  
from xm_smach.common_lib import *
from xm_smach.whoiswho_lib import * 
from smach import State, StateMachine
from math import pi
from geometry_msgs.msg import *
# NavStack in this smach is always nav to place
# donnot specified the people_pos
class WhoisWho():
    def __init__(self):
        rospy.init_node('whoiswho')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)
        
        # add waypoints into the list
        self.waypoints=[]

        location= (Point(0.196, 0.695,0),  #找人数的点
                   Point(6.533,0.976, 0.000),   #出门抓东西的点
                   Point(5.718, 4.349,0))     #结束出门的点
        quaternions=[]

        euler_angles=[-3.085,-3.130,-3.076]
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        for i in range(3):
            self.waypoints.append(Pose(location[i], quaternions[i]))
        # the locate can specified by ourselves
        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            # arm go to the nav_pose in the srdf
            self.sm_EnterRoom.userdata.arm_mode =0
            self.sm_EnterRoom.userdata.arm_ps_1 = PointStamped()
            StateMachine.add('NAV_POSE',
                                ArmCmd(),
                                transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
                                remapping ={'mode':'arm_mode','arm_ps':'arm_ps_1'})
            # wait the door to open
            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'error'})
            # simple delay 2s
            self.sm_EnterRoom.userdata.rec = 2.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'NAV_1','error':'error'},
                                remapping ={'rec':'rec'})

            # navstack to room of crowds
            # waypoints are the list of Pose fit the data type need
            self.sm_EnterRoom.userdata.start_waypoint  = self.waypoints[0]
            self.sm_EnterRoom.userdata.nav_mode =0
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'SELF_INTRO','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint','mode':'nav_mode'})
            
            self.sm_EnterRoom.userdata.sentences_2 = 'I am robot xiaomeng, I will make you happy hahaha'
            StateMachine.add('SELF_INTRO',
                                Speak(),
                                remapping ={'sentences':'sentences_2'},
                                transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        
        
        self.sm_FaceDetect = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            output_keys = ['people_position'])
        with self.sm_FaceDetect:
            self.sm_FaceDetect.userdata.people_position =list()
            self.sm_FaceDetect.userdata.turn_point_1 = Point(0.0,0.0,pi/8)                   
            StateMachine.add('TURN_L',
                                SimpleMove(),
                                transitions={'succeeded':'SPEAK','error':'error'},
                                remapping ={'point':'turn_point_1'})
            self.sm_FaceDetect.userdata.sentences = 'please look at me'
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})
            # call face_reco service for get the people position which is a list
            self.sm_FaceDetect.userdata.command =0
            StateMachine.add('GET_POSITION',
                                FaceReco(),
                                remapping  ={'command':'command','position':'people_position'},
                                transitions ={'succeeded':'TURN_M','aborted':'GET_POSITION','error':'error'})
            
            self.sm_FaceDetect.userdata.turn_point_2 = Point(0.0,0.0,-pi/8)
            StateMachine.add('TURN_M',
                                SimpleMove(),
                                remapping ={'point':'turn_point_2'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})
            StateMachine.add('SPEAK_2',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'GET_POSITION_2','aborted':'aborted','error':'error'})
             
            StateMachine.add('GET_POSITION_2',
                                FaceReco(),
                                remapping  ={'command':'command','position':'people_position'},
                                transitions ={'succeeded':'TURN_R','aborted':'GET_POSITION_2','error':'error'})
            self.sm_FaceDetect.userdata.turn_point_3 = Point(0.0,0.0,-pi/4)
            StateMachine.add('TURN_R',
                                SimpleMove(),
                                transitions={'succeeded':'','aborted':'aborted','error':'error'},
                                remapping ={'point':'turn_point_3'})
        
        self.sm_GetTarget = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['target'])#the target is a string.....
        with self.sm_GetTarget:
            # because xm is nav to pose in the nav_pose 
            self.sm_GetTarget.userdata.nav_ps = self.waypoints[1]
            self.sm_GetTarget.userdata.nav_mode =0
            StateMachine.add('NAV_TARGET',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps','mode':'nav_mode'},
                                transitions ={'succeeded':'FIND_OBJECT','aborted':'NAV_TARGET','error':'error'})
            self.sm_GetTarget.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                remapping ={'current_goal':'target','object_pos':'object_pos'},
                                transitions ={'succeeded':'TALK','aborted':'FIND_OBJECT','error':'error'})
            self.sm_GetTarget.userdata.sentences = 'I find the target'
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'PICK','aborted':'TALK','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode_1','arm_ps':'object_pos'},
                                transitions ={'succeeded':'PUT','aborted':'PICK','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_2 = 0
            self.sm_GetTarget.userdata.arm_ps_2 = PointStamped()
            StateMachine.add('PUT',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode_2','arm_ps':'arm_ps_2'},
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
        # input  one position is a list
        # io_keys can wirte and read 
        # userdata can update real-time 
        self.sm_Remember = StateMachine(outcomes =['succeeded','aborted','error'],
                                        input_keys =['person_position'],
                                        output_keys =['name','target']
                                        )
        with self.sm_Remember:
            self.sm_Remember.userdata.nav_mode =1
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position','mode':'nav_mode'},
                                transitions ={'succeeded':'TALK','aborted':'NAV_GO','error':'error'}
                                )
            self.sm_Remember.userdata.sentences = "what is your name and what do you want"
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'GET_BOTH','aborted':'TALK','error':'error'})
            StateMachine.add('GET_BOTH',
                                NameAndThing(),
                                remapping ={'name':'name','target':'target'},
                                transitions ={'succeeded':'FACE_REM','aborted':'GET_BOTH','error':'error'})
            self.sm_Remember.userdata.command =1
            StateMachine.add('FACE_REM',
                                FaceReco(),
                                remapping ={'command':'command'},
                                transitions ={'succeeded':'succeeded','aborted':'FACE_REM','error':'error'}
                                )
        #return room and find the person who need the thing
        # self.sm_ReturnRoom =StateMachine(outcomes =['succeeded','aborted','error'],
        #                                     output_keys =['people_position'])
        
        # with self.sm_ReturnRoom:
        #     self.sm_ReturnRoom.userdata.nav_ps = self.waypoints[0]        
        #     StateMachine.add('NAV_ROOM',
        #                         NavStack(),
        #                         remapping ={'pos_xm':'nav_ps'},
        #                         transitions ={'succeeded':'TURN_L','aborted':'NAV_ROOM','error':'error'})
        #     self.sm_ReturnRoom.userdata.turn_point_1  = Point(0.0,0.0,pi/8)
        #     StateMachine.add('TURN_L',
        #                         SimpleMove(),
        #                         remapping ={'point':'turn_point_1'},
        #                         transitions ={'succeeded':'TALK_1','aborted':'TURN_L','error':'error'})
        #     self.sm_ReturnRoom.userdata.sentences ='please look at me'
        #     StateMachine.add('TALK_1',
        #                         Speak(),
        #                         remapping ={'sentences':'sentences'},
        #                         transitions ={'succeeded':'GET_POS_1','aborted':'TALK_1','error':})
        #     self.sm_ReturnRoom.userdata.command =0
        #     StateMachine.add('GET_POS_1',
        #                         FaceReco(),
        #                         remapping ={'command':'command','position':"people_position"},
        #                         transitions ={'succeeded':'TURN_R','aborted':'GET_POS_1','error':'error'})
        #     self.sm_ReturnRoom.userdata.turn_point_2 = Point(0.0,0.0,-pi/8)
        #     StateMachine.add('TURN_R',
        #                         SimpleMove(),
        #                         remapping ={'point':'turn_point_2'},
        #                         transitions ={'succeeded':'TALK_2','aborted':'TURN_R','error':'error'})

        #     StateMachine.add('TALK_2',
        #                         Speak(),
        #                         remapping ={'sentences':"sentences"},
        #                         transitions ={'succeeded':'GET_POS_2','aborted':'TALK_2','error':'error'})
        #     StateMachine.add('GET_POS_2',
        #                         FaceReco(),
        #                         remapping ={'command':'command','position':'people_position'},
        #                         transitions ={'succeeded':'TURN_M','aborted':'GET_POS_2','error':'error'})
        #     self.sm_ReturnRoom.userdata.turn_point_3= Point()
        #     StateMachine.add('TURN_M',
        #                         SimpleMove(),
        #                         remapping ={'succeeded':'succeeded','aborted','error':'error'})
        # give back the targets 
        # self.sm_GiveBack= StateMachine(outcomes =['succeeded','aborted','error'],
        #                                 input_keys =['position_person'])
        # this state is the simplest state if the cv can give me the position of people with specified name
        # with self.sm_GiveBack:
        #     StateMachine.add('NAV_POS',
        #                         NavStack(),
        #                         remapping ={'pos_xm':'position_person'},
        #                         transitions ={'succeeded':'PLACE','aborted':'NAV_POS','error':'error'})
        #     # this pose should be specified to make the people get it
        #     self.sm_GiveBack.userdata.place_ps = PointStamped()
        #     self.sm_GiveBack.userdata.place_ps.header.frame_id ='base_footprint'
        #     self.sm_GiveBack.userdata.place_ps.point.x =0.7
        #     self.sm_GiveBack.userdata.place_ps.point.y =0.0
        #     self.sm_GiveBack.userdata.place_ps.point.z =0.3
        #     StateMachine.add('PLACE',
        #                         ArmCmd(),
        #                         remapping ={'place_ps':'place_ps'},
        #                         transitions ={'succeeded':'TALK','aborted':'PLACE','error':'error'})
        #     self.sm_GiveBack.userdata.sentences ="I find the thing you need, please take it"
        #     StateMachine.add('TALK',
        #                         Speak(),
        #                         remapping ={'sentences':'sentences'},
        #                         transitions ={'succeeded':'succeeded':'aborted':'aborted','error':'error'})
        self.sm_GiveBack = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['name_id'])# the name is a string
        with self.sm_GiveBack:
            self.sm_GiveBack.userdata.nav_ps = self.waypoints[0]   
            self.sm_GiveBack.userdata.nav_mode_1 =0     
            StateMachine.add('NAV_ROOM',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps','mode':'nav_mode_1'},
                                transitions ={'succeeded':'FACE_RECO','aborted':'NAV_ROOM','error':'error'})
            self.sm_GiveBack.userdata.command =2
            self.sm_GiveBack.userdata.person_position =PointStamped()
            StateMachine.add('FACE_RECO',
                                FaceReco(),
                                remapping ={'position':'person_position','command':"command",'name_id':'name_id'},
                                transitions ={'succeeded':'NAV_GO','aborted':'FACE_RECO','error':'error'}
                                )
            #please pay attention! 
            self.sm_GiveBack.userdata.nav_mode_2 =1     
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position','mode':'nav_mode_2'},
                                transitions ={'succeeded':'TALK_1','aborted':'NAV_GO','error':'error'})
            self.sm_GiveBack.userdata.sentences_1 = 'I get the thing you want , plase stretched out your hand'
            StateMachine.add('TALK_1',
                                Speak(),
                                remapping ={'sentences':"sentences_1"},
                                transitions ={'succeeded':'PLACE','aborted':"PLACE",'error':'error'})

            # this pose should be specified to make the people get it
            self.sm_GiveBack.userdata.arm_mode =2
            self.sm_GiveBack.userdata.place_ps = PointStamped()
            self.sm_GiveBack.userdata.place_ps.header.frame_id ='base_footprint'
            self.sm_GiveBack.userdata.place_ps.point.x =0.7
            self.sm_GiveBack.userdata.place_ps.point.y =0.0
            self.sm_GiveBack.userdata.place_ps.point.z =0.3
            StateMachine.add('PLACE',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode','arm_ps':'place_ps'},
                                transitions ={'succeeded':'TALK_2','aborted':'PLACE','error':'error'})
            self.sm_GiveBack.userdata.sentences_2 = 'I get the thing you want, am I?'
            StateMachine.add('TALK_2',
                            Speak(),
                            remapping ={'sentences':'sentences_2'},
                            transitions ={'succeeded':'succeeded','aborted':"aborted",'error':'error'})
            


        self.sm_EndTask = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_EndTask:
            self.sm_EndTask.userdata.nav_ps = self.waypoints[2]        
            StateMachine.add('NAV_BYEBYE',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'succeeded','aborted':'NAV_BYEBYE','error':'error'})

        self.WHOISWHO = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.WHOISWHO:
            # we can generate some state which contant the list_value
            StateMachine.add('ENTERROOM',
                                self.sm_EnterRoom,
                                transitions ={'succeeded':'FACEDETECT','aborted':'aborted','error':'error'})
            self.WHOISWHO.userdata.people_position =list()
            StateMachine.add('FACEDETECT',
                                self.sm_FaceDetect,
                                remapping ={'people_position':'people_position'},
                                transitions = {'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            self.WHOISWHO.userdata.person_position = PointStamped()
            StateMachine.add('GETPERSON',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'REMEMBER','aborted':"GETTARGET",'error':'error'}
                                )
            # the cv will remember the face with the increasing ID
            self.WHOISWHO.userdata.name =''
            self.WHOISWHO.userdata.target= ''
            self.WHOISWHO.userdata.name_list =list()
            self.WHOISWHO.userdata.target_list =list()
            StateMachine.add('REMEMBER',
                                self.sm_Remember,
                                remapping ={'person_position':'person_position','name':'name','target':'target'},
                                transitions ={'succeeded':'NAMEINLIST','aborted':'aborted','error':'error'}
                                )
            #this state use for joining the name and the target into the name_list and target_list 
            StateMachine.add('NAMEINLIST',
                                NameInList(),
                                remapping ={'name':'name','target':'target','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            # so far here ,the tasks of remember is completed , the rest is the tasks to return the target to the people
            # we should take out the name and the matched target for xm to grasp and give back
            # in fact, we will pop the name and target out the list,so the name_id is gradually reduced
            StateMachine.add('GETTARGET',
                                GetValue(),
                                remapping ={'element_list':'target_list','element':'target'},
                                transitions ={'succeeded':'CATCHTARGET','aborted':'ENDTASK','error':'error'})
            StateMachine.add('CATCHTARGET',
                                self.sm_GetTarget,
                                transitions= {'succeeded':'GETID','aborted':'GETID','error':'error'},
                                remapping = {'target':'target'})
            self.WHOISWHO.userdata.name_id = 0   
            StateMachine.add('GETID',
                                GetId(),
                                remapping ={'output_id':'name_id','input_list':'target_list'},
                                transitions ={'succeeded':'GIVEBACK','aborted':'ENDTASK','error':'error'}
                                )
            StateMachine.add('GIVEBACK',
                                self.sm_GiveBack,
                                remapping ={'name_id':'name_id'},
                                transitions ={'succeeded':'GETTARGET','aborted':'aborted','error':'error'}
                                )
            StateMachine.add('ENDTASK',
                                self.sm_EndTask,
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            
            self.WHOISWHO.execute()
            self.smach_bool =True
    
    def shutdown(self):
        if self.smach_bool ==False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')


if __name__ =="__main__":
    WhoisWho()          