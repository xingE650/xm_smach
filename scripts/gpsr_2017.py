#!/usr/bin/env python
# encoding:utf8
# you need not to import the rospy module ,but import it here you can enjoy the vscode Tab fun -_-
# should use my modules as much as possible ^_^
from rospy  
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.target_gpsr import target
from geometry_msgs.msg import *
import math
class Gpsr():
    def __init__(self):
        rospy.init_node('GpsrSmach')
        rospy.on_shutdown(self.shutdown)
        self.sm_EnterRoom = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            self.m_EnterRoom.userdata.nav_pose = True
            StateMachine.add('NAV_POSE',
                                AmdCmd(),
                                transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
                                remapping ={'nav_pose':'nav_ps'})

            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','error':'error'})

            self.sm_EnterRoom.userdata.rec = 2.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'NAV_1','error':'error'},
                                remapping ={'rec':'rec'})

            # navstack 1 people 0 place
            self.sm_EnterRoom.userdata.start_waypoint  = target['speaker']['pos']
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'SPEAK1','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            self.sm_EnterRoom.userdata.sentences = 'I arrived,please ask me a question'                    
            StateMachine.add('SPEAK1',
                                Speak(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping ={'sentences':'sentences'})   

        self.sm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['goal','current_task','pos_xm'])
        # find task
        with self.sm_Find:
            self.sm_Person = StateMachine(outcomes = ['succeeded','aborted','error']
                                    )
            # find person ,the statemachine can achieve data meet itself
            with self.sm_Person:
                self.sm_Person.userdata.rec = 2.0
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                    remapping ={'rec':'rec'})
                self.sm_Person.userdata.pos_xm  =PointStamped()
                StateMachine.add('GET_PEOPLE_POS',
                                    FindPeople().find_people_,
                                    transitions ={'invalid':'NAV_PEOPLE','valid':'SPEAK'},
                                    remapping = {'pos_xm':'pos_xm'}
                                    )
                # this state will use the userdata remapping in the last state
                StateMachine.add('NAV_PEOPLE',
                                    NavStack(),
                                    transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                    remapping = {'pos_xm':'pos_xm'})
                self.sm_Person.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping = {'sentences':'sentences'})
            
            self.sm_Pos = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys =['pos_xm'])
            with self.sm_Pos:
                StateMachine.add('NAV_POS',
                                    NavStack(),
                                    remapping ={'pos_xm':'pos_xm'},
                                    transitions ={'succeeded':'SPEAK','aborted':'NAV_POS','error':'error'})    
                self.sm_Pos.userdata.sentences = 'I find it'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'})
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','pos':'POS','error':'error'},
                                remapping = {'goal':'goal','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.sm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('POS',
                                self.sm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})

        #nav tasks  
        self.sm_Nav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['goal','current_task'])
        with self.sm_Nav:
            self.sm_Nav.userdata.pos_xm =Pose()
            StateMachine.add('GETGOAL',
                                GetGoal(),
                                remapping ={'goal':'goal','current_task':"current_task",'current_goal':'pos_xm'})
            StateMachine.add('NAV_GO',
                                NavStack(),
                                transitions ={'succeeded':'SPEAK','aborted':'NAV_GO','error':'error'},
                                remapping ={'pos_xm':'pos_xm'})
            self.sm_Nav.userdata.sentences = 'I arrive here'
            StateMachine.add('SPEAK',
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping ={'sentences':'sentences'})
        
        #follow task 
        # data itself meets
        self.sm_Follow = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_Follow:
            self.sm_Follow.userdata.pos_xm = PointStamped()
            StateMachine.add('WHERE_FOLLOW',
                                FindPeople().find_people_,
                                transitions ={'invalid':'FOLLOW_DATA','valid':'WHERE_FOLLOW'},
                                remapping {'pos_xm':'pos_xm'})
            #data dealed 
            self.sm_Follow.userdata.point = Point()
            StateMachine.add('FOLLOW_DATA',
                                FollowData(),
                                remapping ={'pos_xm':'pos_xm','point':'point'},
                                transitions ={'succeeded':'MOVE','aborted':'aborted','error':"error"})
            # add new state
            StateMachine.add('MOVE',
                                SimpleMove(),
                                transitions ={'succeeded':'succeeded','aborted':'succeeded','error':'error'},
                                remapping ={'point':'point'}
                                )
            
        self.sm_Talk = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Talk:
            StateMachine.add('SPEAK',
                                Anwser(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
        
        self.sm_Pick = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['goal','current_task'])
        with self.sm_Pick:
            self.sm_Pick.userdata.nav_ps =True
            StateMachine.add('NAV_POSE',
                                AmdCmd(),
                                transitions={'succeeded':'FIND_OBJECT','aborted':'NAV_POSE','error':'error'},
                                remapping ={'nav_pose':'nav_ps'})
            self.sm_Pick.userdata.name =''
            StateMachine.add('GETNAME',
                                GetName(),
                                remapping ={'goal':'goal','current_task':'current_task','name':'name'})
            self.sm_Pick.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'PICK','aborted':'aborted','error':'error'},
                                remapping ={'current_goal':'name','object_pos':'object_pos'})
            
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'PUT','aborted':'aborted','error':'error'},
                                remapping ={'pick_pos':'object_pos'})
            StateMachine.add('PUT',
                                ArmCmd(),
                                transitions ={'succeeded':'succeded','aborted':'aborted','error':'error'},
                                remapping ={'nav_ps':'nav_ps'})
        
        self.sm_Place = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Place:
            # place_ps please specified due to the scene
            self.userdata.place_ps = PointStamped()
            self.userdata.place_ps.header.frame_id ='base_footprint'
            self.userdata.place_ps.point.x =0.7
            self.userdata.place_ps.point.y =0.0
            self.userdata.place_ps.point.z =0.6           
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions ={'succeeded':'PUT','aborted':'PLACE','error':'error'},
                                remapping ={'place_ps':'place_ps'})
            self.userdata.nav_ps =True
            StateMachine.add('PUT',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'nav_ps':'nav_ps'})
        
        self.sm_GPSR =StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_GPSR:
            self.sm_GPSR.userdata.goal = list()
            self.sm_GPSR.userdata.action = list()
            self.sm_GPSR.userdata.task_num =0
            self.sm_GPSR.userdata.current_task =-1
            self.sm_GPSR.userdata.sentences ='I donnot know to say what'
            StateMachine.add('RECEIVE_TASKS',
                                GetMeaning(),
                                transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                                remapping ={'goal':'goal','action':'action','task_num':'task_num'}
                                )
            StateMachine.add('GET_NEXT_TASK',
                                GetTask(),
                                transitions ={'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'error':'error',
                                            'find':'FIND',
                                            'go':'GO',
                                            'follow':'FOLLOW',
                                            'pick':'PICK',
                                            'place':'PLACE',
                                            'talk':'TALK',
                                            'aborted':'aborted',
                                            'error':'error'},
                                remapping ={'goal':'goal',
                                            'action':'action',
                                            'current_task_in':'current_task',
                                            'task_num':'task_num',
                                            'sentences':'sentences',
                                            'current_task_out':'current_task'}
                                )
# this smach snippet has at least 2 bug:
# 1 the following snippet I directly copy from the old codes ,so the transitions may have problems
# and the remapping I donnot write
# 2.the error state I have not dealed
# 3.I have never tested the code  
# so today I go to fix the bugs >ToT< 2017/7/22
# haha but the bug is so easy compared to the whoiswho <^_^>
            StateMachine.add('GO',
                                self.sm_Nav,
                                remapping ={'goal':'goal','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            StateMachine.add('FIND',
                                self.sm_Find,
                                remapping ={'goal':'goal','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',
                                self.sm_Follow,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FOLLOW'})
            StateMachine.add('PICK',
                                self.sm_Pick,
                                remapping ={'goal':'goal','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK'})
            StateMachine.add('TALK',
                                self.sm_Talk,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
            StateMachine.add('PLACE',
                                self.sm_Place,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PLACE'})    
        self.smach_bool = False       
        out_1 = self.sm_EnterRoom.execute()
        out_2 = self.sm_GPSR.execute()    
        self.smach_bool = True 

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


if __name__ == "__main__":
    try:
        Gpsr()
    except KeyboardInterrupt；
        pass    

                