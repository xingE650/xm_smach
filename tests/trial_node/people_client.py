#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import *

class client_people():
    def __init__(self):
        rospy.init_node('people_client')
        client = rospy.ServiceProxy('people_reco',xm_People)
        req = xm_PeopleRequest()
        req.command =0
        res =client.call(req)
        list_hehe = res.pos_person
        print list_hehe.pop()
        rospy.logerr('fuck you')
        print list_hehe.pop()
        rospy.logerr('fuck you')  
        print list_hehe.pop()
        rospy.logerr('fuck you')
        
        
    
if __name__=="__main__":
    client_people()