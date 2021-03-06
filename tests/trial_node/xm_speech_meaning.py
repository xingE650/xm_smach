#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import *

class xm_speech_meaning():
    def __init__(self):
        rospy.init_node('xm_speech_meaning')
        service =rospy.Service('xm_speech_meaning',xm_Speech_meaning,self.callback)
        rospy.loginfo('xm_speech_meaning service is beginning')
        rospy.spin()
    def callback(self,req):
        res = xm_Speech_meaningResponse()
        # question and answer mode 
        if req.command ==0:
            rospy.loginfo('begin question and answer')
            rospy.loginfo('e, enjoy yourselves')
        elif req.command ==1:
            # here we will use the specified num and action and goal
            # go to bedroom , catch the soap and return to me
            rospy.loginfo('get the gpsr command')
            res.num = 3
            res.action =['nav','pick','nav']
            res.target =['bedroom','soap','speaker']
        return res

if __name__ == "__main__":
    xm_speech_meaning()