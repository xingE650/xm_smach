#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import*

class xm_name_get():
    def __init__(self):
        rospy.init_node('xm_name_get')
        rospy.on_shutdown(self.shutdown)
        service = rospy.Service('name_get',xm_SpeechName,self.callback)
        rospy.loginfo('name_get service is beginning')
        rospy.spin()

    def shutdown(self):
        rospy.logerr('fuck you')


    def callback(self,req):
        res = xm_SpeechNameResponse()
        if req is not None:
            if req.command ==0:
                res.result =True
                res.name ='hehe'
                res.target ='balala'
            else:
                raise KeyboardInterrupt
        else:
            raise KeyboardInterrupt
        return res

if __name__ =="__main__":
    xm_name_get()