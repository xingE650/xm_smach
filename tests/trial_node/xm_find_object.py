#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import *
from xm_msgs.msg import *
class xm_find_object():
    def __init__(self):
        rospy.init_node('xm_find_object')
        service = rospy.Service('get_object',xm_ObjectDetect,self.callback)
        rospy.loginfo('get_object service is beginning')
        rospy.spin()
    
    def callback(self,req):
        res = xm_ObjectDetectResponse()
        object_ =xm_Object()
        if req.object_name is not None:
            object_.name = req.object_name
            object_.pos.header.frame_id ='base_footprint'
            object_.pos.point.x = 0.7
            object_.pos.point.y = 0.0
            object_.pos.point.z = 0.6
        res.object =object_
        rospy.loginfo('return the result ')
        return res

if __name__ =="__main__":
    xm_find_object()
