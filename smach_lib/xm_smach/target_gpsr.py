#!/usr/bin/env python 
#encoding:utf8 

"""

	it is a stuff list for GPSR 
	it contains the pose of all the stuff
        
"""
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose


# mode = 1 在桌子上
# mode = 2 在架子上
target={
'speaker': {'pos': Pose(Point(1.358, -0.241, 0.000),Quaternion(0.000, 0.000, 0.068, 0.997)), 'mode': 1 },
'Gray': {'pos': Pose(), 'mode': 1 },
'David': {'pos': Pose(), 'mode': 1 },
'Daniel': {'pos': Pose(), 'mode': 1 },
'Jack': {'pos': Pose(), 'mode': 1 },
'Jenny': {'pos': Pose(), 'mode': 1 },
'Michael': {'pos': Pose(), 'mode': 1 },
'Lucy': {'pos': Pose(), 'mode': 1 },
'Peter': {'pos': Pose(), 'mode': 1 },
'Tom': {'pos': Pose(), 'mode': 1 },
'Jordan': {'pos': Pose(), 'mode': 1 },
'kitchen': {'pos': Pose(Point(4.881, 4.595, 0.000),Quaternion(0.000, 0.000, 0.730, 0.683)), 'mode': 1 },
'livingroom': {'pos': Pose(Point(2.336, -1.790, 0.000),Quaternion(0.000, 0.000, 0.005, 1.000)), 'mode': 1 },##########gai
'bedroom': {'pos': Pose(Point(0.000, 0.000, 0.000),Quaternion(0.000, 0.000, 0.006, 1.000)), 'mode': 1 },########gai
'init_pose':{'pos': Pose(Point(9.122, 1.874, 0.000),Quaternion(0.000, 0.000, -0.048, 0.999)), 'mode': 1 },
'dining-room': {'pos': Pose(Point(6.927, 5.776, 0.000),Quaternion(0.000, 0.000, 0.053, 0.999)), 'mode': 1 },
'cooking-table': {'pos': Pose(Point(3.058, 5.605, 0.000),Quaternion(0.000, 0.000, 0.707, 0.708)), 'mode': 1 },
'TV-table': {'pos': Pose(Point(4.138, 2.179, 0.000),Quaternion(0.000, 0.000, -0.639, 0.770)), 'mode': 1 },
'book-cabinet': {'pos': Pose(Point(10.200, 1.910, 0.000),Quaternion(0.000, 0.000, 0.724, 0.690)), 'mode': 1 },
'dining-table': {'pos': Pose(Point(9.470, 3.939, 0.000),Quaternion(0.000, 0.000, 0.681, 0.732)), 'mode': 1 },  
'sprite': {'pos': Pose(), 'mode': 1 },
'red-bull': {'pos': Pose(), 'mode': 1 },
'milk': {'pos': Pose(), 'mode': 1 },
'tea': {'pos': Pose(), 'mode': 1 },
'juice': {'pos': Pose(), 'mode': 1 },
'coffee': {'pos': Pose(), 'mode': 1 },
'biscuit': {'pos': Pose(), 'mode': 1 },
'chips': {'pos': Pose(), 'mode': 1 },
'roll-paper': {'pos': Pose(), 'mode': 1 },
'toothpaste': {'pos': Pose(), 'mode': 1 },
}
