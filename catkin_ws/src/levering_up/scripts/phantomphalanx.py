#!/usr/bin/env python

import os
import numpy as np
import rospy
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

setCart = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setSpeed = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)

setSpeed(100,30)
setJoint(0,0,0,0,90,90)

os.system("rosservice call -- /wsg_50_driver/homing")
os.system("rosservice call -- /wsg_50_driver/set_force 10")

sart_pose=np.array([398, -25, 288, 0, 0.7071, 0.7071, 0])

setCart(sart_pose[0],sart_pose[1],sart_pose[2],sart_pose[3],sart_pose[4],sart_pose[5],sart_pose[6])
# rospy.sleep(2)

os.system("rosservice call -- /wsg_50_driver/move 40 20")

os.system("rosservice call -- /wsg_50_driver/move 68 20")

rospy.sleep(2)
setSpeed(50,20)
setCart(398, -55, 288, 0, 0.7071, 0.7071, 0)


#os.system("rosservice call -- /wsg_50_driver/move 40 20")
setCart(398, 15, 288, 0, 0.7071, 0.7071, 0)
rospy.sleep(3)

setCart(398, -15, 288, 0, 0.7071, 0.7071, 0)
#os.system("rosservice call -- /wsg_50_driver/move 50 20")


setSpeed(200,80)
setJoint(-2.16, 30.02, 14.92, 0, 45.06, 330)

os.system("rosservice call -- robot1_SetZone 3")

setSpeed(50,20)
setJoint(-2.16, 30.02, 14.92, 0, 45.06, 270)

setSpeed(500,200)
setJoint(-2.16, 30.02, 14.92, 0, 45.06, 90)

setSpeed(50,20)
setJoint(-2.16, 30.02, 14.92, 0, 45.06, 30)

os.system("rosservice call -- robot1_SetZone 0")
setSpeed(100,30)
setJoint(-2.16, 30.02, 14.92, 0, 45.06, 90)

setCart(398, -35, 288, 0, 0.7071, 0.7071, 0)
