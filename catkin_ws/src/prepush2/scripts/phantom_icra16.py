#!/usr/bin/env python

#roslaunch prepush2 start_robot_gripper_phantom.launch

import os
import signal
import numpy as np
import rospy
import subprocess 
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from wsg_50_common.srv import *
from std_srvs.srv  import Empty 
import errno
import pdb
import thread

#Define call methods
setCart = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getCart = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoints = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
getJoints=rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
setSpeed = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)

homeGripper = rospy.ServiceProxy('/wsg_50_driver/homing', Empty)
moveGripper = rospy.ServiceProxy('/wsg_50_driver/move', Move)
setGripperForce = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
graspGripper = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
releaseGripper = rospy.ServiceProxy('/wsg_50_driver/release', Move)

#~ def opencloseGripper(open_dist,close_dist,gripperSpeed,iter_num):
    #~ i=0
    #~ if i <=iter_num:
        #~ print 'here'
        #~ moveGripper(open_dist,gripperSpeed)
        #~ moveGripper(close_dist,gripperSpeed)
        #~ 
    #~ return True

#~ toolZ=245.0
toolZ=0.0
# Tool frame is set to fingertip of the long gripper finger
# Work object is set to external finger
#~ 
# 1. Start with gripper closed(phantom finger close to other finger)
#1.1 Move gripper 100 mm away from external finger in X direction
setSpeed(80,40)
setJoints(-44.92,-12.88,35.25,110.89,49.09,-120.24)
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
homeGripper()
# -------------------------------------------------------
# -------------------------------------------------------
# REAL VIDEO STARTS HERE
setCart(15, 0, toolZ, 0, -0.7071, 0.7071, 0)
moveGripper(40,20)
rospy.sleep(5)   
# -------------------------------------------------------
# -------------------------------------------------------
setSpeed(60,30)
# 2. open gripper and move phantom phalange back
thread.start_new_thread(moveGripper,(67, 18))
rospy.sleep(0.05)  
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(1) 
# 3. close gripper and move phantom phalange close
thread.start_new_thread(moveGripper,(40, 18))
setCart(15, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(1) 
# -------------------------------------------------------
# 2-3 again with no gripper motion
rospy.sleep(0.35) 
setSpeed(60,30)
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(1) 
# 3. close gripper and move phantom phalange near
setCart(15, 0, toolZ, 0, -0.7071, 0.7071, 0)
# -------------------------------------------------------
# -------------------------------------------------------
#~ # 4.5. Do 2-3 twice at fast speed
setSpeed(300,150)
rospy.sleep(1)
thread.start_new_thread(moveGripper,(67, 80))
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(0.5)

thread.start_new_thread(moveGripper,(40, 80))
setCart(15, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(0.5)
# -------------------------------------------------------
setSpeed(200,100)
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(0.5)
setCart(15, 0, toolZ, 0, -0.7071, 0.7071, 0)
# -------------------------------------------------------
# -------------------------------------------------------

#~ # 6. Open gripper slowly and move phantom finger back
thread.start_new_thread(moveGripper,(65, 8))
setSpeed(12,8)
setCart(50, 0, toolZ, 0, -0.7071, 0.7071, 0)
rospy.sleep(1)
# -------------------------------------------------------
# 7. Rotate the phantom finger around the gripper by 360 deg (may be in two steps)
#7.1 Get Joint

joints=getJoints()

# REsetting joints becuase getjpoints give some different values
joints.j1=-35.37
joints.j2=-20.38 
joints.j3=40.86 
joints.j4=116.24 
joints.j5=40.19 
joints.j6=-122.84

#7.2 Rotate the 6th joint by 360 deg
setZone(3)
setSpeed(100,50)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+90)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+180)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+270)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+360)
rospy.sleep(2)
# -------------------------------------------------------
#7.3 Rotate back the 6th joint fast
setSpeed(500,350)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+270)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+180)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6+90)
setJoints(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,joints.j6)
setZone(1)
# -------------------------------------------------------

#~ # 8. Rotate phntom finger and bring it close -screw motion

setSpeed(75,50)
setCart(50, 0, toolZ, 0, -0.7071, 0.7071, 0)
moveGripper(67,50)
setZone(3)
rospy.sleep(2)
thread.start_new_thread(moveGripper,(40, 5))
setCart(48, 0, toolZ, 0, 0, 1, 0)
setCart(45, 0, toolZ, 0, 0.7071, 0.7071, 0)
setCart(40, 0, toolZ, 0, 1, 0, 0)
setCart(35, 0, toolZ, 0, 0.7071, -0.7071, 0)
rospy.sleep(1)
setZone(1)
setCart(15, 0, toolZ, 0, 0.7071, -0.7071, 0)
rospy.sleep(3)

setCart(100, 0, toolZ, 0, 0.7071, -0.7071, 0)
setJoints(-44.92,-12.88,35.25,110.89,49.09,-120.24)
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)

# -------------------------------------------------------
# ------------- Object pushing Section-------------------
# -------------------------------------------------------

setJoints(-44.92,-12.88,35.25,110.89,49.09,-120.24)
setCart(100, 0, toolZ, 0, -0.7071, 0.7071, 0)
#~ #9 Pick up an object from ground
moveGripper(65, 10)
# go to pick pose
setSpeed(100,50)
setCart(175, 0, toolZ-50, 0, -0.7071, 0.7071, 0)
rospy.sleep(3)
setCart(225, -155, toolZ-50, 0, -0.7071, 0.7071, 0)
setCart(225, -155, toolZ-70, 0, -0.7071, 0.7071, 0)
setCart(225, -155, toolZ-90, 0, -0.7071, 0.7071, 0)

# grasp the object
setGripperForce(20.0)
graspGripper(40,20)

#lift the gripper up
setCart(225, -155, toolZ-50, 0, -0.7071, 0.7071, 0)
setZone(1)
setCart(175, 0, toolZ-50, 0, -0.7071, 0.7071, 0)

rospy.sleep(3)
setCart(70, 0, toolZ+2, 0, -0.7071, 0.7071, 0)
#~ # move in front of phantom finger
setCart(70, 0, toolZ+2, 0, -0.7071, 0.7071, 0)

# Push the object from one side
setSpeed(15,10)
setCart(40, 0, toolZ+2, 0, -0.7071, 0.7071, 0)

# release contact
setCart(100, 0, toolZ+2, 0, -0.7071, 0.7071, 0)

#move the finger to other side by rotating around the gripper
setSpeed(100,50)
setCart(80, 0, toolZ+2, 0, 0, 1, 0)
setCart(65, 0, toolZ+2, 0, 0.7071, 0.7071, 0)

#correct the push.
setSpeed(50,25)
setCart(55, 0, toolZ+2, 0, 0.7071, 0.7071, 0)
setCart(60, 0, toolZ+2, 0, 0.7071, 0.7071, 0)
setCart(45, 0, toolZ+2, 0, 0.7071, 0.7071, 0)

# Take away the finger
setSpeed(100,50)
setCart(100, 0, toolZ+2, 0, 0.7071, 0.7071, 0)

setCart(200, 0, 0, 0, -0.7071, 0.7071, 0)
setCart(225, -155, toolZ-70, 0, -0.7071, 0.7071, 0)
setCart(225, -155, toolZ-85, 0, -0.7071, 0.7071, 0)

moveGripper(65,20)

setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)
