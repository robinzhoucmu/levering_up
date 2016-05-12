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

import thread

#Define call methods
setCart = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getCart = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setSpeed = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)
homeGripper = rospy.ServiceProxy('/wsg_50_driver/homing', Empty)
moveGripper = rospy.ServiceProxy('/wsg_50_driver/move', Move)
setGripperForce = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
closeGripper = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
openGripper = rospy.ServiceProxy('/wsg_50_driver/release', Move)


# Tool frame is set to fingertip of the long gripper finger
# Work object is set to external finger

# 1. Start with gripper closed(phantom finger close to other finger)
#1.1 Move gripper 100 mm away from external finger in X direction
setSpeed(50,20)
setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)

homeGripper()
moveGripper(35,20)
setCart(10, 0, 0, 0, -0.7071, 0.7071, 0)
    
# 2. open gripper and move phantom phalange back
thread.start_new_thread(moveGripper,(70, 15))
setSpeed(20,10)
setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)

#~ # 3. close gripper and move phantom phalange near
#~ moveGripper(35,20)
#~ setSpeed(50,20)
#~ setCart(10, 0, 0, 0, -0.7071, 0.7071, 0)
#~ 
#~ # 4.5. Do 2-3 twice at fast speed
#~ moveGripper(70,200)
#~ setSpeed(100,40)
#~ setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)
#~ 
#~ moveGripper(35,200)
#~ setCart(10, 0, 0, 0, -0.7071, 0.7071, 0)
#~ 
#~ moveGripper(70,200)
#~ setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)
#~ 
#~ moveGripper(35,200)
#~ setSpeed(100,40)
#~ setCart(10, 0, 0, 0, -0.7071, 0.7071, 0)
#~ 
#~ # 6. Open gripper slowly and move phantom finger back
#~ moveGripper(70,10)
#~ setSpeed(40,10)
#~ setCart(100, 0, 0, 0, -0.7071, 0.7071, 0)

# 7. Rotate the phantom finger around the gripper by 360 deg (may be in two steps)

# 8. Rotate finger in phantom finger in apposite 360 at varying speed

# -------------------------------------------------------

# Pick up an object from ground

# Push the object from one side, move the finger to other side by rotating around the gripper and correct the push.

