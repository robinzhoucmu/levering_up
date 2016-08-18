import rospy
import roslib; 
roslib.load_manifest("netft_rdt_driver")
from wsg_50_common.srv import *
import tf.transformations as tfm
import robot_fac
import gripper_fac
import numpy as np

import os
import signal
import subprocess 
import httplib
from std_srvs.srv  import Empty 
from netft_rdt_driver.srv import Zero
from datetime import date
import errno
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import TransformStamped
import sys, argparse

#Length from robot phalange to center of the gripper finger.
TOOL_Z0 = 145
# Height of the object
HEIGHT_OBJ = 6.21
# Length of the object
LEN_OBJ = 76.00
# Width of the pusher
WIDTH_PUSHER = 8.00
# HEIGHT of the pusher from the gripper center to the bottom contact edge
# Explaination: the 80-20 piece is 76 mm long, the gripper diameter is 19.8 mm
# The top tip of the gripper is about 1.5mm taller than the upper edge of the 
# piece: somehow the grip is more firm this way. 
HEIGHT_PUSHER = 76.00 - 19.8/2 + 1.5

# A bunch of velocity options for tcp.
# Linear velocities mm/s
VEL_LINEAR_SLOW = 5
VEL_LINEAR_NORMAL = 25
VEL_LINEAR_FAST = 50
# Angular velocities degree/s
VEL_ANGULAR_SLOW = 5
VEL_ANGULAR_NORMAL = 10
VEL_ANGULAR_FAST = 20

# Initial safe pose of the object. (Subject to change).
# Also this is environment set up dependent. Should be moved
# to a configuration file later.
POS0_SAFE = [120, 345, -77]
QUAT0_SAFE = [1, 0, 0, 0]

class TrajectoryGen:
    def __init__(self):
        init_ros_services()

        # Use factory design pattern to create robot and gripper instance.
        self.robot = robot_fac.Manipulator.factory("abb120_mcube")
        self.gripper = gripper_fac.Gripper.factory("wsg50")

        # Set robot tool such that the tcp frame is aligned with the work object frame.
        robot.set_tool(0, 0, toolZ0, 0, 0, 1, 0)
        robot.set_speed(VEL_LINEAR_NORMAL, VEL_ANGULAR_NORMAL)
        
        # Move robot to the initial safe position.
        robot.set_cart(POS0_SAFE)
    def init_ros_services(self):
    def move_to_init_pose(self):
        ''' Move the robot to a pregrasp pose'''
        
    def set_geometry(self):
        

    def grasp_pusher(self):
        ''' Grasp the pusher finger. ''' 

    def move_to_pre_levering(self):
        ''' Move the robot (already grasping the finger) to pre-levering pose.
        After the movement, the pusher and the object should be in contact'''

    def generate_trajectory_rolling(self, num_way_pts):
        ''' Return the trajectory of CORs such that the object rolls about the 
        wall-to-object contact points while the pusher sticks with the object at
        pusher-to-object contact point. '''

    def execute_trajectory_rolling(self):
        '''  '''
    
    
