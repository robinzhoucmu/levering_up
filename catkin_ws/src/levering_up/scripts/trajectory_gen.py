import numpy as np
import rospy
import roslib; 
roslib.load_manifest("robot_comm")
roslib.load_manifest("netft_rdt_driver")
from robot_comm.srv import *
from wsg_50_common.srv import *
import tf.transformations as tfm

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

class TrajectoryGen:
    def __init__(self):
    def init_ros_services(self):
    def set_geometry(self):
        
    def move_to_init_pose(self):
        ''' Move the robot to a pregrasp pose'''
    def grasp_pusher(self):
        ''' Grasp the pusher finger. ''' 
    def move_to_pre_levering(self):
        ''' Move the robot (already grasping the finger) to pre-levering pose.
        After the movement, the pusher and the object should be in contact'''

    def generate_trajectory_rolling(self):
        ''' Return the trajectory of CORs such that the object rolls about the 
        wall-to-object contact points while the pusher sticks with the object at
        pusher-to-object contact point. '''

    def execute_trajectory_rolling(self):
        '''  '''
    
   
