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
        self.init_ros_services()
        self.set_geometry()
        # Use factory design pattern to create robot and gripper instance.
        self.robot = robot_fac.Manipulator.factory("abb120_mcube")
        self.gripper = gripper_fac.Gripper.factory("wsg50")

        # Set robot tool such that the tcp frame is aligned with the work object frame.
        self.robot.set_tool(0, 0, TOOL_Z0, 0, 0, 1, 0)
        self.robot.set_speed(VEL_LINEAR_NORMAL, VEL_ANGULAR_NORMAL)
        
        # Move robot to the initial safe position.
        self.robot.set_cart(POS0_SAFE[0], POS0_SAFE[1], POS0_SAFE[2], 
                            QUAT0_SAFE[0], QUAT0_SAFE[1], QUAT0_SAFE[2], QUAT0_SAFE[3])
        # Grasp the pusher from a fixture (or human holding it).
        # grasp_pusher()

    def init_ros_services(self):
        return
    def move_to_init_pose(self):
        ''' Move the robot to a pregrasp pose'''
        
    def set_geometry(self):        
        self.height_obj = HEIGHT_OBJ
        self.len_obj = LEN_OBJ
        self.width_pusher = WIDTH_PUSHER
        self.height_pusher = HEIGHT_PUSHER

    def grasp_pusher(self):
        ''' Grasp the pusher finger. ''' 
        self.gripper.close_gripper(self.width_pusher - 1.0, 5.0)
        
    def loose_pusher(self):
        self.gripper.open_gripper(self.width_pusher + 10.0, 5.0)
    
    def move_to_pre_levering(self):
        ''' Move the robot (already grasping the finger) to pre-levering pose.
        After the movement, the pusher and the object should be in contact'''
        # Assume the gripper now has the pusher in grip. We move down the robot
        self.move_to_platform_above()
        self.robot.set_speed(VEL_LINEAR_SLOW, VEL_ANGULAR_SLOW)
        # perform guarded move to touch the supporting ground.
        self.guarded_move_down()
        # perform guarded move to push until object contact the ground-wall corner.
        self.guarded_move_push_object()
    
    def move_to_platform_above():
        return
    def guarded_move_down():
        return
    def guarded_move_push_object():
        # For now, hard code a x direction offset distance.
        delta_x_push = 42.8
        self.robot.set_cart(POS0_SAFE[0] - delta_x_push, POS0_SAFE[1], POS0_SAFE[2], 
                            QUAT0_SAFE[0], QUAT0_SAFE[1], QUAT0_SAFE[2], QUAT0_SAFE[3])

    def generate_trajectory_rolling(self, tot_theta, num_way_pts):
        ''' Return the trajectory of CORs such that the object rolls about the 
        wall-to-object contact points while the pusher sticks with the object at
        pusher-to-object contact point. 
        - tot_theta is the total pivoting angle of the object.
        - num_way_pts is the total number of way points along the trajectory.
        '''
        # represent the motion of the pusher as a trajectory of small rotations about CORs.
        self.rolling_cors = np.zeros((num_way_pts, 2), float)
        # The wall-object contact point coordinate in local frame where the origin is at the 
        # wall-ground corner point and the orientation is aligned with the work object frame.
        pt_rolling_obj = np.array([0, self.height_obj])
        for i in range(num_way_pts):
            theta = tot_theta * i / (num_thetas + 0.0)
            print theta
            # compute similar triangle edge ratios.
            if (self.width_pusher > self.len_object * np.sin(theta)):
                l2 = np.sqrt( self.width_pusher * self.width_pusher - 
                              np.square(self.len_object * np.sin(theta)) )
                l1 = self.len_object * np.cos(theta) + self.height_obj * np.sin(theta)
                
                # The pusher-object contact point (assuming pusher and the object sticks).
                pt_sticking_obj_pusher = np.array([l1, self.len_obj * np.sin(theta)])
                self.rolling_cors[i,:] = pt_sticking_obj_pusher + (l2 / l1) * (pt_sticking_obj_pusher - pt_rolling_obj)
            
            else:
                assert 0, "The pivoting angle is unachievable without the pusher leaving contact with the ground."
            
    def execute_trajectory_rolling(self):
        ''' Executing the trajectory of rolling cors by resetting the tool center 
        points sequentially. It could be better to buffer all points and execute in batch.
        '''
        num_cors = self.rolling_cors.shape[0]
        for i in range(num_cors):
            tool_cor_x = -(self.rolling_cors[i,0] - (self.len_obj + self.width_pusher/2))
            tool_cor_z = TOOLZ0 + self.height_pusher - self.rolling_cors[i,1]
            self.robot.set_tool(tool_cor_x, 0, tool_cor_z, 0,0,1,0)
        



if __name__ == '__main__':
    traj_gen = TrajectoryGen()
