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
import time
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
#WIDTH_PUSHER = 8.00
OFFSET_PENETRATION = 1.75
# HEIGHT of the pusher from the gripper center to the bottom contact edge
# Explaination: the 80-20 piece is 76 mm long, the gripper diameter is 19.8 mm
# The top tip of the gripper is about 1.5mm taller than the upper edge of the 
# piece: somehow the grip is more firm this way. 
#HEIGHT_PUSHER = 76.00 - 19.8/2 + 1.5
HEIGHT_PUSHER = 76.00 - 19.8/2 + 1.0 + 5.0

# A bunch of velocity options for tcp.
# Linear velocities mm/s
VEL_LINEAR_SLOW = 10
VEL_LINEAR_NORMAL = 25
VEL_LINEAR_FAST = 50
# Angular velocities degree/s
VEL_ANGULAR_SLOW = 5
VEL_ANGULAR_NORMAL = 10
VEL_ANGULAR_FAST = 20

# Pivoting angle for the rolling process.
TOT_PIVOT_ANGLE = np.pi/45
NUM_ROLLING_PTS = 10

# Initial safe pose of the object. (Subject to change).
# Also this is environment set up dependent. Should be moved
# to a configuration file later.
POS0_SAFE = [120, 345, -72]
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
        self.width_pusher = WIDTH_PUSHER - OFFSET_PENETRATION
        self.height_pusher = HEIGHT_PUSHER

    def grasp_pusher(self):
        ''' Grasp the pusher finger. ''' 
        self.gripper.set_gripper_force(20.0)
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
    
    def move_to_platform_above(self):
        return
    def guarded_move_down(self):
        return
    def guarded_move_push_object(self):
        # For now, hard code a x direction offset distance.
        delta_x_push = 46.75
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
        self.tot_pusher_pivot_angle = tot_theta
        # The wall-object contact point coordinate in local frame where the origin is at the 
        # wall-ground corner point and the orientation is aligned with the work object frame.
        pt_rolling_obj = np.array([0, self.height_obj])
        for i in range(num_way_pts):
            theta = tot_theta * i / (num_way_pts + 0.0)
            print theta
            # compute similar triangle edge ratios.
            if (self.width_pusher > self.len_obj * np.sin(theta)):
                l2 = np.sqrt( self.width_pusher * self.width_pusher - 
                              np.square(self.len_obj * np.sin(theta)) )
                l1 = self.len_obj * np.cos(theta) + self.height_obj * np.sin(theta)
                
                # The pusher-object contact point (assuming pusher and the object sticks).
                pt_sticking_obj_pusher = np.array([l1, self.len_obj * np.sin(theta)])
                self.rolling_cors[i,:] = pt_sticking_obj_pusher + (l2 / l1) * (pt_sticking_obj_pusher - pt_rolling_obj)
            
            else:
                assert 0, "The pivoting angle is unachievable without the pusher leaving contact with the ground."
            
    def execute_trajectory_rolling_slip(self):
        ''' Executing the trajectory of rolling cors by resetting the tool center 
        points sequentially. It could be better to buffer all points and execute in batch.
        '''
        num_cors = self.rolling_cors.shape[0]
        for i in range(num_cors):
            tool_cor_x = -(self.rolling_cors[i,0] - (self.len_obj + self.width_pusher/2))
            tool_cor_z = TOOL_Z0 + self.height_pusher - self.rolling_cors[i,1]
            self.robot.set_tool(tool_cor_x, 0, tool_cor_z, 0,0,1,0)
            angle_obj = self.tot_pusher_pivot_angle * (i + 1.0) / num_cors
            angle_pusher = np.arcsin(self.len_obj * np.sin(angle_obj) / self.width_pusher)
            q = np.zeros((4,1), float)
            # Quaternion rotation about y-axis
            q[0] = np.cos(angle_pusher/2.0)
            q[2] = np.sin(angle_pusher/2.0)
            cur_cart = self.robot.get_cart()
            print cur_cart, q
            if (self.robot.get_ik(cur_cart.x, cur_cart.y, cur_cart.z, q[0], q[1], q[2], q[3])):
                print 1
                self.robot.set_cart(cur_cart.x, cur_cart.y, cur_cart.z, q[0], q[1], q[2], q[3])
            else:
                assert 0, "Inverse kinematic solution does not exist"
            # For the last rotation, add slide in component.
            if i == num_cors - 1:
                # Rotate a bit more when trying to slide underneath.
                further_rotation = 5.0
                angle_obj = angle_obj + further_rotation / 180.0 * np.pi
                q[0] = np.cos(angle_pusher/2.0)
                q[2] = np.sin(angle_pusher/2.0)
                tool_z_offset = 200
                tool_cor_z = tool_cor_z + tool_z_offset 
                self.robot.set_tool(tool_cor_x, 0, tool_cor_z, 0,0,1,0)
                cur_cart = self.robot.get_cart()
            
                further_push_in_x = 15
                further_push_up_z = 0
                cur_cart.x = cur_cart.x - further_push_in_x
                cur_cart.z = cur_cart.z + further_push_up_z
                

                if (self.robot.get_ik(cur_cart.x, cur_cart.y, cur_cart.z, q[0], q[1], q[2], q[3])):
                    self.robot.set_speed(VEL_LINEAR_SLOW, VEL_ANGULAR_SLOW)
                    self.robot.set_cart(cur_cart.x, cur_cart.y, cur_cart.z, q[0], q[1], q[2], q[3])
                else:
                    assert 0, "Inverse kinematic solution does not exist"


if __name__ == '__main__':
    traj_gen = TrajectoryGen()
    traj_gen.generate_trajectory_rolling(TOT_PIVOT_ANGLE, NUM_ROLLING_PTS)
    print traj_gen.rolling_cors
    raw_input("Ready for grasping the pusher? Press any key to continue")
    time.sleep(3.0)
    traj_gen.grasp_pusher()
    raw_input("Ready for moving to pre-leverup pose? Press any key to continue")
    traj_gen.move_to_pre_levering()
    raw_input("Ready for rolling?")
    time.sleep(2.0)
    traj_gen.execute_trajectory_rolling_slip()
    raw_input("Loose gripper now? Press any key to continue")
    traj_gen.loose_pusher()
