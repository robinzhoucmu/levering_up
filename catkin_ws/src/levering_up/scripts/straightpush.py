#!/usr/bin/env python

import os
import signal
import numpy as np
import rospy
import subprocess 
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from wsg_50_common.srv import *
from std_srvs.srv  import Empty 
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
from datetime import date
import errno

# Initiate Rosnode
        
rospy.init_node('straight_push')

######### PARAMETERS ################

# Grasp width Line= 35 Flat=29
grasp_width =  29.0
# Data directory (also used for file naming): "flat_contacts"; "line_contacts"
data_dir = "flat_contacts"

#Define call methods
setCart = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getCart = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setSpeed = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)
homeGripper = rospy.ServiceProxy('/wsg_50_driver/homing', Empty)
closeGripper = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
openGripper = rospy.ServiceProxy('/wsg_50_driver/release', Move)
setGripperForce = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
zeroSensorFingerFront = rospy.ServiceProxy('/netft_1/zero', Zero)
zeroSensorFingerBack = rospy.ServiceProxy('/netft_2/zero', Zero)
zeroSensorPusher = rospy.ServiceProxy('/netft_3/zero', Zero)



# Default Orientation
std_ori = np.array([0, -0.701,0.701,0])

# List of velocities
list_of_velocities = [15,20]

# List of Gripping Forces 
list_of_gripping_forces = [15,20]



# Wait until goal position is reached 
def wait_for_goal_position(goal_position):
    goal_reached=False
    while not goal_reached:
        resp = getCart()
        if abs(resp.x - goal_position[0]) < 0.1 and abs(resp.y - goal_position[1]) < 0.1 and abs(resp.z - goal_position[2]) < 0.1:
            goal_reached=True
    rospy.sleep(1)
    return True
    
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)
            
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
            
def move():
        
        #Get date of today
        today = date.today()
        
        dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/straight_push/%s_%s_%s/%s/' % (today.month,today.day,today.year,data_dir)
        make_sure_path_exists(dir_save_bagfile)
        
        for push_velocity in list_of_velocities:
            for gripping_force in list_of_gripping_forces:
                    
                rospy.loginfo("Straight push with velocity: %.2f mm/s and gripping force = %.2f N", push_velocity, gripping_force)
                
                name_of_bag = 'straight_push_%s_%s%s%s_vel=%.2f_gfrc=%.2f' % (data_dir,today.month, today.day, today.year, push_velocity, gripping_force)
                
                # Set Speed 
                setSpeed(20,2)
                
                setGripperForce(gripping_force)
                        
                # Move to Home Pose 
                home_pose=np.array([365,0,360])
                setCart(home_pose[0],home_pose[1],home_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                wait_for_goal_position(home_pose)
                
                #Home Gripper 
                homeGripper()
                
                raw_input("Ready to go? Press Enter to continue...")
                
                # Zero Sensors
                zeroSensorFingerFront()
                zeroSensorFingerBack()
                zeroSensorPusher()
                
                # Move to Approach Pose 
                setSpeed(60,2)
                approach_pose=np.array([365,58,200])
                setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_pose)
                
                #Move to Grasp Pose 
                setSpeed(5,2)
                grasp_pose=np.array([365,58,168.1])
                setCart(grasp_pose[0],grasp_pose[1],grasp_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(grasp_pose)
                
                #Close Gripper 
                closeGripper(grasp_width,1)
                
                #Retract 
                setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_pose)
                
                #GotoPusher 
                setSpeed(40,2)
                goto_pusher_pos=np.array([365,214,275])
                setCart(goto_pusher_pos[0],goto_pusher_pos[1],goto_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(goto_pusher_pos)
                
                #ApproachPusher
                setSpeed(5,2)
                approach_pusher_pos=np.array([365,242,275])
                setCart(approach_pusher_pos[0],approach_pusher_pos[1],approach_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_pusher_pos)
                
                #Wait for user input
                raw_input("Check orientation of object. Press Enter to continue...")
                
                #start recording rosbag
                topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/vicon/PrePushObj/PrePushObj"]
                rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                
                #Push
                setSpeed(push_velocity,1)
                push_distance=20
                push_pos=np.copy(approach_pusher_pos)
                push_pos[1]+=push_distance
                setCart(push_pos[0],push_pos[1],push_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(push_pos)
                
                terminate_ros_node("/record")
                
                #Retract
                setSpeed(5,1)
                setCart(approach_pusher_pos[0],approach_pusher_pos[1],approach_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_pusher_pos)
                
                #Goto Place Position
                setSpeed(40,2)
                place_pos=np.array([365,58,200])
                setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                wait_for_goal_position(place_pos)
                
                # Approach place 
                setSpeed(5,2)
                approach_place_pos=np.array([365,58,172])
                setCart(approach_place_pos[0],approach_place_pos[1],approach_place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_place_pos)
                openGripper(50,2)
                
                #Retract
                setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(place_pos)
                
                #Home 
                setSpeed(60,2)
                setCart(home_pose[0],home_pose[1],home_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                wait_for_goal_position(home_pose)
    
if __name__ == '__main__':
        try:
                move()
        except rospy.ROSInterruptException:
                pass







