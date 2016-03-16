#!/usr/bin/env python

import os
import signal
import numpy as np
import rospy
import subprocess 
import roslib; roslib.load_manifest("robot_comm")
import httplib
from robot_comm.srv import *
from wsg_50_common.srv import *
from std_srvs.srv  import Empty 
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
from datetime import date
import errno
#import pdb

# Initiate Rosnode
        
rospy.init_node('angle_push')

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
startRecording = rospy.ServiceProxy('/panasonic_remote/start_rec', Empty)
stopRecording = rospy.ServiceProxy('/panasonic_remote/stop_rec', Empty)


# X distance to contact plane


# Default Orientation
std_ori = np.array([0, 0.7071,-0.7071,0])

# List of velocities
list_of_velocities = [10,15,20]

# List of Gripping Forces 
list_of_gripping_forces = [15,30]

# List of angles in degrees
list_of_angles = [0]#[-20,-10,0,10,20,25]

# Pushing distance projected on ground in mm
push_distance=15.0

# Distance from front of object to center of mass


# Distance from sensor to Object before contact in mm
# Negative value means initial position reqires pushing.
init_push=4.0

# Contact pose along X
# Front plane of object to center of mass + origin to contact plane of sensor
contact_pose = 107.5

# Ground position in Z
ground_pose_x = -143.0

# Default position of center of gravity on ground
default_CG_on_ground_x = 140.0


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
        
        dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/angle_push/%s_%s_%s/%s/CoM_teting/' % (today.month,today.day,today.year,data_dir)
        make_sure_path_exists(dir_save_bagfile)
        
        count=1
        
        for push_velocity in list_of_velocities:
            for gripping_force in list_of_gripping_forces:
                    for angle in list_of_angles:
                    
                        rospy.loginfo("Anglular push with velocity: %.2f mm/s and gripping force = %.2f N and angle = %.2f degree. ExpNo: %s", push_velocity, gripping_force,angle,count)
                        
                        name_of_bag = 'angle_push_%s_%s%s%s_vel=%.2f_gfrc=%.2f_angl=%.2f' % (data_dir,today.month, today.day, today.year, push_velocity, gripping_force,angle)
                        
                        # Set Speed 
                        setSpeed(20,2)
                        
                        setGripperForce(gripping_force)
                        
                        #Calculate X and Y component of push distance
                        push_distance_x = push_distance*np.cos(angle*np.pi/180)
                        push_distance_z = np.sin(angle*np.pi/180)*push_distance
                                
                                            
                        #Home Gripper 
                        homeGripper()
                        
                                             
                        # Zero Sensors
                        zeroSensorFingerFront()
                        zeroSensorFingerBack()
                        zeroSensorPusher()
                        
                        #raw_input("Ready? Press Enter to continue...")
                        #pdb.set_trace()
                        # Move to Approach Pose 
                        setSpeed(60,2)
                        approach_pose=np.array([default_CG_on_ground_x,-75,-120])
                        approach_pose[0]+=(push_distance_x+init_push)/2.0
                        setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(approach_pose)
                        
                        #Move to Grasp Pose 
                        setSpeed(5,2)
                        grasp_pose=np.copy(approach_pose)
                        grasp_pose[2]=ground_pose_x
                        setCart(grasp_pose[0],grasp_pose[1],grasp_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(grasp_pose)
                        
                        #Close Gripper 
                        closeGripper(grasp_width,1)
                        
                        #Retract 
                        setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(approach_pose)
                        
                        #GotoPusher 
                        setSpeed(40,2)
                        goto_pusher_pos=np.array([130,-75,-35.5])
                        setCart(goto_pusher_pos[0],goto_pusher_pos[1],goto_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(goto_pusher_pos)
                        
                        #Initial Push
                        setSpeed(5,2)
                        approach_pusher_pos=np.array([contact_pose+push_distance_x/2.0-init_push/2.0,-75,-35.5])#116
                        setCart(approach_pusher_pos[0],approach_pusher_pos[1],approach_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(approach_pusher_pos)
                        
                                               
                        #start recording rosbag
                        topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/viconObject","/panasonic_remote/Vid_No"]
                        rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                        
                        #start video 
                        
                        startRecording()
                        rospy.sleep(1)
                        
                        #Push
                        setSpeed(push_velocity,1)
                        push_pos=np.copy(approach_pusher_pos)
                        push_pos[0]= contact_pose - push_distance_x/2.0 - init_push/2.0
                        push_pos[2]+= push_distance_z
                        setCart(push_pos[0],push_pos[1],push_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(push_pos)
                        
                        rospy.sleep(1)
                        
                        terminate_ros_node("/record")
                        
                        #Retract
                        setSpeed(5,1)
                        setCart(approach_pusher_pos[0],approach_pusher_pos[1],approach_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(approach_pusher_pos)
                        
                        #stop video 
                        stopRecording()
                        
                        #Goto Place Position
                        setSpeed(40,2)
                        place_pos=np.array([default_CG_on_ground_x-push_distance_x/2.0- init_push/2.0,-75,-120])
                        setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                        wait_for_goal_position(place_pos)
                        
                        # Approach place 
                        setSpeed(5,2)
                        approach_place_pos=np.copy(place_pos)
                        approach_place_pos[2]=ground_pose_x
                        setCart(approach_place_pos[0],approach_place_pos[1],approach_place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(approach_place_pos)
                        openGripper(40,2)
                        
                        #Retract
                        setSpeed(20,2)
                        setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                        wait_for_goal_position(place_pos)
                        
                                               
                        count+=1
                        
    
if __name__ == '__main__':
        try:
                move()
        except rospy.ROSInterruptException:
                pass







