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
        
rospy.init_node('lateral_slide')

######### PARAMETERS ################

# Grasp width Line= 35 Flat=29
grasp_width =  20.0
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



# Default Orientation
std_ori = np.array([0, -0.701,0.701,0])

# List of velocities
list_of_velocities = [10,15,20]

# List of Gripping Forces 
list_of_gripping_forces = [15,20,25]

# Pushing distance projected on ground in mm
slide_distance=30


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
        
        dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/lateral_slide/%s_%s_%s/%s/' % (today.month,today.day,today.year,data_dir)
        make_sure_path_exists(dir_save_bagfile)
        
        count=1
        
        for slide_velocity in list_of_velocities:
            for gripping_force in list_of_gripping_forces:
             
                rospy.loginfo("Straight push with velocity: %.2f mm/s and gripping force = %.2f N. ExpNo: %s", slide_velocity, gripping_force,count)
                
                name_of_bag = 'lateral_slide_%s_%s%s%s_vel=%.2f_gfrc=%.2f' % (data_dir,today.month, today.day, today.year, slide_velocity, gripping_force)
                
                # Set Speed 
                setSpeed(20,2)
                
                setGripperForce(gripping_force)
                        
                # Move to Home Pose 
                #home_pose=np.array([365,0,360])
                #setCart(home_pose[0],home_pose[1],home_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                #wait_for_goal_position(home_pose)
                
                #Home Gripper 
                homeGripper()
                
                                     
                # Zero Sensors
                zeroSensorFingerFront()
               
               
                
                # Move to Approach Pose 
                setSpeed(60,2)
                approach_pose=np.array([2,-75.0,-37])
                approach_pose[1]-=slide_distance/2
                setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(approach_pose)
                
                ##Move to Grasp Pose 
                #setSpeed(5,2)
                #grasp_pose=np.copy(approach_pose)
                #grasp_pose[2]=168.1
                #setCart(grasp_pose[0],grasp_pose[1],grasp_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                #wait_for_goal_position(grasp_pose)
                
                #Close Gripper 
                closeGripper(grasp_width,1)
                
                #start recording rosbag
                topics = ["/netft_1/netft_data","/robot1_CartesianLog"]
                rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                
                #Slide
                setSpeed(slide_velocity,1)
                slide_pos=np.copy(approach_pose)
                slide_pos[1]+=slide_distance
                setCart(slide_pos[0],slide_pos[1],slide_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                wait_for_goal_position(slide_pos)
                
                terminate_ros_node("/record")
                
                #Open Gripper 
                openGripper(50,2)
                                
                count+=1
    
if __name__ == '__main__':
        try:
                move()
        except rospy.ROSInterruptException:
                pass







