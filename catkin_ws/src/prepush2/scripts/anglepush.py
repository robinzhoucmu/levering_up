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
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import TransformStamped
import tf.transformations as tfm
import sys, argparse

# Initiate Rosnode
        
rospy.init_node('angle_push')


######### PARAMETERS ################

# Grasp width square prism= 34.5, cylinder with Flat=29
grasp_width =  29.0 # 34.5 

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



# Recording Videos?

rec_vid = False

skip_when_exists = True

# Default Orientation
std_ori = np.array([0, -0.7071,0.7071,0])

# Orientation checking threshold
after_push_ori_thresh = 0.35
before_push_ori_thresh = 0.18
# List of velocities
list_of_velocities = [25,20,15,10] 

# List of Gripping Forces 
list_of_gripping_forces = [25,22,20]

# List of angles in degrees
list_of_angles = [0,10,-10,20,-20]

# Pushing distance projected on ground in mm
push_distance=10.0#15.0

# Number of runs per set of parameters
num_runs = 3

# Distance from front of object to center of mass

center_y = -78.0

# Distance from sensor to Object before contact in mm
# Negative value means initial position reqires pushing.
init_push=5.0

# Contact pose along X
# Front plane of object to initial grasp finger position + origin to contact plane of sensor CG is at 98.0
#contact_pose = 101.0 #106 #Changed this for machined dobject
contact_pose = 100.5 #101 106 #Changed this for machined dobject

# Ground position in Z
ground_pose_z = -180.5 #-181.0

# Default grasp position on ground
default_object_position_on_ground = 235.0 #240.0 #Changed this for machined dobject

#Safety distance to pusher
safety_dist_push = 15.0

# Initializing variables
position_and_ori_checking = True
first_time = True
object_position_on_ground = default_object_position_on_ground


if rec_vid:
        startRecording = rospy.ServiceProxy('/panasonic_remote/start_rec', Empty)
        stopRecording = rospy.ServiceProxy('/panasonic_remote/stop_rec', Empty)


#Ros Log callback
def logcallback(data):
        global first_time
        if data.name == '/vicon':
                if data.msg == 'PrePushObj occluded, not publishing... ':
                        if first_time == False:
                                rospy.logerr('Object occluded ... stopping program')
                                if rec_vid:
                                        stopRecording()
                                os.kill(os.getpid(), signal.SIGINT)
                        else:
                                first_time = False
        

rospy.Subscriber("/rosout", Log, logcallback)

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
            
def check_obj_init_pose():
        global object_position_on_ground
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        ideal_pos = default_object_position_on_ground/1000.0 - 0.017#0.019
        upper_threshold = ideal_pos + 0.006
        lower_threshold = ideal_pos - 0.006
        if msg.transform.translation.x <  upper_threshold and msg.transform.translation.x > lower_threshold:
                return True
        if msg.transform.translation.x < 0.15:
            rospy.logerr('Object too close to pusher ... stopping program')
            if rec_vid:
                stopRecording()
            os.kill(os.getpid(), signal.SIGINT)
            
        rospy.logwarn("Initial position is NOT ok. Expected between %f and %f, but measured: %f. Adjusting grasp pose.", upper_threshold ,lower_threshold, msg.transform.translation.x)
        object_position_on_ground -=  (ideal_pos - msg.transform.translation.x)*1000.0
        return False
        
def check_obj_before_push_ori():
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        R = tfm.quaternion_matrix([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w])
        euler = tfm.euler_from_matrix(R, 'sxyz')
        print euler
        if euler[1] < before_push_ori_thresh  and euler[1] > -before_push_ori_thresh:
                return True
        return False

def check_obj_after_push_ori():
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        R = tfm.quaternion_matrix([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w])
        euler = tfm.euler_from_matrix(R, 'sxyz')
        print euler
        if euler[1] < after_push_ori_thresh  and euler[1] > -after_push_ori_thresh:
                return True
        return False
            
def move(contact_type):

        global object_position_on_ground
        
        # Set directory 
        dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/angle_push/%s/' % (contact_type)
        make_sure_path_exists(dir_save_bagfile)
        
        #Go Home
        setJoint(-2.24,25.29,48.0,-2.33,-73.31,0.67)
        
        count=1
        for gripping_force in list_of_gripping_forces:
            if count > 1:
                    rospy.sleep(30)
            for push_velocity in list_of_velocities:
                    for angle in list_of_angles:
                        for run_count in range(num_runs):
                        
                            rospy.loginfo("Anglular push with velocity: %.2f mm/s and gripping force = %.2f N and angle = %.2f degree. ExpNo: %s", push_velocity, gripping_force,angle,count)
                            
                            name_of_bag = 'vel=%.2f_gfrc=%.2f_angl=%.2f_run=%d' % (push_velocity, gripping_force,angle,run_count)
                            
                            bagfilepath = dir_save_bagfile+name_of_bag+".bag"
                            print bagfilepath
                            if skip_when_exists and os.path.isfile(bagfilepath):
                                print bagfilepath, 'exits', 'skip'
                                continue  
                            
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
                            
                            # Get Default position on ground (where it's supposed to be placed)
                            object_position_on_ground = default_object_position_on_ground
                            
                            # Check initial position and orientaion
                            if position_and_ori_checking:
                                    if check_obj_init_pose():
                                            rospy.loginfo("Initial position seems ok!")
                            
                            # Move to Approach Pose 
                            setSpeed(60,30)
                            approach_pose=np.array([object_position_on_ground,center_y,-120])
                            setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(approach_pose)
                            
                            #Move to Grasp Pose 
                            setSpeed(20,10)
                            grasp_pose=np.copy(approach_pose)
                            grasp_pose[2]=ground_pose_z
                            setCart(grasp_pose[0],grasp_pose[1],grasp_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(grasp_pose)
                            
                            #Close Gripper 
                            closeGripper(grasp_width,1)
                            rospy.sleep(1.0)
                            
                            #Retract 
                            setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(approach_pose)
                            
                            
                            initial_push_end_pos=np.array([contact_pose-init_push,center_y,-35.5])
                            
                            #GotoPusher 
                            setSpeed(20,10)
                            goto_pusher_pos = np.copy(initial_push_end_pos)
                            goto_pusher_pos[0] +=  safety_dist_push
                            setCart(goto_pusher_pos[0],goto_pusher_pos[1],goto_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(goto_pusher_pos)
                            
                            if position_and_ori_checking:
                                    if not check_obj_before_push_ori():
                                            rospy.logerr("Orientation is NOT ok. Shutdown.")
                                            os.kill(os.getpid(), signal.SIGINT)
                                    else:
                                            rospy.loginfo("Orientation before push seems ok!")
                            
                            #start video 
                            if rec_vid:
                                    startRecording()
                            
                            #Initial Push
                            setSpeed(5,2)
                            setCart(initial_push_end_pos[0],initial_push_end_pos[1],initial_push_end_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(initial_push_end_pos)
                            
                                                   
                            #start recording rosbag
                            if rec_vid:
                                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/viconObject","/panasonic_remote/Vid_No"]
                            else:
                                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/viconObject"]
                            rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                            
                           
                            rospy.sleep(1)
                            
                            #Push
                            setSpeed(push_velocity,1000)
                            push_pos=np.copy(initial_push_end_pos)
                            push_pos[0]= contact_pose - push_distance_x - init_push
                            push_pos[2]+= push_distance_z
                            setCart(push_pos[0],push_pos[1],push_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(push_pos)
                            
                            rospy.sleep(1)
                            
                            # Stop recording rosbag
                            terminate_ros_node("/record")
                            
                            #Retract
                            setSpeed(5,5)
                            retract_pusher_pos = np.copy(push_pos)
                            retract_pusher_pos[0] += safety_dist_push
                            setCart(retract_pusher_pos[0],retract_pusher_pos[1],retract_pusher_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(retract_pusher_pos)
                            
                                                   
                            #stop video 
                            if rec_vid:
                                    stopRecording()
                            
                            # Check orientation after push
                            if position_and_ori_checking:                        
                                    if not check_obj_after_push_ori():
                                            rospy.logerr("Orientation is NOT ok. Shutdown.")
                                            os.kill(os.getpid(), signal.SIGINT)
                                    else:
                                            rospy.loginfo("Orientation after push seems ok!")
                                            
                            
                            #Goto Place Position
                            setSpeed(60,30)
                            place_pos=np.array([default_object_position_on_ground-push_distance_x- init_push,center_y,-120])
                            setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                            wait_for_goal_position(place_pos)
                            
                            # Approach place 
                            setSpeed(50,20)
                            approach_place_pos=np.copy(place_pos)
                            approach_place_pos[2]=ground_pose_z
                            setCart(approach_place_pos[0],approach_place_pos[1],approach_place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(approach_place_pos)
                            openGripper(40,2)
                            
                            #Retract
                            setSpeed(60,30)
                            setCart(place_pos[0],place_pos[1],place_pos[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            wait_for_goal_position(place_pos)
                            
                                                   
                            count+=1
                
    
if __name__ == '__main__':
        outputdir = ''
        parser = argparse.ArgumentParser()
        parser.add_argument("-o","--outputdir", help="Specify output directory",required=True)
        args = parser.parse_args()
        outputdir = args.outputdir
        print 'Output directory is', outputdir
        try:
            # check_obj_after_push_ori()
            move(outputdir)
        except rospy.ROSInterruptException:
                pass







