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
        
rospy.init_node('pivoting')


######### PARAMETERS ################

# Grasp width Line= 35 Flat=29
grasp_width =  34.5#29.0


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
clearBuffer = rospy.ServiceProxy('/robot1_ClearBuffer',robot_ClearBuffer)
addBuffer = rospy.ServiceProxy('/robot1_AddBuffer',robot_AddBuffer)
execBuffer = rospy.ServiceProxy('/robot1_ExecuteBuffer',robot_ExecuteBuffer)

# Recording Videos?

rec_vid = False

skip_when_exists = True

# Default Orientation
std_ori = np.array([0, -0.7071,0.7071,0])
std_ori_for_tfm = [std_ori[1],std_ori[2],std_ori[3],std_ori[0]]

# List of velocities
list_of_velocities = [10,15,20,25] 

# List of Gripping Forces 
list_of_gripping_forces = [35,32,30,27,25,22,20]

# List offeset to specify distance from sensor center. Higher value means closer to earth
list_of_push_offsets = [0,5,10,15,20,25] #[0,10,20,30,40] 

# Pushing distance projected on ground in mm
push_distance=15.0

# Number of runs per set of parameters
num_runs = 3

# Start angle
start_angle = 55.0

# Distance from front of object to center of mass
center_of_sensor_z = -37.25
#-35.5 for flat contact
#-37.25 for line contact

#Operating height
operating_z = -20.0

# Distance from sensor to Object before contact in mm
# Negative value means initial position reqires pushing.

#For line contact
init_push1=0.95
init_push2=0.5
# init_push1=init_push2=1.0 for flat external contact

# Contact pose along X
# Front plane of object to initial grasp finger position + origin to contact plane of sensor CG is at 98.0
contact_pose = 69.0#70.0 

# Ground position in Z
ground_pose_z = -179.0

# Start placing height 
start_placing_z = ground_pose_z  + 75.0

center_y = -78.5

# Default grasp position on ground
default_object_position_on_ground = 300.0

#Safety distance to pusher
safety_dist_push = 50.0

# X distance for placing motion
placing_distance = 100.0

# Initialize variables
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
    
# Kill recording rosnode
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

# Check if path exists
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def check_obj_init_pose():
        global object_position_on_ground
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        ideal_pos = default_object_position_on_ground/1000.0 #- 0.017#0.019
        upper_threshold = ideal_pos + 0.003
        lower_threshold = ideal_pos - 0.003
        # uncomment the followig line for flat external contact
        # if msg.transform.translation.x <  upper_threshold and msg.transform.translation.x > lower_threshold:
                # return True
        if msg.transform.translation.x < 0.25:
            rospy.logerr('Object too close to pusher ... stopping program')
            if rec_vid:
                stopRecording()
            os.kill(os.getpid(), signal.SIGINT)
        rospy.logwarn("Initial position is NOT ok. Expected between %f and %f, but measured: %f. Adjusting grasp pose.", upper_threshold ,lower_threshold, msg.transform.translation.x)
        object_position_on_ground -=  (ideal_pos - msg.transform.translation.x)*1000.0
        object_position_on_ground=object_position_on_ground+3.0 # comment this line for flat external contact
        return False
        
def check_obj_after_push_ori():
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        R = tfm.quaternion_matrix([msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z])
        euler = tfm.euler_from_matrix(R, 'sxyz')
        print euler
        expected = -1.5
        if euler[1] < expected + 0.4 and euler[1] > expected - 0.4:
                return True
        return False
        
def pivot_traj(inital_quat,start_angle,end_angle):
    steps = 7
    angles = [] 
    quaternions = []
    # Calulation of list of angles
    for i in range(steps):
        angles.append(start_angle+(end_angle - start_angle)*(i)/(steps-1))
        
    for angle in angles:
        quaternions.append(tfm.quaternion_multiply(tfm.quaternion_about_axis((angle/180.0)*np.pi,(0,1,0)),inital_quat))
    return quaternions


def move(contact_type):

        global object_position_on_ground
        dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/pivoting/%s/' % (contact_type)
        make_sure_path_exists(dir_save_bagfile)
        
        #Go Home
        setJoint(-2.24,25.29,48.0,-2.33,-73.31,0.67)
        
        count=1
        for gripping_force in list_of_gripping_forces:
            if count > 1:
                    rospy.sleep(1)#600)
            for push_velocity in list_of_velocities:
                    for push_offset in list_of_push_offsets:
                        if push_offset>20.0:
                            init_push=init_push2
                        else:
                            init_push=init_push1
                            
                        for run_count in range(num_runs):
                            
                            # Set rosbag filename and directory
                            rospy.loginfo("Pivoting with velocity: %.2f mm/s and gripping force = %.2f N and push offset = %.2f degree. ExpNo: %s", push_velocity, gripping_force,push_offset,count)
                            name_of_bag = 'vel=%.2f_gfrc=%.2f_offset=%.2f_run=%d' % (push_velocity, gripping_force,push_offset,run_count)
                            bagfilepath = dir_save_bagfile+name_of_bag+".bag"
                            print bagfilepath
                            
                            # Skip if file exists
                            if skip_when_exists and os.path.isfile(bagfilepath):
                                print bagfilepath, 'exits', 'skip'
                                continue  
                            
                            # Set Speed 
                            setSpeed(20,10)
                            
                            # Set grasping force
                            setGripperForce(gripping_force)
                                                
                            #Home Gripper 
                            homeGripper()
                                                 
                            # Zero Sensors
                            zeroSensorFingerFront()
                            zeroSensorFingerBack()
                            zeroSensorPusher()
                            
                            object_position_on_ground = default_object_position_on_ground
                            
                            # Check inital position. In case object is too close to sensor
                            if position_and_ori_checking:
                                    if check_obj_init_pose():
                                            rospy.loginfo("Initial position seems ok!")
                            
                            # Move to Approach Pose 
                            setSpeed(60,30)
                            approach_pose=np.array([object_position_on_ground,center_y,-120])
                            setCart(approach_pose[0],approach_pose[1],approach_pose[2],std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                            
                            #Move to Grasp Pose 
                            setSpeed(20,10)
                            grasp_pose=np.copy(approach_pose)
                            grasp_pose[2]=ground_pose_z
                            grasp_ori = tfm.quaternion_multiply(tfm.quaternion_about_axis(((90.0-start_angle)/180.0)*np.pi,(0,-1,0)),std_ori_for_tfm)
                            setCart(grasp_pose[0],grasp_pose[1],grasp_pose[2],grasp_ori[3],grasp_ori[0],grasp_ori[1],grasp_ori[2]) 
                            
                            #Close Gripper 
                            closeGripper(grasp_width,1)
                            rospy.sleep(1.0)
                            
                            #Retract 
                            setCart(approach_pose[0],approach_pose[1],approach_pose[2],grasp_ori[3],grasp_ori[0],grasp_ori[1],grasp_ori[2]) 
                            
                            
                            # Define initial push position
                            initial_push_end_pos=np.array([contact_pose-init_push,center_y,center_of_sensor_z-push_offset])
                            pivot_quaternions = pivot_traj(std_ori_for_tfm,start_angle,0.0)
                            
                            #GotoPusher 
                            setSpeed(60,30)
                            goto_pusher_pos = np.copy(initial_push_end_pos)
                            goto_pusher_pos[0] +=  safety_dist_push
                            setCart(goto_pusher_pos[0],goto_pusher_pos[1],initial_push_end_pos[2],pivot_quaternions[0][3],pivot_quaternions[0][0],pivot_quaternions[0][1],pivot_quaternions[0][2]) 
                            
                            if position_and_ori_checking:
                                    if not check_obj_after_push_ori():
                                            rospy.logerr("Orientation is NOT ok. Shutdown.")
                                            os.kill(os.getpid(), signal.SIGINT)
                                    else:
                                            rospy.loginfo("Orientation before push seems ok!")
                            
                            #start video 
                            if rec_vid:
                                    startRecording()
                            
                            #Initial Push
                            setSpeed(10,10)
                            setCart(initial_push_end_pos[0],initial_push_end_pos[1],initial_push_end_pos[2],pivot_quaternions[0][3],pivot_quaternions[0][0],pivot_quaternions[0][1],pivot_quaternions[0][2]) 
                            
                            #start recording rosbag
                            if rec_vid:
                                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/viconObject","/panasonic_remote/Vid_No"]
                            else:
                                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/netft_3/netft_data","/robot1_CartesianLog","/viconObject"]
                            rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                            rospy.sleep(1)
                            
                            #Push
                            setSpeed(push_velocity,20)
                            push_pos=np.copy(initial_push_end_pos)
                            push_pos[0]= contact_pose - init_push
                            clearBuffer()
                            for quat in pivot_quaternions:
                                addBuffer(push_pos[0],push_pos[1],push_pos[2],quat[3],quat[0],quat[1],quat[2]) 
                            execBuffer()
                            
                            # Stop recording rosnode
                            rospy.sleep(1)
                            terminate_ros_node("/record")
                            
                            #Retract
                            setSpeed(20,10)
                            retract_pusher_pos = np.copy(push_pos)
                            retract_pusher_pos[0] += safety_dist_push
                            setCart(retract_pusher_pos[0],retract_pusher_pos[1],retract_pusher_pos[2],pivot_quaternions[-1][3],pivot_quaternions[-1][0],pivot_quaternions[-1][1],pivot_quaternions[-1][2]) 
                            
                            #stop video 
                            if rec_vid:
                                    stopRecording()
                            
                            if position_and_ori_checking:                        
                                    if not check_obj_after_push_ori():
                                            rospy.logerr("Orientation is NOT ok. Shutdown.")
                                            os.kill(os.getpid(), signal.SIGINT)
                                    else:
                                            rospy.loginfo("Orientation after push seems ok!")
                                            
                            #Small rotation 
                            place_ori_quat = tfm.quaternion_multiply(tfm.quaternion_about_axis((40.0/180.0)*np.pi,(0,-1,0)),std_ori_for_tfm)
                                            
                            
                            #Goto Place Position
                            setSpeed(60,30)
                            place_pos=np.array([default_object_position_on_ground + placing_distance,center_y,start_placing_z])
                            setCart(place_pos[0],place_pos[1],operating_z,place_ori_quat[3],place_ori_quat[0],place_ori_quat[1],place_ori_quat[2])
                            setSpeed(30,20)
                            setCart(place_pos[0],place_pos[1],place_pos[2],place_ori_quat[3],place_ori_quat[0],place_ori_quat[1],place_ori_quat[2])
                            
                            # Approach place 
                            setSpeed(30,15)
                            clearBuffer()
                            
                            # Generate circular motion
                            for angle in range(90,-1,-10):
                                addBuffer(default_object_position_on_ground+placing_distance - placing_distance*np.cos(angle/180.0*np.pi),center_y,ground_pose_z+(start_placing_z-ground_pose_z)*np.sin(angle/180.0*np.pi),place_ori_quat[3],place_ori_quat[0],place_ori_quat[1],place_ori_quat[2])
                            
                            # Place object
                            execBuffer()
                            
                            # Open Gripper
                            openGripper(40,20)
                            
                            #Retract
                            setSpeed(60,30)
                            setCart(default_object_position_on_ground,center_y,operating_z,std_ori[0],std_ori[1],std_ori[2],std_ori[3]) 
                                                   
                            count+=1
                
    
if __name__ == '__main__':
        outputdir = ''
        parser = argparse.ArgumentParser()
        parser.add_argument("-o","--outputdir", help="Specify output directory",required=True)
        args = parser.parse_args()
        outputdir = args.outputdir
        rospy.loginfo('Output directory is %s', outputdir)
        try:
                move(outputdir)
        except rospy.ROSInterruptException:
                pass







