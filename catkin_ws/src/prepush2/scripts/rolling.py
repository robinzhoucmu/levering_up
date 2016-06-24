#!/usr/bin/env python

import rospy
import wsg_50_common.srv
import roslib; roslib.load_manifest("robot_comm")
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
from robot_comm.srv import *
from wsg_50_common.srv import *
from std_srvs.srv  import Empty 
import os, errno
import subprocess 
from geometry_msgs.msg import TransformStamped
import argparse
import tf.transformations as tfm

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

# Initiate Rosnode
        
rospy.init_node('rolling')

# Recording Videos?

rec_vid = False

# Overwrite bag files?

skip_when_exists = True

# List of velocities
list_of_velocities = [10,15,20,25] 

# List of Gripping Forces 
list_of_gripping_forces = [3,4,5,10]

# Threshold for initial orientation check about z
init_ori_thresh = 0.07

# Operating Height
plate_height = -181.09 + 82.0

# Operating X position
rolling_x = 220.0 

# Up distance before grasping
approach_dist = 60.0

# Width of the object
object_width = 30.0

# Gripper velocity
gripper_speed = 10.0

# Number of runs per set of parameters
num_runs = 3

# Position 1
pos_1 = [rolling_x, 0.0, plate_height]

# Position 2
pos_2 = [rolling_x, -70.0, plate_height]

# Standard orientation that is used all time
std_ori = [0.0, -0.7076,0.7076, 0.0] 

# Checks initial orientation 
def check_obj_init_ori():
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        R = tfm.quaternion_matrix([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w])
        euler = tfm.euler_from_matrix(R, 'sxyz')
        print euler
        if euler[2] < init_ori_thresh and euler[2] > - init_ori_thresh:
                return True
        return False

# Checks if rosbag file exists
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
            
# Kills record rosnode 
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

# Gets y position of object
def get_init_pose_y():
        rospy.loginfo("Waiting for object position. If this takes too long, check vicon launch.")
        msg = rospy.wait_for_message("/viconObject", TransformStamped)
        return (msg.transform.translation.y)*1000.0

def move(directory):
    
    # Set directory    
    dir_save_bagfile = os.environ['PREPUSH2DATA_BASE'] + '/rolling/%s/' % (directory)
    make_sure_path_exists(dir_save_bagfile)
    
    #Go Home
    setJoint(-40.36,13.81,55.3,-42.29,-74.23,13.86)
    
    # Initialize counter
    count=1

    for gripping_force in list_of_gripping_forces:
        for push_velocity in list_of_velocities:
            for run_count in range(num_runs):
                
                # Give object time to settle
                rospy.sleep(3)
                
                # Check initial position
                if not check_obj_init_ori():
                    rospy.logerr("Inital orientation not ok. Exit program.")
                    import sys;sys.exit(1)
                
                rospy.loginfo("Rolling with velocity: %.2f mm/s and gripping force = %.2f N ExpNo: %s", push_velocity, gripping_force,count)
                            
                name_of_bag = 'vel=%.2f_gfrc=%.2f_run=%d' % (push_velocity, gripping_force,run_count)
                
                bagfilepath1 = dir_save_bagfile+name_of_bag+"_first.bag"
                bagfilepath2 = dir_save_bagfile+name_of_bag+"_second.bag"
                
                rospy.loginfo("Name of bagfile: %s",name_of_bag)
                if skip_when_exists and os.path.isfile(bagfilepath1) and os.path.isfile(bagfilepath2):
                    print name_of_bag, 'exits', 'skip'
                    continue  
                
                # Set Speed 
                setSpeed(30,10)
                
                # Set grasping force
                setGripperForce(gripping_force)
                                    
                #Home Gripper 
                homeGripper()
                                     
                # Zero Sensors
                zeroSensorFingerFront()
                zeroSensorFingerBack()
                
                # Get y position of object                
                init_y_pose = get_init_pose_y()
                
                # Approach Object
                setCart(pos_1[0],init_y_pose,pos_1[2]+approach_dist, std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
                # Set Speed 
                setSpeed(10,10)
                
                # Move to Grasp pose
                setCart(pos_1[0],init_y_pose,pos_1[2], std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
                # Move to Start pose
                setCart(pos_1[0],pos_1[1],pos_1[2], std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
                # Grasp
                closeGripper(object_width,gripper_speed)
                
                #start recording first rosbag
                if rec_vid:
                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/robot1_CartesianLog","/viconObject","/panasonic_remote/Vid_No"]
                else:
                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/robot1_CartesianLog","/viconObject"]
                rosbag_proc = subprocess.Popen('rosbag record -q -O %s%s %s' % (name_of_bag,"_first", " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                rospy.sleep(1)
                
                # Set speed 
                setSpeed(push_velocity,push_velocity/2.0)
                
                # They see me Rollin'
                setCart(pos_2[0],pos_2[1],pos_2[2], std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
                # Stop recording first rosbag
                rospy.sleep(1)
                terminate_ros_node("/record")
                
                #start recording first rosbag
                if rec_vid:
                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/robot1_CartesianLog","/viconObject","/panasonic_remote/Vid_No"]
                else:
                    topics = ["/netft_1/netft_data","/netft_2/netft_data","/robot1_CartesianLog","/viconObject"]
                rosbag_proc = subprocess.Popen('rosbag record -q -O %s%s %s' % (name_of_bag,"_second", " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
                rospy.sleep(1)
                
                # They see me rollin' back
                setCart(pos_1[0],pos_1[1],pos_1[2], std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
                # Stop recording second rosbag
                rospy.sleep(1)
                terminate_ros_node("/record")
                
                # Open Gripper
                openGripper(50,10)
                
                # Retract from Object
                setCart(pos_1[0],pos_1[1],pos_1[2]+approach_dist, std_ori[0],std_ori[1],std_ori[2],std_ori[3])
                
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
