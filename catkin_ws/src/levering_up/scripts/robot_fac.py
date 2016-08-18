import rospy
import roslib
roslib.load_manifest("robot_comm")
from robot_comm.srv import *

class Manipulator(object):
    def factory(type):
        if type == "abb120_mlab": return Abb120MLab()
        if type == "abb120_mcube": return Abb120MCube()
        assert 0, "Bad robot type: " + type
    factory = staticmethod(factory)

class Abb120MCube(Manipulator):
    def __init__(self):
        self.init_ros_services()
    def init_ros_services(self):
        # Initialize related robot services.
        self.get_cart = rospy.ServiceProxy('/robot1_GetCartesian', 
                                           robot_GetCartesian)
        self.set_work_object = rospy.ServiceProxy('/robot1_SetWorkObject', 
                                                  robot_SetWorkObject)
        self.set_tool = rospy.ServiceProxy('/robot1_SetTool', 
                                           robot_SetTool)
        self.set_cart = rospy.ServiceProxy('/robot1_SetCartesian', 
                                           robot_SetCartesian)
        self.set_joints = rospy.ServiceProxy('/robot1_SetJoints', 
                                             robot_SetJoints)
        self.get_joints = rospy.ServiceProxy('/robot1_GetJoints', 
                                             robot_GetJoints)
        self.set_speed = rospy.ServiceProxy('/robot1_SetSpeed', 
                                            robot_SetSpeed)
        self.get_ik = rospy.ServiceProxy('/robot1_GetIK', robot_GetIK)
        

class Abb120MLab(Manipulator):
    def __init__(self):
        self.init_ros_services()
    def init_ros_services(self):
        # Initialize related robot services.
        self.get_cart = rospy.ServiceProxy('/robot_GetCartesian', 
                                           robot_GetCartesian)
        self.set_work_object = rospy.ServiceProxy('/robot_SetWorkObject', 
                                                  robot_SetWorkObject)
        self.set_tool = rospy.ServiceProxy('/robot_SetTool', 
                                           robot_SetTool)
        self.set_cart = rospy.ServiceProxy('/robot_SetCartesian', 
                                           robot_SetCartesian)
        self.set_joints = rospy.ServiceProxy('/robot_SetJoints', 
                                             robot_SetJoints)
        self.get_joints = rospy.ServiceProxy('/robot_GetJoints', 
                                             robot_GetJoints)
        self.add_trajectory = rospy.ServiceProxy('/robot_AddTrajectoryPoint', 
                                                 robot_AddTrajectoryPoint)
        self.clear_trajectory = rospy.ServiceProxy('/robot_ClearTrajectory', 
                                                   robot_ClearTrajectory)
        self.execute_trajectory = rospy.ServiceProxy('/robot_ExecuteTrajectory', 
                                                     robot_ExecuteTrajectory)
        self.set_speed = rospy.ServiceProxy('/robot_SetSpeed', 
                                            robot_SetSpeed)
        self.get_ik = rospy.ServiceProxy('/robot1_GetIK', robot_GetIK)
