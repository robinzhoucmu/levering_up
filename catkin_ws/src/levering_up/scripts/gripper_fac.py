import wsg_gripper
from wsg_50_common.srv import *

class Gripper(object):
    def factory(type):
        if type == "wsg50": return WSG50Gripper()
        if type == "robotiq2finger": return Robotiq2FingerGripper()
        assert 0, "Bad gripper type: " + type
        factory = staticmethod(factory)

class WSG50Gripper(Gripper):
    def __init__(self):
    def init_ros_services(self):
        home_gripper = rospy.ServiceProxy('/wsg_50_driver/homing', Empty)
        close_gripper = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
        open_gripper = rospy.ServiceProxy('/wsg_50_driver/release', Move)
        set_gripper_force = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
        
class Robotiq2FingerGripper(Gripper): 
    def __init__(self):
    def init_ros_services(self):
    
