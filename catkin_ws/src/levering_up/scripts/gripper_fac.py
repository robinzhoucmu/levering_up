import robotiq_gripper
import wsg_gripper
from wsg_50_common.srv import *

class Gripper(object):
    def factory(type):
        if type == "wsg50": return WSG50Gripper()
        if type == "robotiq2finger": return Robotiq2FingerGripper()

class WSG50Gripper(Gripper):

class Robotiq2FingerGripper(Gripper): 
