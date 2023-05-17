#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Pose

#API
from xarm.wrapper import XArmAPI

""" 
#######################################################
# Helper class for xArm interface with SDK 

# input: none
# output: none

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 

class xArm_Motion():
    def __init__(self, ip_addr):
        print(f" ---- creating xArm_Wrapper for ip {ip_addr}----")
        # self.stuff = 0
        self.ip = ip_addr

    def initialize_robot(self):
        print(" ---- initializing robot ----")
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

    




    
if __name__ == "__main__":
    print(" ================ testing main of xArm_Motion wrapper ============ ")