#!/usr/bin env python3

import rospy
import tf
import numpy as np
import threading
import concurrent.futures

from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Bool, Int8

"""
Class precision listens to drone and hears a request from drone to land
if it does request a land -> probably make this a service? allow permission
"""

class UserControl():

    def __init__(self):

        """
        1 = TRACK
        2 = PRECLAND
        3 = LAND
        4 = DISARM 
        5 = WAYPOINT
        """
        self.user_cmds_dict = {
            "TRACK": self.track_cmd,
            "PRECLAND": self.precland_cmd,  
            "LAND": self.land_cmd,
            "DISARM": self.disarm_cmd,
            "WAYPOINT": self.waypoint_cmd
        }
        self.cmd = None

        self.user_control_pub = rospy.Publisher("user_control", Int8, queue_size= 10)
        self.pub = rospy.Publisher("precland", Bool, queue_size=10)
        self.sub = rospy.Subscriber("target_found", Bool, self.target_foundcb)
        quad_odom_sub = rospy.Subscriber("mavros/offset_local_position/pose", PoseStamped, self.quad_odom_cb)
        
        self.user_input = Int8()
        
        self.target_found = False
        self.allow_land = Bool()
        self.z = 0.0

    def target_foundcb(self,msg):
        self.target_found = msg.data

    def quad_odom_cb(self,msg):
        z = msg.pose.position.z

    #### COMMAND PROTOCOLS ##############################################
    def track_cmd(self):
        self.user_input.data = 1
        #print("im tracking")
        self.user_control_pub.publish(self.user_input)

    #need to make sure that we quad is also stablized and that the error of
    #tag and drone is within tolerance to allow safe landing
    def precland_cmd(self):
        self.user_input.data = 2
        self.user_control_pub.publish(self.user_input)
        self.check_permission

    def check_permission(self):
        self.user_control_pub.publish(self.user)
        if self.target_found == True or self.z < 0.8: #probably need to set this better 
            self.allow_land.data = True
            self.pub.publish(self.allow_land)
        else: 
            self.allow_land.data = False
            self.pub.publish(self.allow_land)

    def land_cmd(self):
        print("I'm landing")

    def disarm_cmd(self):
        print("I'm disarming")

    def waypoint_cmd(self):
        print("Waypoint cmd")


    def run(self):
        rate = 30
        rate = rospy.Rate(rate)

        self.cmd = input("Enter your command: ")
        
        if self.cmd in self.user_cmds_dict:
            print("Starting command", self.cmd, ",to exit press ENTER")

            while True:
                try:
                    self.user_cmds_dict.get(self.cmd)()
                    rate.sleep()
                except KeyboardInterrupt:
                    break  # The answer was in the question!

        else:
            print ("Wrong input, commands are: %s" %  self.user_cmds_dict.keys())
 
def main():
    usercontrol = UserControl()
    usercontrol.run()

if __name__=='__main__':
    rospy.init_node("user_control", anonymous=True, disable_signals=True)
    while True:
        main()    
   
