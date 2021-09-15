#!/usr/bin env python3

import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Bool

"""
Class precision listens to drone and hears a request from drone to land
if it does request a land -> probably make this a service? allow permission
"""

class PrecLand():

    def __init__(self):
        self.pub = rospy.Publisher("precland", Bool, queue_size=10)
        self.sub = rospy.Subscriber("target_found", Bool, self.target_foundcb)
        quad_odom_sub = rospy.Subscriber("mavros/offset_local_position/pose", PoseStamped, self.quad_odom_cb)
        self.target_found = False
        self.allow_land = Bool()

        self.z = 0.0

    def target_foundcb(self,msg):
        self.target_found = msg.data

    def quad_odom_cb(self,msg):
        z = msg.pose.position.z

    #need to make sure that we quad is also stablized and that the error of
    #tag and drone is within tolerance to allow safe landing
    def check_permission(self):
        if self.target_found == True or self.z < 0.8: #probably need to set this better 
            self.allow_land.data = True
            self.pub.publish(self.allow_land)
        else: 
            self.allow_land.data = False
            self.pub.publish(self.allow_land)
    
    
if __name__=='__main__':
    rospy.init_node("land_permission", anonymous=True)
    
    rate_val = 30
    rate = rospy.Rate(rate_val)
    precland = PrecLand()

    while not rospy.is_shutdown():
        precland.check_permission()
        rate.sleep()
