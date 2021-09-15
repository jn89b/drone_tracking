#!/usr/bin env python3

import rospy 
import numpy as np


from math import pi, sqrt
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from threading import Thread

"""
Might need to use inheritance of flight modes or put into submodule  
"""
class FlightModes():
    def __init__(self):
        pass

    def set_arm(self):
        rospy.wait_for_service("mavros/cmd/arming")
        try:
            armService = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            armService(True)
            #print("armed")
        except rospy.ServiceException:
            print("Service arming call failed")

    def set_disarm(self):
        rospy.wait_for_service("mavros/cmd/arming")
        try:
            armService = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            armService(False)
        except rospy.ServiceException:
            print("Service arming call failed")

    def set_offboard_mode(self):
        rospy.wait_for_service("mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy("mavros/set_mode", SetMode)
            flightModeService(custom_mode="OFFBOARD")
            print("OFFBOARD")
        except rospy.ServiceException:
            print ("offboard could not be set")

    def set_land_mode(self):
        rospy.wait_for_service("mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy("mavros/set_mode", SetMode)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException:
            print ("offboard could not be set")

class PositionController():
    def __init__(self):
        # Instantiate a setpoint topic structure
        self.setpoint_ = PositionTarget()

        # use velocity and yaw setpoints
        self.setBodyVelMask()

        # Velocity setpoint by user
        self.vel_setpoint_ = Vector3()

        # Position setpoint by user
        self.pos_setpoint_ = Point()

        # Current local velocity
        self.local_vel_ = TwistStamped()

        # Current body velocity
        self.body_vel_ = TwistStamped()

        # Yaw setpoint by user (degrees); will be converted to radians before it's published
        self.yaw_setpoint_ = 0.0

        # Current drone position (local frame)
        self.drone_pos_ = Point()

        # setpoint publisher (velocity to Pixhawk)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Subscriber for user setpoints (yaw in degrees)
        rospy.Subscriber('setpoint/yaw_deg', Float32, self.yawSpCallback)

        # Subscriber to current drone's position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        # Subscriber to body velocity
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.bodyVelCallback)

        # Subscriber to local velocity
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.localVelCallback)

    def setUp(self):
        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
    
    #send a stream of positions
    def send_pos(self):
        rate = rospy.Rate(20)  # Hz
        #self.pos.header = Header()
        #self.pos.header.frame_id = "base_footprint"
        while not rospy.is_shutdown():
            self.publishSetpoint()
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def publishSetpoint(self):
        self.setpoint_.header.stamp = rospy.Time.now()

        # Only one type of the following setpoints will be consumed based on the type_mask
        """
        self.setpoint_.position.x = self.pos_setpoint_.x
        self.setpoint_.position.y = self.pos_setpoint_.y
        self.setpoint_.position.z = self.pos_setpoint_.z

        self.setpoint_.velocity.x = self.vel_setpoint_.x
        self.setpoint_.velocity.y = self.vel_setpoint_.y
        self.setpoint_.velocity.z = self.vel_setpoint_.z

        self.setpoint_.yaw = self.yaw_setpoint_ * pi / 180. # convert to radians
        """
        self.setpoint_.position.x = 1.0
        self.setpoint_.position.y = 0.0
        self.setpoint_.position.z = 2.0

        self.setpoint_.velocity.x = 0.0
        self.setpoint_.velocity.y = 0.0
        self.setpoint_.velocity.z = 0.0

        self.setpoint_.yaw = 25 * pi / 180. # convert to radians
        self.setpoint_pub_.publish(self.setpoint_)



    def bodyVelCallback(self, msg):
        self.body_vel_ = msg

    def localVelCallback(self, msg):
        self.local_vel_ = msg

    def dronePosCallback(self, msg):
        self.drone_pos_.x = msg.pose.position.x
        self.drone_pos_.y = msg.pose.position.y
        self.drone_pos_.z = msg.pose.position.z

    def velSpCallback(self, msg):
        """
        Velocity setpoint callback
        """
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y
        self.vel_setpoint_.z = msg.z

        self.setBodyVelMask()
    
    def posSpCallback(self, msg):
        """
        Position setpoint callback
        """
        self.pos_setpoint_.x = msg.x
        self.pos_setpoint_.y = msg.y
        self.pos_setpoint_.z = msg.z

        self.setLocalPositionMask()

    def yawSpCallback(self, msg):
        """
        Yaw setpoint callback
        """
        self.yaw_setpoint_ = msg.data


    def setBodyVelMask(self):
        """
        Sets type_mask for velocity setpoint in body frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setLocalVelMask(self):
        """
        Sets type_mask for velocity setpoint in local frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE


class TrackTarget(FlightModes):
    def __init__(self):
        #inheriting from flightmode
        #self.setpoint_ = PositionTarget()
        super().__init__()
        
        # setpoint of local frame
        self.sp_local_x = 0.0
        self.sp_local_y = 0.0
        self.sp_local_z = 0.0

        #relative setpoints WRT(With Respect To)
        self.sp_relative_x = 0.0
        self.sp_relative_y = 0.0
        self.sp_relative_z = 0.0

         # Controller object to calculate velocity commands
        self.controller_ = PositionController()
       

if __name__ == '__main__':
    rospy.init_node("offboard_drone", anonymous=True)
    rate = rospy.Rate(20)
    #flightmode = FlightModes()
    tracktarget = TrackTarget()
    while not rospy.is_shutdown():
        tracktarget.controller_.setUp()
        rate.sleep()