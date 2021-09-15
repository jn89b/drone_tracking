#!/usr/bin/env python3
import roslib
import rospy
import tf

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped

# MAVROS TF broadcasts the local position frame of the quad wrt to map
class MavrosTF():

    def __init__(self):
        #init tf broacast
        self.br = tf.TransformBroadcaster()
        #init transformation we will broadcast 
        self.new_tf = rospy.get_param("~quad_tf", "/drone0_wrt_world")
        #source tf we will want to make a transformation of Rnew_tf/source_tf
        self.source_tf = rospy.get_param("~world_tf", "world_enu")
        #offset from airsim 
        self.offset_z = rospy.get_param("~offset_z", 0.6)
        # Current drone position (local frame)
        self.drone_pos = [0,0,0,0,0,0,0]
        # Subscriber to current drone's position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        self.pub = rospy.Publisher("mavros/offset_local_position/pose",PoseStamped, queue_size=10)
    
    def dronePosCallback(self, msg):
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = (msg.pose.position.z)  - self.offset_z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        now = rospy.Time.now()

        #publish posestamped
        posestamped = PoseStamped()
        posestamped.header.frame_id = "drone_offset"
        posestamped.header.stamp = now
        posestamped.pose.position.x = x
        posestamped.pose.position.y = y
        posestamped.pose.position.z = z

        posestamped.pose.orientation.x = qx
        posestamped.pose.orientation.y = qy
        posestamped.pose.orientation.z = qz
        posestamped.pose.orientation.w = qw

        self.pub.publish(posestamped)

        #send transformation 
        self.br.sendTransform((x,y,z),(qx,qy,qz,qw),now,self.new_tf, self.source_tf)
        
        #print(self.drone_pos)

    #convert transformation R mavros/px4 odom
    def broadcast_drone_tf(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            rate.sleep()

            
if __name__ == '__main__':
    # Initiate node
    rospy.init_node("mavros_tf", anonymous=True)
    mavrostf = MavrosTF()
    mavrostf.broadcast_drone_tf()


