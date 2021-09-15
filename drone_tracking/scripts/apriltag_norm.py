#!/usr/bin/env python3
import roslib
import rospy
import tf

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped

from apriltag_ros.msg import AprilTagDetectionArray


class ApriltagNorm():
    def __init__(self):

        self.tag_id_ = rospy.get_param("~tag_id",0)
        self.new_tf = rospy.get_param("~norm_tag_tf", "/tag_0")
        self.source_tf = rospy.get_param("~cam_tf", "front_center_custom_optical")       
        #self.drone_frame_id_ = rospy.get_param("~quad_tf", "/drone0_wrt_world")
        self.tags_topic_ = rospy.get_param('~tags_topic', '/tag_detections')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')
        rospy.Subscriber(self.tags_topic_, AprilTagDetectionArray, self.tagsCallback)
        self.tag_norm_pub = rospy.Publisher("tag_norm", PoseStamped,queue_size=10)

        self.br = tf.TransformBroadcaster()

    # tags callback
    def tagsCallback(self, msg):
        valid = False
        trans = []
        scale_factor = 10
        if len(msg.detections) > 0: # make sure detection is valid
            overall_pose = msg.detections[0].pose.pose.pose
            x = overall_pose.position.x/scale_factor
            y = overall_pose.position.y/scale_factor
            z = overall_pose.position.z/scale_factor
            qx = overall_pose.orientation.x
            qy = overall_pose.orientation.y
            qz = overall_pose.orientation.x
            qw = overall_pose.orientation.z
            
            #(trans,rot) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
            #print(trans,rot)
            valid = True
        else:
            rospy.logwarn("No valid TF for the required tag %s", self.tag_id_)
            return

        if valid:
            print(x,y,z) 
            now = rospy.Time.now()
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.new_tf
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z 
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self.tag_norm_pub.publish(pose_msg)
            #sending transform rtagwrtdrone Rtag/drone
            #self.br.sendTransform((trans[0]/scale_factor,trans[1]/scale_factor, trans[2]- camera_offset_z),(rot[0],rot[1],rot[2],rot[3]),now,self.new_tf, self.drone_frame_id_)
            self.br.sendTransform((x,y,z),(0,0,0,1),now,self.new_tf,  self.source_tf)
        else:
            pass


if __name__ == '__main__':
    rospy.init_node('apriltag_norm', anonymous=True)

    sp_o = ApriltagNorm()

    rospy.loginfo("intialize apriltag norm")

    rospy.spin()
