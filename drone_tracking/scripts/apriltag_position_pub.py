#!/usr/bin env python3

import rospy 
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagPositionPub():
    def __init__(self):
        #apriltag id 
        self.tag_id_ = rospy.get_param("~tag_id",0)
        self.new_tf = rospy.get_param("~rtag_drone", "/tag_wrt_drone0")       
        self.drone_frame_id_ = rospy.get_param("~quad_tf", "/drone0_wrt_world")
        self.tag_frame_id_ = rospy.get_param("~tag_frame_id", "/tag_0")
        self.tags_topic_ = rospy.get_param('~tags_topic', '/tag_detections')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')
        self.tf_listener_ = tf.TransformListener()
        #broadcast tf transform
        self.br = tf.TransformBroadcaster()
        #init transformation we will broadcast 
 
        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0

        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=1)

        # Relative pose publisher as PoseStamped
        self.pose_pub_ = rospy.Publisher('tag/pose', PoseStamped, queue_size=1)

        #boolean statement 
        self.target_pub = rospy.Publisher("/target_found", Bool, queue_size=1)
        # Subscriber to Kalman filter estimate
        #rospy.Subscriber("kf/estimate", PoseWithCovarianceStamped, self.kfCallback)

        rospy.Subscriber(self.tags_topic_, AprilTagDetectionArray, self.tagsCallback)

    # tags callback
    def tagsCallback(self, msg):
        valid = False
        trans = []
        scale_factor = 1.0
        camera_offset_z = 0.5  
        if len(msg.detections) > 0: # make sure detection is valid
            overall_pose = msg.detections[0].pose.pose.pose
            x = overall_pose.position.x/scale_factor
            y = overall_pose.position.y/scale_factor
            z = overall_pose.position.z/scale_factor
            (trans,rot) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
            #print(trans,rot)
            target_found = Bool()
            target_found.data = True
            self.target_pub.publish(target_found)
            valid = True
        else:
            target_found = Bool()
            target_found.data = False
            self.target_pub.publish(target_found) 
            rospy.logwarn("No valid TF for the required tag %s", self.tag_id_)
            return

        if valid: # Publish relative setpoint
            
            now = rospy.Time.now()
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.new_tf
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = trans[0]/scale_factor
            pose_msg.pose.position.y = (trans[1]/scale_factor)
            pose_msg.pose.position.z = trans[2] - camera_offset_z
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]
            self.pose_pub_.publish(pose_msg)
            #sending transform rtagwrtdrone Rtag/drone
            self.br.sendTransform((trans[0]/scale_factor,trans[1]/scale_factor, trans[2]- camera_offset_z),(rot[0],rot[1],rot[2],rot[3]),now,self.new_tf, self.drone_frame_id_)
            
        else:
            pass
                
if __name__ == '__main__':
    rospy.init_node('apriltag_position_pub', anonymous=True)

    rate_val = 10
    #rate = rospy.Rate(rate_val)
    
    sp_o = AprilTagPositionPub()
    rospy.spin()
    #while not rospy.is_shutdown():

        #sp_o = AprilTagPositionPub()

        #rospy.loginfo("tag_setpoint_publisher_node is started")

        #rate.sleep()
