#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry


def handle_pose(msg, robot):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     robot,
                     "world")

if __name__ == '__main__':
    rospy.init_node('robot0_tf_broadcaster')
    rospy.Subscriber('/robot_0/odom',
                     Odometry,
                     handle_pose,
                     'robot_0')
    rospy.spin()
