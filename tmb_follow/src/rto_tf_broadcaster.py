#!/usr/bin/env python3

import rospy

import tf
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry



def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     quaternion,
                     rospy.Time.now(),
                     robotname,
                     "map")

if __name__ == '__main__':
    rospy.init_node('rto_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    rospy.Subscriber('/%s/odom' % robotname,
                     Odometry,
                     handle_robot_pose,
                     robotname)
    rospy.spin()
