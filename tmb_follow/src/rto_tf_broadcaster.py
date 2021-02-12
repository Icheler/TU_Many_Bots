#!/usr/bin/env python3

import rospy

import tf
from geometry_msgs.msg import Pose
from tmb_messages.msg import Computed_Pose
from tf import transformations


def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    yaw = msg.yaw
    if robotname == "robot_blind":
        rospy.loginfo(yaw)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    br.sendTransform((msg.x, msg.y, 0),
                     quaternion,
                     rospy.Time.now(),
                     robotname,
                     "map")

if __name__ == '__main__':
    rospy.init_node('rto_tf_broadcaster')
    robot_name = rospy.get_param('~robot')
    rospy.Subscriber(f'/{robot_name}/tmb_computed_pose',  Computed_Pose,
                     handle_robot_pose,
                     robot_name)
    rospy.spin()
