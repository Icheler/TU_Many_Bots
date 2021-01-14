#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Point


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    position = Point()
    listener = tf.TransformListener()

    pub = rospy.Publisher('position', Point, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('robot1/map', 'robot1/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        position.x = trans[0]
        position.y = trans[1]
        position.z = trans[2]

        pub.publish(position)

        rate.sleep()