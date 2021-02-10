#! /usr/bin/env python3

import rospy
import math
import tf
import geometry_msgs.msg
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse




def trigger_response1(request):
    
    global active_
    active_ = 1
    return TriggerResponse(
        success=True,
        message="Following robot 1"
    )


def trigger_response2(request):
    
    global active_
    active_ = 2
    return TriggerResponse(
        success=True,
        message="Following robot 2"
    )

def trigger_response3(request):
    
    global active2_
    active2_ = 1
    return TriggerResponse(
        success=True,
        message="Following blind robot"
    )


if __name__ == '__main__':


    global active_
    active_ = 0
    active2_= 0

    rospy.init_node('rto_tf_listener')


    
    my_service = rospy.Service('/follow_rob1', Trigger, trigger_response1)
    my_service_2 = rospy.Service('/follow_rob2', Trigger, trigger_response2)
    my_service_3 = rospy.Service('/follow_blind', Trigger, trigger_response3)

    listener = tf.TransformListener()
    robot_vel = rospy.Publisher('robot_blind/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    robot_vel_2 = rospy.Publisher('robot2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    robot_vel_22 = rospy.Publisher('robot1/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        # Here decide which robot the blind_robot should follow
        if active_ == 1:

            try:
                (trans, rot) = listener.lookupTransform('/robot_blind', '/robot1', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            robot_vel.publish(msg)

        elif active_ == 2:

            try:
                (trans, rot) = listener.lookupTransform('/robot_blind', '/robot2', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            robot_vel.publish(msg)
        
        # Then if we already have the follow, then the other robot should follow the blind one, once certain criteria is met

        if active_ == 1 and active2_ == 1:

            try:
                (trans, rot) = listener.lookupTransform('/robot2', '/robot_blind', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            robot_vel_2.publish(msg)

        elif active_ == 2 and active2_ == 1:

            try:
                (trans, rot) = listener.lookupTransform('/robot1', '/robot_blind', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            robot_vel_22.publish(msg)




        
        rate.sleep()
