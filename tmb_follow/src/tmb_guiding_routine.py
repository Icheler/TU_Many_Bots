#! /usr/bin/env python3

import rospy
import math
import tf
import geometry_msgs.msg
from std_srvs.srv import *

'''
tmb_guiding_routine Node

This node is responsible for the grunt of the work in the follow
subroutine. The node listens to the transforms broadcasted by its 
sister node and uses the data, when the conditions in the exploration
phase are met, to guide the blind_robot to the goal position.

The node utilizes THREE Trigger services to give the individual robots 
their roles in the follow routine, the transform listener is used 
afterwards to calculate and publish the required velocity profiles.
The services provide the required chain of events and collision avoidance
in case the robots follow each other too closely.

PUBLISHERS : 
- robot1_cmd_vel : Publishes velocity profile to robot1  [TOPIC : /robot1/cmd_vel -- MSG: Twist]
- robot2_cmd_vel : Publishes velocity profile to robot2  [TOPIC : /robot2/cmd_vel -- MSG: Twist]
- robot_blind_cmd_vel : Publishes velocity profile to the blind robot [TOPIC : /robot_blind/cmd_vel -- MSG: Twist]

SUBSCRIBERS : 
 None
 
SERVICES :

- follow_robot1_service : Indicates following routine request for blind robot to follow robot1 [REQUEST : Trigger -- Callback : trigger_response1]
- follow_robot2_service : Indicates following routine request for blind robot to follow robot2 [REQUEST : Trigger -- Callback : trigger_response2]
- follow_robot_blind_service : Indicates following routine request for the corrective (last) robot to follow blind robot [REQUEST : Trigger -- Callback : trigger_response1]
- is_safe_blind_service : Indicates whether its safe or not for the blind robot to continue moving [REQUEST : Boolean -- Callback : safety_switch_1]
- is_safe_corrective_service : Indicates whether its safe or not for the corrective (last) robot to continue moving [REQUEST : Boolean -- Callback : safety_switch_2]

IMPORTANT VARS :

- leading_robot_ : Global variable for deciding which robot is "active" as the guiding robot. [0 --> NONE , 1 --> Robot1 , 2 --> Robot2]
- corrective_robot_ : Global variable for signalling the corrective robot to follow the blind robot [0 --> OFF , 1 --> ON]
- safe_blind_ : Boolean indicating its safe for the blind robot to move
- safe_corrective_ : Boolean indicating its safe for the corrective robot to move
'''


def trigger_response1(request):
    '''
    Callback for follow_robot1_service
    '''
    global leading_robot_
    leading_robot_ = 1
    return TriggerResponse(
        success=True,
        message="Following robot 1"
    )


def trigger_response2(request):
    '''
    Callback for follow_robot2_service
    '''
    global leading_robot_
    leading_robot_ = 2
    return TriggerResponse(
        success=True,
        message="Following robot 2"
    )

def trigger_response3(request):
    '''
    Callback for follow_robot_blind_service
    '''
    global corrective_robot_
    corrective_robot_ = 1
    return TriggerResponse(
        success=True,
        message="Following blind robot"
    )


def safety_switch1(req):
    '''
    Callback for is_safe_blind_service
    '''
    global safe_blind_
    safe_blind_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def safety_switch2(req):
    '''
    Callback for is_safe_corrective_service
    '''
    global safe_corrective_
    safe_corrective_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res






if __name__ == '__main__':


    global active_, corrective_, safe_blind_, safe_corrective_

    leading_robot_ = 0
    corrective_robot_= 0
    safe_blind_ = True
    safe_corrective_ = True

    rospy.init_node('tmb_guiding_routine')

    listener = tf.TransformListener()
    
    follow_robot1_service = rospy.Service('/follow_rob1', Trigger, trigger_response1)
    follow_robot2_service = rospy.Service('/follow_rob2', Trigger, trigger_response2)
    follow_robot_blind_service = rospy.Service('/follow_blind', Trigger, trigger_response3)
    is_safe_blind_service = rospy.Service('/is_safe_blind', SetBool, safety_switch1)
    is_safe_corrective_service = rospy.Service('/is_safe_corrective', SetBool, safety_switch2)

    robot1_cmd_vel = rospy.Publisher('robot1/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    robot2_cmd_vel = rospy.Publisher('robot2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    robot_blind_cmd_vel = rospy.Publisher('robot_blind/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    
    

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        # Blind robot to follow robot 1
        if leading_robot_ == 1: 

            try:
                '''
                This is a recurring block of code [Need refactoring] that uses the TransformListener to
                compute  transform between the poses of the robots in question and saves them in :

                trans : translational, rot : rotational (trans[1] indicates y, trans[0] indicates x)
                Using the time stamps is necessary for smooth motion planning hence the use of rospy.Time()

                From these transforms the angular and linear velocities are both calculated and sent to the robot
                '''
                (trans, rot) = listener.lookupTransform('/robot_blind', '/robot1', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()
            
            if safe_blind_ and (linear/0.5) > 1.0:
                msg.linear.x = linear
                msg.angular.z = angular
            else:
                msg.linear.x = 0
                msg.angular.z = 0
            robot_blind_cmd_vel.publish(msg)

        # Blind robot to follow robot 2
        elif leading_robot_ == 2: 

            try:
                (trans, rot) = listener.lookupTransform('/robot_blind', '/robot2', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()

            if safe_blind_ and (linear/0.5) > 1.0:
                msg.linear.x = linear
                msg.angular.z = angular
            else:
                msg.linear.x = 0
                msg.angular.z = 0
            robot_blind_cmd_vel.publish(msg)
        


        # Robot 2 follows the blind robot
        if leading_robot_ == 1 and corrective_robot_ == 1: 

            try:
                (trans, rot) = listener.lookupTransform('/robot2', '/robot_blind', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()

            if safe_corrective_ and (linear/0.5) > 1.0:
                msg.linear.x = linear
                msg.angular.z = angular
            else:
                msg.linear.x = 0
                msg.angular.z = 0
            robot2_cmd_vel.publish(msg)

        # Robot 1 follows the blind robot
        elif leading_robot_ == 2 and corrective_robot_ == 1:

            try:
                (trans, rot) = listener.lookupTransform('/robot1', '/robot_blind', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            msg = geometry_msgs.msg.Twist()

            if safe_corrective_ and (linear/0.5) > 1.0:
                msg.linear.x = linear
                msg.angular.z = angular
            else:
                msg.linear.x = 0
                msg.angular.z = 0
            robot1_cmd_vel.publish(msg)




        
        rate.sleep()
