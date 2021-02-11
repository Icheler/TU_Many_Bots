#!/usr/bin/env python3

'''
RTO_BROADCASTER NODE

This node play the role of reading the position estimates of the robots
including the blind_robot [via odom] and re-broadcasts them as tf frames
relative to the Global Map Topic: /map

The launch file follow_rto.launch assigns the frames for all the robots
in the simulation, visit to adjust accordingly.

The broadcaste subscribes to the respective robots namespace /odom topic
and message type Odometry. 

MAKE SURE : Change the Subscriber on line 43 and msg definitions on line
35 to change the localization data origin. 

------------------------------------------------------------------------

PUBLISHERS : 

 None

SUBSCRIBERS : 

- Odometry 

ROSPARAMS : 

- ~robot

'''

import rospy
import tf
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry



def handle_robot_pose(msg, robotname):
    '''
    Callback for the Localization subscriber [Uses ~/odom]

    Transforms are taken from pose and sent relative to /map
    '''
    
    # Set the parent frame name
    parent_frame = "map"
    br = tf.TransformBroadcaster()

    # Can send here yaw directly [depending on message definition] and change to quaternion via tf.transforms
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0), quaternion, rospy.Time.now(), robotname, parent_frame)


if __name__ == '__main__':

    rospy.init_node('rto_tf_broadcaster')
    robotname = rospy.get_param('~robot')   

    #Subscriber to odom via the handle callback
    rospy.Subscriber('/%s/odom' % robotname, Odometry, handle_robot_pose, robotname) 
    rospy.spin()
