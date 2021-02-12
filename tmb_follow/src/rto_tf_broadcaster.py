#!/usr/bin/env python3


import rospy
import tf
from geometry_msgs.msg import Pose
from tmb_messages.msg import Computed_Pose
from tf import transformations


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
- tmb_computed_pose whichh reads the data from the perception estimated pose  


'''




def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    yaw = msg.yaw
    if robotname == "robot_blind":
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
