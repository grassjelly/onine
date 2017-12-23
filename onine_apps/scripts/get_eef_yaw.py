#!/usr/bin/env python
import rospy, tf
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion

tf_listener = tf.TransformListener()

if __name__ == '__main__':
    rospy.init_node('get_eef')

    while not rospy.is_shutdown():
        try:
            t = tf_listener.getLatestCommonTime('/base_footprint', '/wrist_roll_link') # <7>
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue

            (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "wrist_roll_link", t) # <8>
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        (roll,pitch,yaw) = euler_from_quaternion(item_orientation)
        print "ROLL: " + str(roll) 
        print "PITCH: " + str(pitch) 
        print "YAW: " + str(yaw) 
        print "AR TRANSLATION: " + str(item_translation)
    
        break