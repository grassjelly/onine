#!/usr/bin/env python
import rospy, tf
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion

tf_listener = tf.TransformListener() 

if __name__ == '__main__':
    rospy.init_node('get_ar')

    while not rospy.is_shutdown():
        try:
            t = tf_listener.getLatestCommonTime('/torso_link', '/ar_marker_3') 
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue

            (item_translation, item_orientation) = tf_listener.lookupTransform('/torso_link', "ar_marker_3", t) 
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        (roll,pitch,yaw) = euler_from_quaternion(item_orientation)

        print "ROLL: " + str(roll) 
        print "PITCH: " + str(pitch) 
        print "YAW: " + str(-1.570796 + yaw) 

        try:
            t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') 
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue

            (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print "AR TRANSLATION: " + str(item_translation)
    
        break