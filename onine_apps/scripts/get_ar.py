#!/usr/bin/env python
import rospy, tf
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

tf_listener = tf.TransformListener() 
     
if __name__ == '__main__':
    rospy.init_node('get_ar')

    while not rospy.is_shutdown():
        try:
            t = tf_listener.getLatestCommonTime('/base_footprint', '/torso_link') 
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue

            (origin_translation, origin_orientation) = tf_listener.lookupTransform('/base_footprint', "torso_link", t) 
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print "ORIGIN TRANSLATION: " + str(origin_translation)

        try:
            t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') 
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue

            (target_translation, target_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print "TARGET TRANSLATION: " + str(target_translation)
    
        # distance = calculateDistance(origin_translation[0], origin_translation[1], target_translation[0], target_translation[1])
        x = target_translation[0] - origin_translation[0]
        y = target_translation[1] - origin_translation[1]

        print "X: " + str(x)
        print "Y: " + str(y)
        if(target_translation[1] > 0):
            yaw = math.atan(x/y)
        elif(target_translation[1] < 0):
            yaw = (math.pi / 2) - math.atan(x/y)
        print "YAW: " + str(-1 * yaw)

        break