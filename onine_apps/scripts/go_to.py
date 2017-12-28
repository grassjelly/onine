#!/usr/bin/env python
import os, sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion

if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_up_item')

  arm = moveit_commander.MoveGroupCommander("onine_arm") 
  arm.allow_replanning(True)
  tf_listener = tf.TransformListener() 
  rate = rospy.Rate(10)

  p = Pose()

  while not rospy.is_shutdown():
    rate.sleep()
    # try:
    #   t = tf_listener.getLatestCommonTime('/torso_link', '/ar_marker_3') 
    #   if (rospy.Time.now() - t).to_sec() > 0.2:
    #     rospy.sleep(0.1)
    #     continue

    #   (item_translation, item_orientation) = tf_listener.lookupTransform('/torso_link', "ar_marker_3", t) 
    # except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     continue
    
    
    # (roll,pitch,yaw) = euler_from_quaternion(item_orientation)
    # yaw = -1.57079632679 + yaw

    # try:
    #   t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') # <7>
    #   if (rospy.Time.now() - t).to_sec() > 0.2:
    #     rospy.sleep(0.1)
    #     continue

    #   (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
    # except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     continue

    # yaw =  -1.70223872081
    # item_translation =  [0.3263662327879304, -0.030257178712262592, 0.8940227682404134]
    yaw =  -0.964827218361
    item_translation =  [0.3451770476444722, 0.1716242817246255, 0.7486642342494648]






    print yaw
    print item_translation

    p.position.x = item_translation[0]
    p.position.y = item_translation[1] 
    p.position.z = item_translation[2] 
    p.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, yaw))

    arm.set_pose_target(p)
    arm.set_goal_tolerance(0.001)
    # arm.set_goal_position_tolerance(0.05)
    # arm.set_goal_orientation_tolerance(0.1)
    arm.set_num_planning_attempts(10)
    arm.set_planning_time(10)
    arm.set_planner_id("RRTkConfigDefault")
    os.system("rosservice call clear_octomap") # <11>

    plan1 = arm.plan()

    arm.execute(plan1)

    break 