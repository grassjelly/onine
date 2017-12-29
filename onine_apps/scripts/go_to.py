#!/usr/bin/env python
import os, sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import math
class Onine():

    def __init__(self, arm_group):
        self.arm_group = arm_group
        self.tf_listener = tf.TransformListener() 
        self.p = Pose()

    def goto(self, x, y, z, yaw):
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, yaw))
        self.arm_group.set_pose_target(self.p)
        os.system("rosservice call clear_octomap") # <11>
        # plan1 = self.arm_group.plan()
        # self.arm_group.execute(plan1)
        self.arm_group.go(wait=True)


    def get_grasp_pose(self, x, y, z, distance):
        origin_translation = [0.095, 0.00, 0.00]

        delta_x = x - origin_translation[0]
        delta_y = y - origin_translation[1]

        if(y > 0):
            theta = math.atan(delta_x / delta_y)
            grasp_yaw = -1.00 * theta
            grasp_x = x - (distance * math.sin(grasp_yaw))
            grasp_y = y + (distance * math.cos(grasp_yaw))

        elif(y < 0):
            theta = math.atan(delta_x / delta_y)
            grasp_yaw = -1.00 * ((math.pi / 2.00) - theta)
            grasp_x = x - (distance * math.sin(theta))
            grasp_y = y + (distance * math.cos(theta))

        return (grasp_x, grasp_y, z, grasp_yaw)

    def ready(self):
        self.arm_group.set_named_target("onine_ready")
        os.system("rosservice call clear_octomap") # <11>
        self.arm_group.go(wait=True)

    def open_gripper(self):
        os.system("rostopic pub /onine_gripper std_msgs/Bool 1 -1")

    def close_gripper(self):
        os.system("rostopic pub /onine_gripper std_msgs/Bool 0 -1")

    def pickup(self, x, y, z):
        self.ready()
        self.open_gripper()
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_grasp_pose(x, y, z, -0.10)
        self.goto(aim_x, aim_y, aim_z, aim_yaw)
        self.goto(x, y, z, aim_yaw)
        self.close_gripper()

if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_up_item')

  arm_group = moveit_commander.MoveGroupCommander("onine_arm") 
  arm_group.allow_replanning(True)
  rate = rospy.Rate(10)

  tf_listener = tf.TransformListener() 

  while not rospy.is_shutdown():
    rate.sleep()

    # try:
    #   t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') # <7>
    #   if (rospy.Time.now() - t).to_sec() > 0.2:
    #     continue

    #   (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
    # except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     continue

    # print item_translation

    # yaw =  -1.70223872081
    # item_translation =  [0.3263662327879304, -0.030257178712262592, 0.8940227682404134]

    # yawd =  -0.96482721836
    # item_translation =  [0.3451770476444722, 0.1716242817246255, 0.7486642342494648]

    # yaw = -0.949421004148
    item_translation = [0.33292386367734217, 0.1685605027519197, 0.8339949674141176]

    arm_group.set_goal_tolerance(0.005)
    # arm_group.set_goal_position_tolerance(0.05)
    # arm_group.set_goal_orientation_tolerance(0.1)
    arm_group.set_num_planning_attempts(10)
    arm_group.set_planning_time(10)
    arm_group.set_planner_id("RRTkConfigDefault")

    onine_arm = Onine(arm_group)

    onine_arm.pickup(item_translation[0], item_translation[1], item_translation[2])
    break 