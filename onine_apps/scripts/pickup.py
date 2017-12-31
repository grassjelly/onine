#!/usr/bin/env python
import os, sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import PickupAction, PickupGoal
from actionlib import SimpleActionClient, GoalStatus
from manipulation_msgs.msg import Grasp

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
        # plan1 = self.arm_group.plan()
        # self.arm_group.execute(plan1)
        self.arm_group.go(wait=True)
        os.system("rosservice call clear_octomap")


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
            grasp_y = y - (distance * math.cos(theta))
        print grasp_x
        print grasp_y
        print z
        print grasp_yaw
        return (grasp_x, grasp_y, z, grasp_yaw)

    def ready(self):
        self.arm_group.set_named_target("onine_ready")
        self.arm_group.go(wait=True)
        os.system("rosservice call clear_octomap")

    def open_gripper(self):
        os.system("rostopic pub /onine_gripper std_msgs/Bool 1 -1")

    def close_gripper(self):
        os.system("rostopic pub /onine_gripper std_msgs/Bool 0 -1")

    def pickup(self, x, y, z):
        self.ready()
        self.open_gripper()
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_grasp_pose(x, y, z, -0.15)
        self.goto(aim_x, aim_y, aim_z, aim_yaw)
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_grasp_pose(x, y, z, -0.10)
        self.goto(x, y, z, aim_yaw)
        self.close_gripper()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_up_item')

    arm_group = moveit_commander.MoveGroupCommander("onine_arm") 
    arm_group.allow_replanning(True)
    rate = rospy.Rate(10)

    tf_listener = tf.TransformListener() 

    scene = PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    while not rospy.is_shutdown():
        rate.sleep()

        # try:
        #   t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') # <7>
        #   if (rospy.Time.now() - t).to_sec() > 0.2:
        #     continue

        #   (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
        # except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        #left test
        yaw = -0.949421004148
        item_translation = [0.33292386367734217, 0.1685605027519197, 0.8339949674141176]
        
        #right test
        # yaw =  -2.33954420079
        # item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]
        # scene.remove_world_object("target") 

        # p = PoseStamped()
        # p.header.frame_id = robot.get_planning_frame()
        # p.pose.position.x = item_translation[0]
        # p.pose.position.y = item_translation[1]
        # p.pose.position.z = item_translation[2]
        # q = quaternion_from_euler(0.0, 0.0, 0.0)
        # p.pose.orientation = Quaternion(*q)

        # scene.add_box("target", p, (0.01, 0.01, 0.09))

        # # arm_group.set_goal_tolerance(0.001)
        arm_group.set_goal_position_tolerance(0.005)
        arm_group.set_goal_orientation_tolerance(0.1)
        arm_group.set_num_planning_attempts(30)
        arm_group.set_planning_time(10)
        arm_group.set_planner_id("RRTkConfigDefault")

        onine_arm = Onine(arm_group)
        onine_arm.pickup(item_translation[0], item_translation[1], item_translation[2])

        # pickup_ac = SimpleActionClient('/pickup', PickupAction)
        # if not pickup_ac.wait_for_server(rospy.Duration(5.0)):
        #     rospy.logerr('Pick up action client not available!')
        #     rospy.signal_shutdown('Pick up action client not available!')
        # else:
        #     print('Action available')




        # # Create goal:
        # goal = PickupGoal()

        # goal.group_name  = "onine_arm"
        # goal.target_name = "target"

        # # goal.possible_grasps.extend(grasps)

        # goal.allowed_touch_objects.append("target")

        # # goal.support_surface_name = self._table_object_name

        # # Configure goal planning options:
        # goal.allowed_planning_time = 7.0

        # goal.planning_options.planning_scene_diff.is_diff = True
        # goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        # goal.planning_options.plan_only = False
        # goal.planning_options.replan = True
        # goal.planning_options.replan_attempts = 20
        
        # onine_arm = Onine(arm_group)
        # (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_grasp_pose(item_translation[0], item_translation[1], item_translation[2], -0.10)

        # grasp = Grasp() 
        # grasp.id = "blah" 
        # grasp.grasp_pose.header.frame_id = "wrist_roll_link" 
        # grasp.grasp_pose.header.stamp = rospy.Time.now() 
        # grasp.grasp_pose.pose.position.x = aim_x
        # grasp.grasp_pose.pose.position.y = aim_y
        # grasp.grasp_pose.pose.position.z = aim_z
        # # grasp.grasp_pose.pose.orientation.w = .7 
        # # grasp.grasp_pose.pose.orientation.x = -.08 
        # # grasp.grasp_pose.pose.orientation.y = .7 
        # # grasp.grasp_pose.pose.orientation.z = -.08 
        
        # grasp.grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, aim_yaw))
        # grasp.allowed_touch_objects = ["part"]
        # # goal.possible_grasps.append(grasp) 

        # robot.onine_arm.pick("target")


        # state = pickup_ac.send_goal_and_wait(goal)
        # rospy.sleep(1)
        # if state != GoalStatus.SUCCEEDED:
        #     rospy.logerr('Pick up goal failed!: %s' % pickup_ac.get_goal_status_text())


        # result = pickup_ac.get_result()

        # # Check for error:
        # err = result.error_code.val
        # if err != MoveItErrorCodes.SUCCESS:
        #     rospy.logwarn("CANNOT PICKUP TARGET")

        break 


 
#https://groups.google.com/forum/#!topic/moveit-users/7hzzICsfLOQ