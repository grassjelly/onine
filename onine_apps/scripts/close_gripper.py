#!/usr/bin/env python
import os, sys, rospy, tf, math
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

class Onine():

    def __init__(self, arm, gripper):
        self.tf_listener = tf.TransformListener() 
        self.p = Pose()

        self.gripper = gripper
        self.arm = arm

        # arm.set_goal_tolerance(0.001)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_num_planning_attempts(30)
        self.arm.set_planning_time(15)
        self.arm.set_planner_id("RRTkConfigDefault")

    def go(self, x, y, z, yaw):
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation = Quaternion(*quaternion_from_euler(0.0, 0, yaw))
        self.arm.set_pose_target(self.p)
        # plan1 = self.arm.plan()
        # self.arm.execute(plan1)
        os.system("rosservice call clear_octomap")
        self.arm.go(wait=True)


    def get_valid_pose(self, x, y, z, distance):
        origin_translation = [0.095, 0.00, 0.00]

        delta_x = x - origin_translation[0]
        delta_y = y - origin_translation[1]

        theta = math.atan(delta_y / delta_x)
        grasp_yaw = theta
        grasp_x = x + (distance * math.cos(theta))
        grasp_y = y + (distance * math.sin(theta))

        print grasp_x
        print grasp_y
        print z
        print grasp_yaw
        return (grasp_x, grasp_y, z, grasp_yaw)

    def ready(self):
        self.arm.set_named_target("onine_ready")
        self.arm.go(wait=True)
        os.system("rosservice call clear_octomap")

    def home(self):
        self.arm.set_named_target("onine_home")
        self.arm.go(wait=True)
        os.system("rosservice call clear_octomap")

    def open_gripper(self):
        # self.gripper.set_named_target("gripper_open")
        os.system("rostopic pub /onine_gripper std_msgs/Float64 0.085 -1")
        os.system("rosservice call clear_octomap")
        # self.gripper.go()
        # rospy.sleep(2)

    def close_gripper(self):
        # self.gripper.set_named_target("gripper_closed")
        os.system("rostopic pub /onine_gripper std_msgs/Float64 0.045 -1")
        os.system("rosservice call clear_octomap")
        # self.gripper.go()
        # rospy.sleep(2)

    def pickup_sim(self, x, y, z):
        self.ready()
        self.open_gripper()
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, -0.10)
        self.go(aim_x, aim_y, aim_z, aim_yaw)
        self.go(x, y, z, aim_yaw)
        self.close_gripper()

if __name__ == '__main__':
    rospy.init_node('pick_up_item')

    tf_listener = tf.TransformListener() 
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    gripper = MoveGroupCommander("onine_gripper")
    arm = MoveGroupCommander("onine_arm") 

    rate = rospy.Rate(10)

    rospy.sleep(2)

    while not rospy.is_shutdown():
        rate.sleep()
        
        onine_arm = Onine(arm, gripper)
        onine_arm.close_gripper()

        break 