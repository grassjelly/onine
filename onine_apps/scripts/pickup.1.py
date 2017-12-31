#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import *

import tf
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
        os.system("rosservice call clear_octomap")
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
            grasp_y = y - (distance * math.cos(theta))
        else:
            grasp_yaw = math.pi / 2
            grasp_x = x - distance
            grasp_y = 0.00

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
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_grasp_pose(x, y, z, -0.10)
        self.goto(aim_x, aim_y, aim_z, aim_yaw)
        print aim_x
        print aim_y
        print aim_z
        print aim_yaw
        self.goto(x, y, z, aim_yaw)
        self.close_gripper()

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("onine_arm")
    right_gripper = MoveGroupCommander("onine_gripper")
    rospy.sleep(1)

    # clean the scene
    # scene.remove_world_object("table")
    scene.remove_world_object("part")

    arm.set_named_target("onine_ready")
    arm.go()
    
    right_gripper.set_named_target("gripper_open")
    right_gripper.go()
    
    rospy.sleep(1)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # item_translation = [0.33292386367734217, 0.1685605027519197, 0.8]
    item_translation = [0.33292386367734217, 0.00, 0.8]

    # add a table
    # p.pose.position.x = 0.52
    # p.pose.position.y = -0.2
    # p.pose.position.z = 0.35
    # scene.add_box("table", p, (0.5, 1.5, 0.7))

    # add an object to be grasped
    # p.pose.position.x = item_translation[0]
    # p.pose.position.y = item_translation[1]
    # p.pose.position.z = item_translation[2]
    # scene.add_box("part", p, (0.01, 0.01, 0.2))

    rospy.sleep(1)

    onine_arm = Onine(arm)
    (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_grasp_pose(item_translation[0], item_translation[1], item_translation[2], -0.20)

    grasps = []
    

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_footprint"
    grasp_pose.pose.position.x = aim_x
    grasp_pose.pose.position.y = aim_y
    grasp_pose.pose.position.z = aim_z
    grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, aim_yaw))

    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.1)
    arm.set_num_planning_attempts(30)
    arm.set_planning_time(15)
    arm.set_planner_id("RRTkConfigDefault")
    arm.set_pose_target(grasp_pose)
    arm.go()
    rospy.sleep(60)

    (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_grasp_pose(item_translation[0], item_translation[1], item_translation[2], - 0.10)
    grasp_pose.pose.position.x = aim_x
    grasp_pose.pose.position.y = aim_y
    grasp_pose.pose.position.z = aim_z
    grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, aim_yaw))
    arm.set_pose_target(grasp_pose)
    arm.go()
    rospy.sleep(60)

    p.pose.position.x = item_translation[0]
    p.pose.position.y = item_translation[1]
    p.pose.position.z = item_translation[2]
    scene.add_box("part", p, (0.01, 0.01, 0.2))
    # rospy.sleep(20)
    # (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_grasp_pose(item_translation[0], item_translation[1], item_translation[2], - 0.050)

    # grasp = Grasp() 
    # grasp.id = "test" 
    # grasp.grasp_pose.header.frame_id = "wrist_roll_link" 
    # grasp.grasp_pose.header.stamp = rospy.Time.now() 
    # grasp.grasp_pose.pose.position.x = aim_x
    # grasp.grasp_pose.pose.position.y = aim_y
    # grasp.grasp_pose.pose.position.z = aim_z
    # grasp.grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.55961170956, 0.00000, aim_yaw))
    # grasp.allowed_touch_objects = ["part", "table"]

    # #grasps.append(grasp)
    # robot.onine_arm.pick("part")

    # # set the grasp pose
    # g.grasp_pose = grasp_pose
    
    # # define the pre-grasp approach
    # g.pre_grasp_approach.direction.header.frame_id = "base_footprint"
    # g.pre_grasp_approach.direction.vector.x = 1.0
    # g.pre_grasp_approach.direction.vector.y = 0.0
    # g.pre_grasp_approach.direction.vector.z = 0.0
    # g.pre_grasp_approach.min_distance = 0.001
    # g.pre_grasp_approach.desired_distance = 0.1
    
    # g.pre_grasp_posture.header.frame_id = "right_gripper_base_link"
    # g.pre_grasp_posture.joint_names = ["right_gripper_finger_joint"]
    
    # pos = JointTrajectoryPoint()
    # pos.positions.append(0.0)
    
    # g.pre_grasp_posture.points.append(pos)
    
    # # set the grasp posture
    # g.grasp_posture.header.frame_id = "right_gripper_base_link"
    # g.grasp_posture.joint_names = ["right_gripper_finger_joint"]

    # pos = JointTrajectoryPoint()
    # pos.positions.append(0.2)
    # pos.effort.append(0.0)
    
    # g.grasp_posture.points.append(pos)

    # # set the post-grasp retreat
    # g.post_grasp_retreat.direction.header.frame_id = "base_footprint"
    # g.post_grasp_retreat.direction.vector.x = 0.0
    # g.post_grasp_retreat.direction.vector.y = 0.0
    # g.post_grasp_retreat.direction.vector.z = 1.0
    # g.post_grasp_retreat.desired_distance = 0.25
    # g.post_grasp_retreat.min_distance = 0.01

    # g.allowed_touch_objects = ["table"]

    # g.max_contact_force = 0
    
    # # append the grasp to the list of grasps
    # grasps.append(g)

    # rospy.sleep(2)

    # # pick the object
    # robot.onine_arm.pick("part", grasps)

    # rospy.spin()
    # roscpp_shutdown()
    
 
#https://groups.google.com/forum/#!topic/moveit-users/7hzzICsfLOQ
#https://groups.google.com/forum/#!msg/moveit-users/_M0mf-R7AvI/CGdh10TrAxMJ