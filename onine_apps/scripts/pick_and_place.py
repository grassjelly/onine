#!/usr/bin/env python
import os, sys, rospy, tf, math
import moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

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
        rospy.sleep(2)


    def get_valid_pose(self, x, y, z, distance):
        origin_translation = [0.095, 0.00, 0.00]

        delta_x = x - origin_translation[0]
        delta_y = y - origin_translation[1]

        theta = math.atan(delta_y / delta_x)
        grasp_yaw = theta
        grasp_x = x + (distance * math.cos(theta))
        grasp_y = y + (distance * math.sin(theta))

        return (grasp_x, grasp_y, z, grasp_yaw)

    def ready(self):
        self.arm.set_named_target("onine_ready")
        self.arm.go(wait=True)

    def home(self):
        self.arm.set_named_target("onine_home")
        self.arm.go(wait=True)

    def open_gripper(self):
        # self.gripper.set_named_target("gripper_open")
        os.system("rostopic pub /onine_gripper std_msgs/Bool 1 -1")
        os.system("rosservice call clear_octomap")
        # self.gripper.go()
        # rospy.sleep(2)

    def close_gripper(self):
        # self.gripper.set_named_target("gripper_closed")
        os.system("rostopic pub /onine_gripper std_msgs/Bool 0 -1")
        os.system("rosservice call clear_octomap")
        # self.gripper.go()
        # rospy.sleep(2)

    def pickup_sim(self, x, y, z):
        self.ready()
        self.open_gripper()
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, -0.12)
        self.go(aim_x, aim_y, aim_z, aim_yaw)

        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, 0)
        self.go(aim_x, aim_y, aim_z, aim_yaw)
        self.close_gripper()

    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        gripper_joint_names = ['l_gripper_joint']
        gripper_effort = [1.0, 1.0]

        t.joint_names = gripper_joint_names
        tp = JointTrajectoryPoint()
        tp.positions = [pose/2.0 for j in t.joint_names]
        print tp.positions
        tp.effort = gripper_effort
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = "tool_link"
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('pick_and_place', anonymous=True)
    debugging_pose_pub = rospy.Publisher('onine_debugging_pose', PoseArray, queue_size=1, latch=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("onine_arm")
    gripper = MoveGroupCommander("onine_gripper")

    debugging_pose = PoseArray()

    rospy.sleep(1)

    onine_arm = Onine(arm, gripper)

    # clean the scene
    # scene.remove_world_object("table")
    scene.remove_world_object("target")
    
    rospy.sleep(1)

    # item_translation = [0.33292386367734217, 0.1685605027519197, 0.8]
    # item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]
    item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]

    onine_arm.ready()
    onine_arm.open_gripper()

    # bring the arm near the object
    (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_valid_pose(item_translation[0], item_translation[1], item_translation[2], -0.20)
    # onine_arm.go(aim_x, aim_y, aim_z, aim_yaw)

    # rospy.sleep(20)
    (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_valid_pose(item_translation[0], item_translation[1], item_translation[2], 0)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = item_translation[0]
    p.pose.position.y = item_translation[1]
    p.pose.position.z = item_translation[2]
    p.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0, aim_yaw))
    scene.add_box("target", p, (0.045, 0.045, 0.08))

    # add a table
    # p.pose.position.x = 0.52
    # p.pose.position.y = -0.2
    # p.pose.position.z = 0.35
    # scene.add_box("table", p, (0.5, 1.5, 0.7))

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_footprint"
    grasp_pose.header.stamp = rospy.Time.now()
    grasp_pose.pose.position.x = aim_x
    grasp_pose.pose.position.y = aim_y
    grasp_pose.pose.position.z = aim_z
    grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0, aim_yaw))

    g = Grasp()
    g.pre_grasp_posture = onine_arm.make_gripper_posture(0.08)
    g.grasp_posture = onine_arm.make_gripper_posture(0.0075)
    g.pre_grasp_approach = onine_arm.make_gripper_translation(0.13, 0.15)
    g.post_grasp_retreat = onine_arm.make_gripper_translation(0.13, 0.15, -0.15)
    g.grasp_pose = grasp_pose

    #2 degrees resolution
    pitch_vals = [-0.10472, -0.0698132, -0.0349066, 0, 0.0349066, 0.0698132, 0.10472]
    height_vals = [-0.005, -0.004, -0.003, -0.002, -0.001, 0, 0.001, 0.002, 0.003, 0.004, 0.005]
    pos_vals = [-0.005, -0.004, -0.003, -0.002, -0.001, 0, 0.001, 0.002, 0.003, 0.004, 0.005]
    # generate list of grasps
    grasps = []
    for pos in pos_vals:
        for h in height_vals:
            for p in pitch_vals:

                pos_x = item_translation[0] - pos
                pos_y = item_translation[1] - pos

                (dx, dy, dz, dyaw) = onine_arm.get_valid_pose(pos_x, pos_y, aim_z, 0)

                g.grasp_pose.pose.position.x =  dx
                g.grasp_pose.pose.position.y =  dy
                g.grasp_pose.pose.position.z =  dz - h
                q = quaternion_from_euler(0.0, - p, dyaw)
                
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                g.id = str(len(grasps))
                g.grasp_quality = 1.0
                g.allowed_touch_objects = ["target"]
                grasps.append(copy.deepcopy(g))
                debugging_pose.poses.append(copy.deepcopy(Pose(g.grasp_pose.pose.position, g.grasp_pose.pose.orientation)))


    debugging_pose.header.frame_id = robot.get_planning_frame()
    debugging_pose.header.stamp = rospy.Time.now()
    debugging_pose_pub.publish(debugging_pose)

    rospy.sleep(10)
    result = None
    n_attempts = 0
    max_pick_attempts = 10;
    # repeat until will succeed
    while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
        result = robot.onine_arm.pick("target", grasps)       
        n_attempts += 1
        print "Attempts: ", n_attempts
        rospy.sleep(0.2)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

#https://groups.google.com/forum/#!topic/moveit-users/7hzzICsfLOQ
#https://groups.google.com/forum/#!msg/moveit-users/_M0mf-R7AvI/CGdh10TrAxMJ
#https://github.com/mikeferguson/chessbox/blob/hydro-devel/chess_player/src/chess_player/chess_utilities.py
#https://groups.google.com/forum/#!topic/moveit-users/TSCBCWbH5Ko