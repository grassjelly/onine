#!/usr/bin/env python
import os, sys, rospy, tf, math
import moveit_commander
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, CollisionObject


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
        self.gripper.set_named_target("gripper_open")
        os.system("rosservice call clear_octomap")
        self.gripper.go()
        rospy.sleep(2)

    def close_gripper(self):
        self.gripper.set_named_target("gripper_closed")
        os.system("rosservice call clear_octomap")
        self.gripper.go()
        rospy.sleep(2)

    def pickup_sim(self, x, y, z):
        self.ready()
        self.open_gripper()
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, -0.20)
        self.go(aim_x, aim_y, aim_z, aim_yaw)

        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, -0.08)
        self.go(aim_x, aim_y, aim_z, aim_yaw)
        self.close_gripper()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('go_to', anonymous=True)

    tf_listener = tf.TransformListener() 
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    gripper = MoveGroupCommander("onine_gripper")
    arm = MoveGroupCommander("onine_arm") 

    rate = rospy.Rate(10)

    rospy.sleep(2)

    while not rospy.is_shutdown():
        rate.sleep()
        
        debugging_pose_pub = rospy.Publisher('onine_debugging_pose', PoseArray, queue_size=1, latch=True)
        pose_msg = PoseArray()

        scene.remove_world_object("target")

        #left test
        # yaw = -0.949421004148
        # item_translation = [0.33292386367734217, 0.1685605027519197, 0.8339949674141176]
        
        #right test
        # yaw =  -2.33954420079
        item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]

        onine_arm = Onine(arm, gripper)

        onine_arm.ready()

        (aim_x, aim_y, aim_z, aim_yaw) = onine_arm.get_valid_pose(item_translation[0], item_translation[1], item_translation[2], - 0.08)

        debugging_pose = PoseStamped()
        debugging_pose.pose.position.x = aim_x
        debugging_pose.pose.position.y = aim_y
        debugging_pose.pose.position.z = aim_z
        debugging_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0, aim_yaw))

        pose_msg.poses.append(Pose(debugging_pose.pose.position, debugging_pose.pose.orientation))
        pose_msg.header.frame_id = robot.get_planning_frame()
        pose_msg.header.stamp = rospy.Time.now()
        debugging_pose_pub.publish(pose_msg)

        onine_arm.go(aim_x, aim_y, aim_z, aim_yaw)

        p = PoseStamped()
        p.header.frame_id = "base_footprint"
        p.pose.position.x = item_translation[0]
        p.pose.position.y = item_translation[1]
        p.pose.position.z = item_translation[2]
        scene.add_box("target", p, (0.02, 0.02, 0.09))
        rospy.sleep(2)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

        break 