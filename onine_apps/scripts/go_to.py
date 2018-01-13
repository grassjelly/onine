#!/usr/bin/env python
import rospy, sys, tf
from onine_arm import Arm
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('go_to', anonymous=True)

scene = PlanningSceneInterface()
robot = RobotCommander()
rospy.sleep(2)

while not rospy.is_shutdown():    
    debugging_pose_pub = rospy.Publisher('onine_debugging_pose', PoseArray, queue_size=1, latch=True)
    pose_msg = PoseArray()

    scene.remove_world_object("target")

    #left test
    # yaw = -0.949421004148
    # item_translation = [0.33292386367734217, 0.1685605027519197, 0.8339949674141176]
    
    #right test
    # yaw =  -2.33954420079
    item_translation = [0.3155979994864394, 0, 0.8829674860024487]

    onine_arm = Arm()

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

    onine_arm.go(aim_x, aim_y, aim_z, 0.0, 0.0, aim_yaw)

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