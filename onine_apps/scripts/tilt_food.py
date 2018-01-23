#!/usr/bin/env python
import rospy, sys, tf
from onine_arm import Arm
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('tilt_eef', anonymous=True)

scene = PlanningSceneInterface()
rospy.sleep(2)

scene.remove_world_object("target")

onine_arm = Arm()
onine_arm.tilt_food()

moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

