#!/usr/bin/env python
import rospy, sys, tf, os
from onine_arm import Arm
from onine_base import Base
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface
import moveit_commander

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('fishtask', anonymous=True)

scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size = 10)

tf_listener = tf.TransformListener() 
scene = PlanningSceneInterface()
robot = RobotCommander()

rospy.sleep(1)

while not rospy.is_shutdown():

    onine_base = Base()
    onine_arm = Arm()

    reached = onine_base.go(Pose(Point(0.525607347488, 1.70954406261, 0.0), Quaternion(0.000, 0.000, 0.0, 0.0180804141297)), 80)
    onine_arm.ready()
    
    onine_base.dock(0.10)
    
    rospy.sleep(5)

    rospy.loginfo("Looking for food")
    tf_listener.waitForTransform('/base_footprint', '/ar_marker_3', rospy.Time(), rospy.Duration(4.0))

    try:
        now = rospy.Time.now()
        tf_listener.waitForTransform('/base_footprint', '/ar_marker_3', now, rospy.Duration(60.0))

        (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", now) 

    except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "cannot find marker"
        break

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = item_translation[0] + 0.025
    p.pose.position.y = item_translation[1]
    p.pose.position.z = item_translation[2]
    scene.add_box("target", p, (0.045, 0.045, 0.08))

    onine_arm.pickup_sim(item_translation[0] + 0.025, item_translation[1], item_translation[2])

    rospy.sleep(1)

    attached_object = AttachedCollisionObject()
    attached_object.link_name = "tool_link"
    #The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "tool_link"
    #The id of the object
    attached_object.object.id = "target"

    #A default pose 
    pose = Pose()
    pose.orientation.w = 1.0

    #Define a box to be attached 
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.045,0.045, 0.08]

    attached_object.object.primitives.append(primitive)
    attached_object.object.primitive_poses.append(pose)
    attached_object.object.operation = attached_object.object.ADD

    planning_scene = PlanningScene(is_diff = True)
    planning_scene.world.collision_objects.append(attached_object.object)
    scene_pub.publish(planning_scene)
    rospy.sleep(2)

    remove_object = CollisionObject()
    remove_object.id = "target"
    remove_object.header.frame_id = "base_footprint"
    remove_object.operation = remove_object.REMOVE

    rospy.loginfo("Attaching the object to the right wrist and removing it from the world.")
    planning_scene.world.collision_objects.append(remove_object)
    planning_scene.robot_state.attached_collision_objects.append(attached_object)
    scene_pub.publish(planning_scene)
    rospy.sleep(2)

    onine_arm.close_gripper()
    
    #move away from table
    onine_base.move(1, 1, -0.25, 0.3)

    os.system("rosservice call clear_octomap")

    #move arm to feeding positing
    onine_arm.feed_pos()

    #give some time for the arm to finish moving
    rospy.sleep(5)

    #dock the robot 10 cm away from the table
    onine_base.dock(0.10)

    rospy.sleep(5)

    onine_arm.tilt_food()
    onine_arm.feed_pos()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    break 

#http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html
#https://answers.ros.org/question/215045/obstacles-in-moveit-python/