#!/usr/bin/env python
import rospy, sys, tf
from onine_arm import Arm
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface
import moveit_commander

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('pickup', anonymous=True)

scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size = 10)

tf_listener = tf.TransformListener() 
scene = PlanningSceneInterface()
robot = RobotCommander()
rospy.sleep(2)

while not rospy.is_shutdown():

    scene.remove_world_object("target")

    try:
      t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_3') # <7>
      print ((rospy.Time.now() - t).to_sec())
      if (rospy.Time.now() - t).to_sec() > 1:
        continue

      (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_3", t) 
    except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    #left test
    # yaw = -0.949421004148
    # item_translation = [0.33292386367734217, 0.1685605027519197, 0.799949674141176]
    
    #right test
    # yaw =  -2.33954420079

    # item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]
    # item_translation = [0.3155979994864394, 0, 0.8829674860024487]

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = item_translation[0] + 0.025
    p.pose.position.y = item_translation[1]
    p.pose.position.z = item_translation[2]
    scene.add_box("target", p, (0.045, 0.045, 0.08))

    onine_arm = Arm()
    onine_arm.pickup_sim(item_translation[0]+ 0.025, item_translation[1] , item_translation[2])

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

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    break 

#http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html
#https://answers.ros.org/question/215045/obstacles-in-moveit-python/