#!/usr/bin/env python
import os, sys, rospy, tf, math
import moveit_commander
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

class Onine():

    def __init__(self, arm, gripper):
        self.tf_listener = tf.TransformListener() 
        self.p = Pose()

        self.gripper = gripper
        self.arm = arm

        arm.set_goal_tolerance(0.002)
        self.arm.allow_replanning(True)
        # self.arm.set_goal_position_tolerance(0.005)
        # self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planning_time(5)
        self.arm.set_planner_id("RRTkConfigDefault")

    def go(self, x, y, z, roll, pitch, yaw):
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        self.arm.set_pose_target(self.p)

        os.system("rosservice call clear_octomap")
        # plan1 = self.arm.plan()
        # self.arm.execute(plan1)
        self.arm.go(wait=True)
        rospy.loginfo("Moving to target")
        rospy.sleep(1)

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
        
        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z + 0.15, 0.025)
        self.go(aim_x, aim_y, aim_z, 0.0, 0.0, aim_yaw)

        (aim_x, aim_y, aim_z, aim_yaw) = self.get_valid_pose(x, y, z, 0.025)
        self.go(aim_x, aim_y, aim_z, 0.0, 0.0, aim_yaw)
        
        # self.close_gripper()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('pickup', anonymous=True)

    scene_pub = rospy.Publisher('/planning_scene', PlanningScene)

    tf_listener = tf.TransformListener() 
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    gripper = MoveGroupCommander("onine_gripper")
    arm = MoveGroupCommander("onine_arm") 

    rospy.sleep(2)

    while not rospy.is_shutdown():

        scene.remove_world_object("target")

        try:
          t = tf_listener.getLatestCommonTime('/base_footprint', '/ar_marker_0') # <7>
          if (rospy.Time.now() - t).to_sec() > 0.2:
            continue

          (item_translation, item_orientation) = tf_listener.lookupTransform('/base_footprint', "ar_marker_0", t) 
        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #left test
        # yaw = -0.949421004148
        # item_translation = [0.33292386367734217, 0.1685605027519197, 0.799949674141176]
        
        #right test
        # yaw =  -2.33954420079

        # item_translation = [0.3155979994864394, -0.21095350748804098, 0.8829674860024487]

        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = item_translation[0] + 0.025
        p.pose.position.y = item_translation[1]
        p.pose.position.z = item_translation[2]
        scene.add_box("target", p, (0.045, 0.045, 0.08))

        onine_arm = Onine(arm, gripper)
        onine_arm.pickup_sim(item_translation[0], item_translation[1], item_translation[2])

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

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

        break 

#http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html
#https://answers.ros.org/question/215045/obstacles-in-moveit-python/