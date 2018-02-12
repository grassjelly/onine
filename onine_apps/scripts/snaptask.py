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

import requests
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Snap:
    def __init__(self):
        image_topic = "/camera/rgb/image_color"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        self.bridge =CvBridge()
        self.is_triggered = False

    def image_callback(self, msg):
        if self.is_triggered:
            print("Saving Image!")
            try:
                # convert received image to CV2
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                # save received image to local drive
                cv2.imwrite('photo.jpg', image)
            self.is_triggered = False

    def save_photo(self):
        self.is_triggered = True

    def send_photo(self):
        #set flag to take photo on the image callback
        self.save_photo()

        rospy.sleep(1)

        #send photo to user's phone
        r = requests.post("https://api.pushover.net/1/messages.json", 
                data={
                    "token":"APP_TOKEN",
                    "user":"USER_KEY",
                    "message":"Living Room Photo"
                }, 
                files={
                    "attachment":open("photo.jpg","rb")
                }
            )

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('snap', anonymous=True)

rospy.sleep(1)

while not rospy.is_shutdown():

    onine_base = Base()
    onine_arm = Arm()

    onine_arm.home()
    #TODO replace real living room coordinates (this is currently a placeholder)
    # onine_base.go(Pose(Point(0.525607347488, 1.70954406261, 0.0), Quaternion(0.000, 0.000, 0.0, 0.0180804141297)), 80)
    
    s = Snap()
    s.send_photo()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    break 