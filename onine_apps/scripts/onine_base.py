#!/usr/bin/env python
import rospy, os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

class Base:

    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.wall_distance = 0.0;

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

    def laser_callback(self, data):
        laser_data = data.ranges
        self.wall_distance = laser_data[abs(len(laser_data)/2)]
        # rospy.loginfo("DISTANCE: %f", self.wall_distance)

    def dock(self, distance):
        twist_msg = Twist()
        rospy.loginfo("Docking mobile base")
        t0 = rospy.Time.now().to_sec()

        current_distance = 0
        rospy.loginfo("LASER DISTANCE: %f", self.wall_distance)

        dock_distance = self.wall_distance - (distance + 0.05)
        
        #TODO: call move() function here for modularity

        try:
            while(current_distance < dock_distance):
                twist_msg.linear.x = 0.15
                self.vel_pub.publish(twist_msg)
                t1 = rospy.Time.now().to_sec()
                current_distance = 0.15 * (t1 - t0)
                # rospy.loginfo("MOVING")
                rospy.sleep(0.1)

            rospy.loginfo("DISTANCE: %f", current_distance)
            twist_msg.linear.x = 0.0
            self.vel_pub.publish(twist_msg)
            return 1

        except:
            return 0
 
    def move(self, x, y, speed, distance):
        twist_msg = Twist()
        rospy.loginfo("Undocking mobile base")

        t0 = rospy.Time.now().to_sec()

        current_distance = 0

        try: 
            while(current_distance < distance):
                twist_msg.linear.x = speed * x
                twist_msg.linear.y = speed * y
                self.vel_pub.publish(twist_msg)
                t1 = rospy.Time.now().to_sec()
                current_distance = abs(speed) * (t1 - t0)
                # rospy.loginfo("MOVING")
                rospy.sleep(0.1)

            rospy.loginfo("DISTANCE: %f", current_distance)
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            self.vel_pub.publish(twist_msg)
            os.system("rosservice call move_base/clear_costmaps")
            return 1

        except:
            return 0
    
    def go(self, pose, timeout):

        #move_base objec constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        rospy.loginfo("Moving mobile base to goal")

        #move the robot to next point
        self.move_base.send_goal(goal)
        reached = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not reached:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose")
            
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                msg = "Waypoint reached\n"
                rospy.loginfo(msg)
        
        return reached

if __name__ == "__main__":
    
    rospy.init_node("wall_hugger", anonymous = True)
    while not rospy.is_shutdown():

        dock = Dock()
        rospy.loginfo("%d", dock.dock())
        rospy.loginfo("%d", dock.undock())
        rospy.loginfo("hello")
        break