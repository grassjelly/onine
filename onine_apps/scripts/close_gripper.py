#!/usr/bin/env python
import os

#TODO: make this ROS native

os.system("rostopic pub /onine_gripper std_msgs/Bool 0 -1")
