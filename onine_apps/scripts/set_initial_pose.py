#!/usr/bin/env python
import os
os.system("rostopic pub initialpose geometry_msgs/PoseWithCovarianceStamped \'{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 0.25, y: 0 ,z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1 }}}}\' -1")