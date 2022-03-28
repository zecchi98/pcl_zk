#!/usr/bin/env python3
#fold all: ctrl + k + 0
#unfold all: ctrl + k + j
import copy
import math
from shutil import move
import sys
import time
from logging import setLoggerClass
from math import cos, pi, sin
from os import access
from re import X
import rospkg
from pytest import Mark, mark

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
from moveit_commander import *
from moveit_commander.conversions import pose_to_list
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ur10_control.srv import * 
from ur10_control.msg import *
from noether_msgs.msg import *
from visualization_msgs.msg import *
import open3d as o3d
from moveit.core import robot_state
bool_exit=False
class Noether_comunication(object):
  def __init__(self):
    super(Noether_comunication, self).__init__()
    rospy.Subscriber("/generate_tool_paths/result",GenerateToolPathsActionResult,self.noether_result_callback)
  def noether_result_callback(self,msg):

    tool_paths=msg.result.tool_paths
    for tool_path in tool_paths:
      paths=tool_path.paths
      for path in paths:
        segments=path.segments
        for segment in segments:
          poses=segment.poses
          #movegroup_library.follow_pose_trajectory(poses)
          for pose in poses:
            #print(pose)
            #movegroup_library.go_to_pose_cartesian(pose)
            #time.sleep(1)
            add_pose_to_marker_array(pose)
    print("finished")
def add_pose_to_marker_array(pose):
  global marker_cont
  marker=Marker()
  marker.header.frame_id = "cameradepth_link"
  marker.header.stamp = rospy.get_rostime()

  marker.ns = ""
  marker.id = marker_cont
  marker.type = visualization_msgs.msg.Marker.ARROW
  marker.action = 0
  marker.pose=pose
  marker.scale.x = 0.02
  marker.scale.y = 0.005
  marker.scale.z = 0.005
  marker.color.a = 1.0; 
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0


  markerArray.markers.append(marker)
  marker_cont=marker_cont+1

def main():
  global markerArray,pub,marker_cont
  rospy.init_node('publish_marker', anonymous=True)

  noether_library=Noether_comunication()

  markerArray=MarkerArray()
  marker_cont=0
  pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):
        if not marker_cont==0 :
          pub.publish(markerArray)
        rospy.rostime.wallsleep(0.5)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  except bool_exit==True:
      return
if __name__ == '__main__':
  main()
