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
#from pytest import Mark, mark

import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String
from ur10_control.srv import * 
from ur10_control.msg import *
from visualization_msgs.msg import *
import o3d as o3d
import random
from sensor_msgs.msg import PointCloud2
import ros_numpy
bool_exit=False

def callback_pointcloud(points):
        point_cloud = open3d.geometry.PointCloud()
        point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(.subscriber.pc))
        vis.create_window()
        print('get points')
        vis.add_geometry(point_cloud)
        print ('add points')
        vis.update_geometry()
        vis.poll_events()

        vis.update_renderer()

        while not rospy.is_shutdown():
            point_cloud.points =  open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(subscriber.pc))
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()  


def main():
  global pathToFile,pathToFilestl
  rospy.init_node('test', anonymous=True)

  rospy.Subscriber('/camera/depth/points', PointCloud2, callback_pointcloud)
  prova()
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):

        rospy.rostime.wallsleep(0.5)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
if __name__ == '__main__':
  main()
