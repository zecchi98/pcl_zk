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
bool_exit=False
def from_stl_to_ply():
  nul=0
  
  print("printing")
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  mesh = o3d.io.read_triangle_mesh(pathTopkg+"mesh.stl")
  
  o3d.io.write_triangle_mesh(pathTopkg+"meshNotCorrect.ply", mesh,write_ascii=True)
  f = open(pathTopkg+"meshNotCorrect.ply", "r")
  output = open(pathTopkg+"mesh.ply", "w")
  cont=0
  for x in f:

    if(cont==4):
      output.write("property float x\n")
    elif(cont==5):
      output.write("property float y\n")
    elif(cont==6):
      output.write("property float z\n")
    elif(cont==11):
      output.write("property list uchar int vertex_indices\n") 

    else:
      output.write(x)
    cont=cont+1
  print("finish_to_print")
  rospy.set_param("need_to_create_traj",True)
def main():
  
  rospy.init_node('correct_ply', anonymous=True)
  rospy.set_param("/need_to_transform",False)
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):


        bool_transform=rospy.get_param("/need_to_transform")
        if(bool_transform):
          print("ok")
          from_stl_to_ply()
          rospy.set_param("need_to_transform",False)
        rospy.rostime.wallsleep(0.1)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  except bool_exit==True:
      return
if __name__ == '__main__':
  main()
