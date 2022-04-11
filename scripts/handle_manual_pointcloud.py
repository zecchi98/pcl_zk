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
import open3d as o3d
import random
bool_exit=False

def reading_pointcloud():
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(pathToFile)
    print(pcd)
    print(np.asarray(pcd.points))
    #o3d.visualization.draw_geometries([pcd])
    return pcd
def creating_mesh_from_pointcloud(pcd):
    alpha = 100
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    o3d.io.write_triangle_mesh(pathToFilestl,mesh)
def estimate_normals(pcd):
    print("Recompute the normal of the downsampled point cloud")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    #o3d.visualization.draw_geometries([pcd])
    return pcd  
def creating_and_saving_pointcloud():

  output = open(pathToFile, "w")
  for i in range(1,10):
    x=random.random()*50
    y=random.random()*200
    z=random.random()*200
    output.write(str(x)+" "+str(y)+" "+str(z)+"\n")
  
  


def main():
  global pathToFile,pathToFilestl
  rospy.init_node('test', anonymous=True)

  rospy.set_param("/elaborate_manual_pointcloud",False)
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  pathToFilestl=pathTopkg+"my_stl.stl"
  #creating_and_saving_pointcloud()
  bool_exit=False
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):

        rospy.rostime.wallsleep(0.5)
        pointcloud_ready=rospy.get_param("/elaborate_manual_pointcloud")
        bool_exit=rospy.get_param("/CloseSystem")
        if(pointcloud_ready):
          pcd=reading_pointcloud()
          pcd=estimate_normals(pcd)
          creating_mesh_from_pointcloud(pcd)
          pointcloud_ready=False
          rospy.set_param("/elaborate_manual_pointcloud",False)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
if __name__ == '__main__':
  main()
