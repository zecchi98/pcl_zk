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


def eliminare_punti_da_cloud_above_z(z_above):
  
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  input_file = open(pathToFile, "r")
  
  new_file=[]
  cont=0
  for line in input_file:
    xyz_line = line.split(" ")
    z=float(xyz_line[2])
    
    print(xyz_line[0]+" "+xyz_line[1]+ " " + xyz_line[2])
    if(z>z_above):
      new_file.append(xyz_line)
    else:
      cont=cont+1

  print("Sto per eliminare "+str(cont)+" dati\n")
  scelta=input("Se sei sicuro inserire 42\n Risposta:")
  if(scelta!="42"):
    print("Exiting without damage")
    return

  input_file.close()
  output_file = open(pathToFile, "w")
  for x in new_file:
    output_file.write(x[0]+" "+x[1]+ " " + x[2])


    

def main():
  global pathToFile,pathToFilestl
  rospy.init_node('test', anonymous=True)

  rospy.set_param("/elaborate_manual_pointcloud",False)
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  pathToFilestl=pathTopkg+"my_stl.stl"
  
  z_delete=float(input("\nSotto quale valore di z vuoi che elimino i dati?Risposta:"))
  
  print("Elimino dati")
  eliminare_punti_da_cloud_above_z(z_delete)


if __name__ == '__main__':
  main()
