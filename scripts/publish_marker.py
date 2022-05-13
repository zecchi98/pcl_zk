#!/usr/bin/env python
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
from pytest import  mark
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String
from noether_msgs.msg import *
from visualization_msgs.msg import *
import rospkg
bool_exit=False

class Transformation_class():
  def __init__(self):
        null=0
  def rad_to_grad(self,angle):
    return angle*180/3.1415
  def grad_to_rad(self,angle):
    return angle*3.1415/180
  def rot2eul(self,R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))
  def rpy_from_quat (self,quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll,pitch,yaw]
  def Rotation_from_quat(self,quaternion):
    
    euler_angles=self.rpy_from_quat(quaternion)
    return self.eul2rot(euler_angles)
  def eul2rot(self,theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R
  def Rotation_matrix_of_Affine_matrix(self,aff_matrix):
    R=np.zeros([3,3])
    for r in range(3):
      for c in range(3):
        R[r,c]=aff_matrix[r,c]
    return R
  def Translation_vector_of_Affine_matrix(self,AffineMat):
    vet=np.zeros(3)
    for r in range(3):
      vet[r]=AffineMat[r,3]
    return vet
  def create_affine_matrix_from_rotation_matrix_and_translation_vector(self,R,transl):
    #input: una matrice di rotazione e un vettore riga di traslazione"[0,1,3]"
    #return the result affine matrix
    AffineMat=np.zeros([4,4])
    
    #copio matrice di rotazione
    for r in range(3):
      for c in range(3):
        AffineMat[r,c]=R[r,c]

    #copio vettore traslazione
    AffineMat[0,3]=transl[0]
    AffineMat[1,3]=transl[1]
    AffineMat[2,3]=transl[2]

    #setto ultima riga standard
    AffineMat[3,0]=0
    AffineMat[3,1]=0
    AffineMat[3,2]=0
    AffineMat[3,3]=1

    return AffineMat
  def from_euler_to_quaternion(self,euler_vet):
    #Input: vettore degli angoli di eulero
    #Output: Pose with the correct orientation in quaternion
    roll=euler_vet[0]
    pitch=euler_vet[1]
    yaw=euler_vet[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    pose=Pose()
    pose.orientation.x=qx
    pose.orientation.y=qy
    pose.orientation.z=qz
    pose.orientation.w=qw
    
    return pose 
  def from_rotation_to_quaternion(self,R):
    #Input: Rotation matrix
    #Output: Pose with the correct orientation in quaternion
    euler_vet=self.rot2eul(R)
    pose_oriented=self.from_euler_to_quaternion(euler_vet)
    return pose_oriented
  def from_vet_to_posePosition(self,vet):
    #Input: vettore contentente la posizione di un frame
    #Output: Pose con la corretta position
    pose=Pose()
    pose.position.x=vet[0]
    pose.position.y=vet[1]
    pose.position.z=vet[2]
    return pose
  def from_pose_to_matrix(self,pose_):
    R=self.Rotation_from_quat(pose_.orientation)
    vet=[pose_.position.x,pose_.position.y,pose_.position.z]
    AffineMat=self.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,vet)
    return AffineMat
  def from_matrix_to_pose(self,AffineMat):
    #Input: Affine matrix
    #Output: Pose
    pose=Pose()
    
    R=self.Rotation_matrix_of_Affine_matrix(AffineMat)
    translation_vet=self.Translation_vector_of_Affine_matrix(AffineMat)
    
    #pose with the correct position
    pose_position=self.from_vet_to_posePosition(translation_vet)

    #pose with the correct orientation
    pose_orientation=self.from_rotation_to_quaternion(R)

    pose.orientation=pose_orientation.orientation
    pose.position=pose_position.position
    return pose
  def inverse_matrix(self,AffineMat):
    return np.linalg.inv(AffineMat)


class Noether_comunication(object):
  def __init__(self):
    super(Noether_comunication, self).__init__()
    rospy.Subscriber("/generate_tool_paths/result",GenerateToolPathsActionResult,self.noether_result_callback)
    self.all_poses_in_traj=[]
  def noether_result_callback(self,msg):
    self.all_poses_in_traj=[]
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
            self.add_pose_to_marker_array(pose)
            self.all_poses_in_traj.append(pose)
    self.save_traiettoria()
  def save_traiettoria(self):
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathToFile=pathTopkg+"/data/traiettoria.txt"
    output_file = open(pathToFile, "w")
    for pose in self.all_poses_in_traj:
      x=str(pose.position.x)
      y=str(pose.position.y)
      z=str(pose.position.z)
      ox=str(pose.orientation.x)
      oy=str(pose.orientation.y)
      oz=str(pose.orientation.z)
      ow=str(pose.orientation.w)
      output_file.write(x+" "+y+" "+z+" "+ox+" "+oy+" "+oz+" "+ow+"\n")
    output_file.close()
    print("Finish to plot and save trajectory\n\n")
  def add_pose_to_marker_array(self,pose):
    global marker_cont
    marker=Marker()
    marker.header.frame_id = "cameradepth_link"
    #marker.header.frame_id = "base"
    marker.header.stamp = rospy.get_rostime()

    marker.ns = ""
    marker.id = marker_cont
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = 0
    marker.pose=pose  
    
    #R1=transformation_library.Rotation_from_quat(marker.pose.orientation)
    #R2=transformation_library.eul2rot([0,math.pi/2,0])
    #R=np.dot(R1,R2)
    #pose_quat=transformation_library.from_rotation_to_quaternion(R)
    #marker.pose.orientation=pose_quat.orientation

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
  global markerArray,pub,marker_cont,transformation_library
  rospy.init_node('publish_marker', anonymous=True)

  noether_library=Noether_comunication()
  transformation_library=Transformation_class()
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
