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
    pose=geometry_msgs.msg.Pose()
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
    pose=geometry_msgs.msg.Pose()
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
    pose=geometry_msgs.msg.Pose()
    
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
  def compute_distance_pose(self,pose1,pose2):
    x1=pose1.position.x
    y1=pose1.position.y
    z1=pose1.position.z
    x2=pose2.position.x
    y2=pose2.position.y
    z2=pose2.position.z
    dx=(x1-x2)*(x1-x2)
    dy=(y1-y2)*(y1-y2)
    dz=(z1-z2)*(z1-z2)
    return math.sqrt(dx+dy+dz)

class Noether_comunication(object):
  def __init__(self):
    global marker_cont
    super(Noether_comunication, self).__init__()
    #rospy.Subscriber("/generate_tool_paths/result",GenerateToolPathsActionResult,self.noether_result_callback)
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathToFile=pathTopkg+"/data/traiettoria.txt"
    self.all_poses_in_traj=[]
    self.all_normals=[]
    self.real_traj=[]
    self.pathToFile=pathToFile
    self.bool_traj_saved=False
    self.bool_normals_saved=False
    self.bool_associazione_completata=False

    marker_cont=0
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
            add_pose_to_marker_array(pose)
            self.all_poses_in_traj.append(pose)
    self.save_traiettoria()
  def save_traiettoria(self):
    
    output_file = open(self.pathToFile, "w")
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
  def read_traj_from_file(self):
    nul=0
    self.all_poses_in_traj=[]
    input_file = open(self.pathToFile, "r")
    for x in input_file:
      s=x.split()
      pose=geometry_msgs.msg.Pose()
      pose.position.x=float(s[0])
      pose.position.y=float(s[1])
      pose.position.z=float(s[2])
      pose.orientation.x=float(s[3])
      pose.orientation.y=float(s[4])
      pose.orientation.z=float(s[5])
      pose.orientation.w=float(s[6])
      self.all_poses_in_traj.append(pose)
    #print(self.all_poses_in_traj)
    self.bool_traj_saved=True
  def read_normals_from_file(self):
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathTopkg=pathTopkg+"/data/"
    pathToFile=pathTopkg+"my_normals.pcd"
    input_file = open(pathToFile, "r")
    cont=0
    cont2=0
    for x in input_file:
      cont=cont+1
    
      if(cont>11):
        
        s=x.split()
        if(s[0]!="nan"):
          pos=movegroup_library.get_current_ee_pose().pose
          pos.position.x=float(s[0])
          pos.position.y=float(s[1])
          pos.position.z=float(s[2])
          quat=transformation_library.from_euler_to_quaternion([float(s[3]),float(s[4]),float(s[5])])
          pos.orientation=quat.orientation
          trans=[0,0,0]
          R=transformation_library.eul2rot([0,math.pi/2,0])
          T_normal=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)



          T_actual=transformation_library.from_pose_to_matrix(pos)

          T_final=np.dot(T_actual,T_normal)
          Pos_final=transformation_library.from_matrix_to_pose(T_final)


          self.all_normals.append(Pos_final)
    self.bool_normals_saved=True
    print("Normals reading completed")
  def associate_traj_and_normals_points(self):
    if not self.bool_traj_saved :
      self.read_traj_from_file()
    if not self.bool_normals_saved :
      self.read_normals_from_file()
     
    for traj_pose in self.all_poses_in_traj:
      min=1000
      for normal_pose in self.all_normals:

        dist=transformation_library.compute_distance_pose(traj_pose,normal_pose)
        if(dist<min):
          min=dist
          normal_piu_vicina=normal_pose
      
      self.real_traj.append(normal_piu_vicina)
    
    print("Associazione finita")
  def add_pose_to_marker_array_as_points(self,pose,sdr):
    global marker_cont,markerArray
    marker=Marker()
    #marker.header.frame_id = "cameradepth_link"
    marker.header.frame_id = sdr
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

    
    marker.scale.x = 0.01
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0; 
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0


    markerArray.markers.append(marker)
    marker_cont=marker_cont+1        
  def add_pose_to_marker_array_as_arrows(self,pose,sdr):
    global marker_cont,markerArray
    marker=Marker()
    #marker.header.frame_id = "cameradepth_link"
    marker.header.frame_id = sdr
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

    marker.scale.x = 0.01
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0; 
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0


    markerArray.markers.append(marker)
    marker_cont=marker_cont+1        
  def publish_traj(self):
    markerArray=MarkerArray()
    for x in self.all_poses_in_traj:
      self.add_pose_to_marker_array_as_arrows(x,"base")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
  def publish_traj_CAMERA(self):
    markerArray=MarkerArray()
    for x in self.all_poses_in_traj:
      self.add_pose_to_marker_array_as_arrows(x,"cameradepth_link")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
  def CAMERA_leggi_normals(self):
    #this function works with all the points in the camera reference system
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathTopkg=pathTopkg+"/data/"
    pathToFile=pathTopkg+"mesh.pcd"

    pcd = o3d.io.read_point_cloud(pathToFile)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    print(pcd)
    o3d.visualization.draw_geometries([pcd])
    num_tot=len(pcd.normals)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.3)
    
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)  # we do not need the geometry ID for mesh

    
    R_identical=transformation_library.eul2rot([0,0,0])
    trans_separiamoli=[-0.005,0,0]
          
    T2=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R_identical,trans_separiamoli)


    for i in range(num_tot):
          
          bool_exit=rospy.get_param("/CloseSystem")
          if bool_exit:
            return
          x=pcd.points[i][0]
          y=pcd.points[i][1]
          z=pcd.points[i][2]
          
          vx=pcd.normals[i][0]
          vy=pcd.normals[i][1]
          vz=pcd.normals[i][2]

          vxy=math.sqrt(vx*vx+vy*vy)

          alpha=math.atan2(vy,vx)
          beta=math.atan2(vz,vxy)

          R1=transformation_library.eul2rot([0,0,alpha])
          R2=transformation_library.eul2rot([0,-beta,0])
          #R3=transformation_library.eul2rot([0,-math.pi/2,0])
          #R3=transformation_library.eul2rot([0,0,0])
          R=np.dot(R1,R2)
          #R=np.dot(R,R3)

          trans=[x,y,z]
          T=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

          T_final=np.dot(T,T2)

          Pos_final=transformation_library.from_matrix_to_pose(T_final)
          
          query_point = o3d.core.Tensor([[Pos_final.position.x,Pos_final.position.y,Pos_final.position.z]], dtype=o3d.core.Dtype.Float32)
          # Compute distance of the query point from the surface
          #unsigned_distance = scene.compute_distance(query_point)
          #signed_distance = scene.compute_signed_distance(query_point)
          occupancy = scene.compute_occupancy(query_point)
            
            
          if(occupancy==0):

          
           
            #Riprendo le distanze corrette
            #Pos_final=transformation_library.from_matrix_to_pose(T)
        
            self.all_normals.append(Pos_final)
            cont=cont+1
            if(cont%(2000)==0):
              print(cont)
              noether_library.publish_normals()
    noether_library.publish_normals()
    self.bool_normals_saved=True   
    print("Finish to read normals")
  def CAMERA_associate_traj_and_normals_points(self):
    if self.bool_associazione_completata:
      return
    if not self.bool_traj_saved :
      self.read_traj_from_file()
    if not self.bool_normals_saved :
      self.CAMERA_leggi_normals()
    T_base_camera_depth=movegroup_library.get_T_base_camera_depth()


    for traj_pose in self.all_poses_in_traj:
      min=1000
      for normal_pose in self.all_normals:

        dist=transformation_library.compute_distance_pose(traj_pose,normal_pose)
        if(dist<min):
          min=dist
          normal_piu_vicina=normal_pose
      
      #Trasformo da camera_depth a base sdr
      pos_target=normal_piu_vicina
      T_from_camera_depth_TO_target=transformation_library.from_pose_to_matrix(pos_target)
      T_base_target=np.dot(T_base_camera_depth,T_from_camera_depth_TO_target)
      final_target=transformation_library.from_matrix_to_pose(T_base_target)
      #print(final_target)
      #print(pos_target)
      #time.sleep(1)

      self.real_traj.append(final_target)
    self.bool_associazione_completata=True
    print("Associazione finita")
  def go_to_all_normals_CAMERA(self):
    global bool_exit
    pose_array=self.all_normals
    #print(pose_array)
    T_base_camera_depth=movegroup_library.get_T_base_camera_depth()
    cont=0
    for pose in pose_array:
      #print(pose)
      cont=cont+1
      if cont%1000==0:
        #Questa rotazione ha lo scopo di mettere il tool sopra l'oggetto, normale ad esso
        T_from_camera_depth_TO_target=transformation_library.from_pose_to_matrix(pose)
        T_base_target=np.dot(T_base_camera_depth,T_from_camera_depth_TO_target)
        
        trans=[0,0,0]
        R=transformation_library.eul2rot([0,0,math.pi])
        T_sr_change=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

        
        T_final=np.dot(T_sr_change,T_base_target)
        Pos_final=transformation_library.from_matrix_to_pose(T_base_target)
        movegroup_library.go_to_pose_goal(Pos_final)
        bool_exit=rospy.get_param("/CloseSystem")
        if bool_exit:
          return
  def publish_normals(self):
    global markerArray
    markerArray=MarkerArray()
    cont=0
    print(len(self.all_normals))
    for x in self.all_normals:
      cont=cont+1
      if cont%100==0:
        
        self.add_pose_to_marker_array_as_points(x,"cameradepth_link")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    #print(markerArray)
    pub.publish(markerArray)
    
def reading_pointcloud():
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(pathToFile)
    print(pcd)
    print(np.asarray(pcd.points))
    #o3d.visualization.draw_geometries([pcd])
    return pcd
def creating_mesh_from_pointcloud(pcd,alpha):
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    o3d.io.write_triangle_mesh(pathToFilestl,mesh)
    o3d.io.write_triangle_mesh(pathToFileply,mesh)
    
    return mesh    
def estimate_normals(pcd):
    print("Recompute the normal of the downsampled point cloud")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    print(pcd)
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(pathToFilepcd,pcd,True)
    return pcd
def elabora_normali_e_filtra_quelle_buone(pcd):
    cont=0
    num_tot=len(pcd.normals)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.3)
    
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)  # we do not need the geometry ID for mesh

    
    print("Inizio a filtrare")
    for i in range(num_tot):
          
          #bool_exit=rospy.get_param("/CloseSystem")
          #if bool_exit:
          #  return
          x=pcd.points[i][0]
          y=pcd.points[i][1]
          z=pcd.points[i][2]
          
          vx=pcd.normals[i][0]
          vy=pcd.normals[i][1]
          vz=pcd.normals[i][2]

          vxy=math.sqrt(vx*vx+vy*vy)

          alpha=math.atan2(vy,vx)
          beta=math.atan2(vz,vxy)

          R1=transformation_library.eul2rot([0,0,alpha])
          R2=transformation_library.eul2rot([0,-beta,0])
          R3=transformation_library.eul2rot([0,-math.pi/2,0])
          R3=transformation_library.eul2rot([0,0,0])
          R=np.dot(R1,R2)
          #R=np.dot(R,R3)

          trans=[x,y,z]
          T=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

          R_identical=transformation_library.eul2rot([0,0,0])
          trans_separiamoli=[-0.0001,0,0]
          
          T2=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R_identical,trans_separiamoli)

          T_final=np.dot(T,T2)

          Pos_final=transformation_library.from_matrix_to_pose(T_final)
          
          query_point = o3d.core.Tensor([[Pos_final.position.x,Pos_final.position.y,Pos_final.position.z]], dtype=o3d.core.Dtype.Float32)
          # Compute distance of the query point from the surface
          #unsigned_distance = scene.compute_distance(query_point)
          #signed_distance = scene.compute_signed_distance(query_point)
          occupancy = scene.compute_occupancy(query_point)
            
            
          if(occupancy==0):

          
           
            #Riprendo le distanze corrette
            Pos_final=transformation_library.from_matrix_to_pose(T)
        
            noether_library.all_normals.append(Pos_final)
            cont=cont+1
            if(cont%(2000)==0):
              print(cont)
              noether_library.publish_normals()
              
    noether_library.publish_normals()
    noether_library.bool_normals_saved=True
    print("finito")
    return pcd  
def creating_and_saving_pointcloud():

  output = open(pathToFile, "w")
  for i in range(1,10):
    x=random.random()*50
    y=random.random()*200
    z=random.random()*200
    output.write(str(x)+" "+str(y)+" "+str(z)+"\n")
def salva_real_surface_points():
  #per ora non sono nell sdr della base

  pathToNewCoordinates=pathTopkg+"real_surface_points.pcd"
  cont=0
  output_file = open(pathToNewCoordinates, "w")
  print("Inizio a stampare")
  for x in noether_library.all_normals:
    cont=cont+1
    output_file.write(str(x.position.x)+" "+str(x.position.y)+" "+str(x.position.z)+ " ")
    output_file.write(str(x.orientation.x)+" "+str(x.orientation.y)+" "+str(x.orientation.z)+" "+str(x.orientation.w)+ "\n")
    if cont%5000==0:
      print(cont)
  print("Finito di stampare")
def main():
  global pathToFile,pathToFilestl,pathTopkg,pathToFilepcd,pathToNewCloud,pathToFileply,noether_library
  global marker_cont,markerArray,pub,transformation_library
  rospy.init_node('test', anonymous=True)
  pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
  
  markerArray=MarkerArray()
  marker_cont=0
  rospy.set_param("/CloseSystem",False)
  rospy.set_param("/elaborate_manual_pointcloud",False)
  rospy.set_param("/alpha",0.07)
  noether_library=Noether_comunication()
  transformation_library=Transformation_class()
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  #pathToFile=pathTopkg+"mesh.ply"
  pathToFile=pathTopkg+"mesh.ply"
  pathToNewCloud=pathTopkg+"new_cloud.pcd"
  pathToFilestl=pathTopkg+"mesh.stl"
  pathToFilepcd=pathTopkg+"my_normals.pcd"
  pathToFileply=pathTopkg+"meshNotCorrect.ply"


  alpha=rospy.get_param("/alpha")


  
  pcd=reading_pointcloud()
  pcd=estimate_normals(pcd)
  elabora_normali_e_filtra_quelle_buone(pcd)
  salva_real_surface_points()
  return
  #mesh=creating_mesh_from_pointcloud(pcd,alpha)
  #creating_and_saving_pointcloud()
  bool_exit=False
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):

        rospy.rostime.wallsleep(0.5)
        pointcloud_ready=rospy.get_param("/elaborate_manual_pointcloud")
        bool_exit=rospy.get_param("/CloseSystem")
        if(pointcloud_ready):
          alpha=rospy.get_param("/alpha")
          pcd=reading_pointcloud()
          pcd=estimate_normals(pcd)
          mesh=creating_mesh_from_pointcloud(pcd,alpha)
          
          pointcloud_ready=False
          rospy.set_param("/elaborate_manual_pointcloud",False)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
if __name__ == '__main__':
  main()
