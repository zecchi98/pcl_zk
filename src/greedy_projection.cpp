#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/surface/mls.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

 #include <tf2_eigen/tf2_eigen.h>
 #include "pcl_ros/transforms.h"
 #include "pcl_ros/impl/transforms.hpp"
using namespace std;
//ctmviewer
bool fine=false;
bool bool_need=false;
shared_ptr<tf::TransformListener > listener;
void saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera)
{
  pcl::PLYWriter writer;
  writer.writeASCII(filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary, use_camera);
  
}
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if(bool_need==false)
    return ;
  fine=true;


  //Changing reference frame
  sensor_msgs::PointCloud2* cloud_msg_converted;
  

  pcl_ros::transformPointCloud("/base_link",*cloud_msg, *cloud_msg_converted, *listener);
  // Container for original & filtered data
  ROS_INFO("Message received");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2; 



  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg_converted, *cloud_blob);



  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered;



  ROS_INFO("from msg to cloud2");

  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);

  ROS_INFO("from cloud2 to cloud");
  

    

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  
  





  //* cloud_with_normals = cloud + normals
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  ROS_INFO("Saving the file");
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles);
  std::string path = ros::package::getPath("pcl_zk");
  std::string path2 = ros::package::getPath("pcl_zk");
  std::string path3 = ros::package::getPath("pcl_zk");
  //path=path+"/data";
  std::string nome="/data/mesh.stl";
  std::string nome2="/data/mesh.pcd";
  std::string nome3="/data/mesh_prova.pcd";
  path=path+nome;
  path2=path2+nome2;
  path3=path3+nome3;
  ROS_INFO("Getting the pkg");

  pcl::io::savePCDFileASCII (path2, *cloud_with_normals);
  pcl::io::savePCDFileASCII (path3, *cloud);

  // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    viewer.addPointCloud(cloud);
    
    pcl::visualization::PCLVisualizer viewer_normals("PCL Viewer Normals");
    viewer_normals.setBackgroundColor (0.0, 0.0, 0.5);
    viewer_normals.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
   
    while (!viewer_normals.wasStopped ())
    {
      viewer.spinOnce ();
    }
  pcl::io::savePolygonFileSTL(path, triangles);
  ros::param::set("need_to_transform",true);
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles,true);
  
    

  ROS_INFO("ALL GOOD");
  return ;
}
/*void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if(bool_need==false)
    return ;
  fine=true;
  // Container for original & filtered data
  ROS_INFO("Message received");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered;



  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);

  ROS_INFO("from msg to cloud2");

  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);

  ROS_INFO("from cloud2 to cloud");
  
  tf::TransformListener listener;
  sensor_msgs::PointCloud buffer_local;
  listener.lookupTransform("/odom", "/base_laser", ros::Time(0), transform);

  pcl_ros::transformPointCloud("/base_laser",_buffer, buffer_local, listener);

    

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  
  





  //* cloud_with_normals = cloud + normals
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  ROS_INFO("Saving the file");
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles);
  std::string path = ros::package::getPath("pcl_zk");
  std::string path2 = ros::package::getPath("pcl_zk");
  std::string path3 = ros::package::getPath("pcl_zk");
  //path=path+"/data";
  std::string nome="/data/mesh.stl";
  std::string nome2="/data/mesh.pcd";
  std::string nome3="/data/mesh_prova.pcd";
  path=path+nome;
  path2=path2+nome2;
  path3=path3+nome3;
  ROS_INFO("Getting the pkg");

  pcl::io::savePCDFileASCII (path2, *cloud_with_normals);
  pcl::io::savePCDFileASCII (path3, *cloud);

  // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    viewer.addPointCloud(cloud);
    
    pcl::visualization::PCLVisualizer viewer_normals("PCL Viewer Normals");
    viewer_normals.setBackgroundColor (0.0, 0.0, 0.5);
    viewer_normals.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
   
    while (!viewer_normals.wasStopped ())
    {
      viewer.spinOnce ();
    }
  pcl::io::savePolygonFileSTL(path, triangles);
  ros::param::set("need_to_transform",true);
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles,true);
  
    

  ROS_INFO("ALL GOOD");
  return ;
}
*/
void from_file_to_stl(){

  std::string path = ros::package::getPath("pcl_zk");
  std::string nome="/data/mesh.pcd";
  path=path+nome;
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (path, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  ROS_INFO("PRINTING");
  pcl::io::savePolygonFileSTL("mesh.stl", triangles);
  // Finish
  
}

void from_my_manual_points_to_pointcloud(){


std::cout << "\n\n" << "starting program" << "\n\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);      // !!! with ::Ptr !!!

  int numPoints = 1000;

  for (int i = 0; i < numPoints; i++)
  {
    pcl::PointXYZ point;

    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);

    cloud->points.push_back(point);
  }

  // for simplicity, use an "unorganized" cloud, "width" = num points, "height" = 1
  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense=false;

  pcl::io::savePCDFileASCII("my_cloud.pcd", *cloud);

  std::cout << "\n\n" << "program complete" << "\n\n";





  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  
  





  //* cloud_with_normals = cloud + normals
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  ROS_INFO("Saving the file");
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles);
  std::string path = ros::package::getPath("pcl_zk");
  //path=path+"/data";
  std::string nome="/data/manual_mesh.stl";
  std::string path2=path+"/data/manual_mesh.vtk";
  path=path+nome;
  std::string path1=path+"1";
  pcl::io::savePolygonFileSTL(path1, triangles);
  pcl::io::saveVTKFile(path2, triangles);
  ros::param::set("need_to_transform",true);
  ROS_INFO("FINISH TO SAVE MESH");

  
}

void view_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptrcloud)
{
  ROS_INFO("Printing cloud");

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (ptrcloud);

  while (!viewer.wasStopped ())
  {
  }
}

int main(int argc, char **argv)
{
    // Initialize ROS
  ros::init(argc, argv, "greedy_projection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  tf::StampedTransform transform;
  sleep(1);
  //listener.waitForTransform("/base", "/cameradepth_link", t, ros::Duration(4.0));
  try{
  listener.lookupTransform("/base","/cameradepth_link",ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  ROS_INFO("PRONTO");

  ros::param::set("/need_to_hadle_cloud",false);
  //from_file_to_stl();
  /*
  ROS_INFO("DO you want create a pcd o convert to an stl?\n 1)stl \n 2)pcd");
  int scelta;
  cin>>scelta;
  if (scelta==1)
    from_file_to_stl();
  if(scelta==2)
    from_my_manual_points_to_pointcloud();
*/
  // Spin
  do{
  ros::param::get("/need_to_hadle_cloud",bool_need);
  ros::spinOnce();
  }while(!fine);
  

  ROS_INFO("CLOSING");
  return 0;
}

