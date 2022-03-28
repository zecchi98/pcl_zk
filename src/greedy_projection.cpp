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

//ctmviewer
bool fine=false;
bool bool_need=false;
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
  std::string nome="/data/mesh.stl";
  path=path+nome;
  pcl::io::savePolygonFileSTL(path, triangles);
  ros::param::set("need_to_transform",true);
  //pcl::io::savePolygonFilePLY("mesh.ply", triangles,true);
  
  /*
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
      viewer.spinOnce ();
    }*/

  ROS_INFO("ALL GOOD");
  return ;
}

void from_file_to_stl(){

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
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

  pcl::io::savePolygonFileSTL("mesh.stl", triangles);
  // Finish
  
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

  ros::param::set("/need_to_hadle_cloud",false);

  // Spin
  do{
  ros::param::get("/need_to_hadle_cloud",bool_need);
  ros::spinOnce();
  ROS_INFO("CLOSING");
  }while(!fine);
  ROS_INFO("CLOSING");
  return 0;
}

