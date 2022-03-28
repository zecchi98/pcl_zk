cmake_minimum_required(VERSION 3.0.2)
project(pcl_zk)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()


find_package(PCL REQUIRED)
if(PCL_FOUND AND ("${PCL_VERSION}" VERSION_LESS 1.9))
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9)
else()
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
geometry_msgs
message_generation
roslib

actionlib
cmake_modules
mesh_segmenter
noether_conversions  
noether_msgs
path_sequence_planner
pcl_conversions
roslib
tool_path_planner

)


find_package(vtk_viewer REQUIRED)
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      actionlib
      mesh_segmenter
      path_sequence_planner
      pcl_conversions
      roscpp
      roslib
      tool_path_planner
    DEPENDS
      VTK
      PCL
      vtk_viewer)


      include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
      )

add_executable(plane_slicer_example_node  src/plane_slicer_raster_generator_node.cpp)
target_link_libraries(plane_slicer_example_node ${catkin_LIBRARIES} noether::vtk_viewer)
list (APPEND PACKAGE_TARGETS plane_slicer_example_node)

add_executable(greedy_projection  src/greedy_projection.cpp)
target_link_libraries(greedy_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} noether::vtk_viewer)
list (APPEND PACKAGE_TARGETS greedy_projection)

catkin_install_python(PROGRAMS
scripts/correct_ply.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PACKAGE_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)