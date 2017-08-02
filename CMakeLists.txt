cmake_minimum_required(VERSION 2.8.3)
project(hsd_pcl_tutorial)
SET(FAST_PCL_PACKAGES filters registration)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # pcl
  # pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  #catkin_ceres_msgs
  ceres_msgs
  infant_msgs
  tf
  nav_msgs
  ${FAST_PCL_PACKAGES}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(PCL REQUIRED)
find_package(PCL 1.8.0 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES my_pcl_tutorial
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs ${FAST_PCL_PACKAGES}
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# IF(PCL_VERSION VERSION_LESS "1.7.2")
	# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
# ELSE(PCL_VERSION VERSION_LESS "1.7.2")
	# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
# ENDIF(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Declare a C++ library
# add_library(my_pcl_tutorial
#   src/${PROJECT_NAME}/my_pcl_tutorial.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_pcl_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(my_pcl_tutorial_node src/my_pcl_tutorial_node.cpp)
#add_executable(global_obstacle_map_creater src/global_obstacle_map_creater.cpp)
#add_executable(global_obstacle_map_creater_before_crcl src/global_obstacle_map_creater_before_crcl.cpp)
#add_executable(global_obstacle_map_creater_before_crcl_for_3d_ndt src/global_obstacle_map_creater_before_crcl_for_3d_ndt.cpp)
#add_executable(global_obstacle_map_creater_after_crcl src/global_obstacle_map_creater_after_crcl.cpp)
#add_executable(global_obstacle_map_creater_after_crcl_for_ndt src/global_obstacle_map_creater_after_crcl_for_ndt.cpp)
# add_executable(global_obstacle_map_creater_after_crcl_for_3d_ndt src/global_obstacle_map_creater_after_crcl_for_3d_ndt.cpp)
#add_executable(global_obstacle_map_creater_tsukuba src/global_obstacle_map_creater_tsukuba.cpp)
# add_executable(hosoda_gauss src/hosoda_gauss.cpp)
# add_executable(hsd_gauss src/hsd_gauss.cpp)
# add_executable(hsd_normal_shimizu src/hsd_normal_shimizu.cpp)
add_executable(hsd_normal_shimizu2 src/hsd_normal_shimizu2.cpp)
# add_executable(hsd_normal_x src/hsd_normal_x.cpp)
add_executable(region_growing_segmentation src/region_growing_segmentation.cpp)
# add_executable(otamesi src/otamesi.cpp)
add_executable(calc_shape src/calc_shape.cpp)
add_executable(calc_shape_savecloud src/calc_shape_savecloud.cpp)
add_executable(save_velodyne_points src/save_velodyne_points.cpp)
# add_executable(remove_cluster src/remove_cluster.cpp)
add_executable(rwrc14_remove_cluster_for_3dmap src/rwrc14_remove_cluster_for_3dmap.cpp)
add_executable(rwrc14_remove_cluster_for_3dmap_filter src/rwrc14_remove_cluster_for_3dmap_filter.cpp)
# add_executable(detect_peak src/detect_peak.cpp)
add_executable(detect_peak_vizu src/detect_peak_vizu.cpp)
add_executable(detect_peak_savecloud src/detect_peak_savecloud.cpp)
# add_executable(detect_peak_vizu2 src/detect_peak_vizu2.cpp)
add_executable(intersection_detection src/intersection_detection.cpp)
# add_executable(filter1 src/filter1.cpp)
# add_executable(filter2 src/filter2.cpp)
add_executable(filter3 src/filter3.cpp)
add_executable(ransac_example src/ransac_example.cpp)
#
#add_executable(example_ndt src/example_ndt.cpp)
#add_executable(example_fast_ndt src/example_fast_ndt.cpp)
#add_executable(example_fast_3d_ndt_ver2 src/example_fast_3d_ndt_ver2.cpp)
#add_executable(example_voxel_grid src/example_voxel_grid.cpp)
#add_executable(example_pcd src/example_pcd.cpp)
#
#add_executable(example src/example.cpp)
#add_executable(example1 src/example1.cpp)
#add_executable(example2 src/example2.cpp)
#add_executable(example3 src/example3.cpp)
#add_executable(example4 src/example4.cpp)
#add_executable(example5 src/example5.cpp)
#add_executable(example6 src/example6.cpp)
#add_executable(example7 src/example7.cpp)
#add_executable(example8 src/example8.cpp)
#add_executable(example9 src/example9.cpp)
#add_executable(example10 src/example10.cpp)
#add_executable(example11 src/example11.cpp)
# add_executable(example11_ver2 src/example11_ver2.cpp)
#add_executable(example11_with_tom_laser src/example11_with_tom_laser.cpp)
#add_executable(example12 src/example12.cpp)
#add_executable(example12_fast_ndt src/example12_fast_ndt.cpp)
#add_executable(example12_fast_ndt_ver2 src/example12_fast_ndt_ver2.cpp)
# add_executable(example12_fast_3d_ndt src/example12_fast_3d_ndt.cpp)
#add_executable(example12_fast_3d_ndt_for_vis src/example12_fast_3d_ndt_for_vis.cpp)
#add_executable(test1 src/test1.cpp)
#add_executable(sample_pub_odom src/sample_pub_odom.cpp)
#add_executable(sample_pub_odom2 src/sample_pub_odom2.cpp)
#add_executable(sample_pub_wp_mode src/sample_pub_wp_mode.cpp)
#add_executable(sample_pub_local_goal src/sample_pub_local_goal.cpp)
#
#add_executable(Dgauss_score src/Dgauss_score.cpp)
#add_executable(NDT_score src/NDT_score.cpp)
#
#add_executable(infant_height_map_220 src/infant_height_map_220.cpp)
#add_executable(infant_height_map_221 src/infant_height_map_221.cpp)
#add_executable(infant_height_map_222 src/infant_height_map_222.cpp)

# add_executable(infant_curvature_220 src/infant_curvature_220.cpp)
# add_executable(infant_curvature_221 src/infant_curvature_221.cpp)
# add_executable(infant_curvature_222 src/infant_curvature_222.cpp)
#add_executable(infant_curvature_220_new src/infant_curvature_220_new.cpp)
#add_executable(infant_curvature_221_new src/infant_curvature_221_new.cpp)
#add_executable(infant_curvature_222_new src/infant_curvature_222_new.cpp)
#
#add_executable(vis_tom_laser src/vis_tom_laser.cpp)
#
#add_executable(create_tom_laser_local_map src/create_tom_laser_local_map.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_pcl_tutorial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(my_pcl_tutorial_node
  # ${catkin_LIBRARIES}
# )
#target_link_libraries(global_obstacle_map_creater
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(global_obstacle_map_creater_before_crcl
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(global_obstacle_map_creater_before_crcl_for_3d_ndt
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(global_obstacle_map_creater_after_crcl
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(global_obstacle_map_creater_after_crcl_for_ndt
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
# target_link_libraries(global_obstacle_map_creater_after_crcl_for_3d_ndt
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
#target_link_libraries(global_obstacle_map_creater_tsukuba
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#target_link_libraries(example_ndt
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example_fast_ndt
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example_fast_3d_ndt_ver2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example_voxel_grid
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example_pcd
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#target_link_libraries(example
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example1
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example3
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example4
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example5
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example6
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
# target_link_libraries(hosoda_gauss
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
# target_link_libraries(hsd_gauss
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
# target_link_libraries(hsd_normal_shimizu
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(hsd_normal_shimizu2
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(hsd_normal_x
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(region_growing_segmentation
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(otamesi
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(calc_shape
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
target_link_libraries(calc_shape_savecloud
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
target_link_libraries(save_velodyne_points
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(remove_cluster
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(rwrc14_remove_cluster_for_3dmap
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
target_link_libraries(rwrc14_remove_cluster_for_3dmap_filter
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(detect_peak
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(detect_peak_vizu
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
target_link_libraries(detect_peak_savecloud
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(detect_peak_vizu2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(intersection_detection
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
# target_link_libraries(filter1
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
# target_link_libraries(filter2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
# )
target_link_libraries(filter3
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
target_link_libraries(ransac_example
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)

#target_link_libraries(example8
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example9
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example10
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example11
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
# target_link_libraries(example11_ver2
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
#target_link_libraries(example11_with_tom_laser
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example12
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example12_fast_ndt
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(example12_fast_ndt_ver2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
# target_link_libraries(example12_fast_3d_ndt
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
#target_link_libraries(example12_fast_3d_ndt_for_vis
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(test1
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(sample_pub_odom
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(sample_pub_odom2
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(sample_pub_wp_mode
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(sample_pub_local_goal
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#
#target_link_libraries(Dgauss_score
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(NDT_score
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#target_link_libraries(infant_height_map_220
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(infant_height_map_221
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(infant_height_map_222
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
# target_link_libraries(infant_curvature_220
  # ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
# )
# target_link_libraries(infant_curvature_221
  # ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
# )
# target_link_libraries(infant_curvature_222
  # ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
# )
#target_link_libraries(infant_curvature_220_new
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(infant_curvature_221_new
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#target_link_libraries(infant_curvature_222_new
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#target_link_libraries(vis_tom_laser
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#
#target_link_libraries(create_tom_laser_local_map
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_pcl_tutorial my_pcl_tutorial_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
  # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
