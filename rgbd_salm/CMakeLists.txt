cmake_minimum_required(VERSION 2.8.3)
project(rgbd_salm)

#list( APPEND CMAKE_MODULE_PATH /home/leo/Downloads/g2o/cmake_modules )
#set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

#SET( G2O_ROOT /usr/local/include/g2o )
#find_package(G2O REQUIRED)
#find_package(CSparse REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${G2O_INCLUDE_DIRS}
#  ${CSPARSE_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/slam_base.cpp src/slam_base.h src/slam.cpp src/slam.h)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
#   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_types_slam3d g2o_solver_csparse
#   ${CSPARSE_LIBRARY}
 )

