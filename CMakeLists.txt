cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(lidar_camera_calibration)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
  image_geometry
  velodyne_msgs
  velodyne_pointcloud
  pcl_ros
)

add_message_files(FILES marker_6dof.msg)
generate_messages(DEPENDENCIES
                  std_msgs
                  )

catkin_package(CATKIN_DEPENDS std_msgs )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

find_package(Eigen3  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(image_laser_process src/image_laser_process.cpp)
target_link_libraries(image_laser_process ${OpenCV_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

find_package(Ceres REQUIRED)

include_directories(src/Config.h
        src/fusion_lidar_camera.h
        src/compute_rt.h
        ${CERES_INCLUDE_DIRS})

add_executable(gs_calibrate src/main.cc
        src/Config.cc
        src/fusion_lidar_camera.cc
        src/compute_rt.cc)
target_link_libraries(gs_calibrate ${OpenCV_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})