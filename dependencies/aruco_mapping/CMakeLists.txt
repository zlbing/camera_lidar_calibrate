cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping)
#set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
             roscpp
             message_generation
             image_transport
             cv_bridge
             tf
             aruco
             visualization_msgs
             camera_calibration_parsers
             lidar_camera_calibration
             pcl_ros
             std_msgs
             sensor_msgs) 

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)

include_directories(${PROJECT_SOURCE_DIR}/src/)

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
            ${PROJECT_SOURCE_DIR}/src/aruco_mapping.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_mapping.h)

add_message_files(FILES ArucoMarker.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)
   
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} aruco_mapping_gencpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(aruco_create ${PROJECT_SOURCE_DIR}/src/create_marker.cpp)
target_link_libraries(aruco_create ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
 
add_executable(image_process ${PROJECT_SOURCE_DIR}/src/image_process.cpp)
target_link_libraries(image_process ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})