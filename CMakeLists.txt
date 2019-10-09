cmake_minimum_required(VERSION 2.8.3)
project(pose_save)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf2
)

find_package(Bullet REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)

# OpenCV is required
find_package(OpenCV 3 REQUIRED)
if("${OpenCV_VERSION}" VERSION_LESS "3.0")
  message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SavePose.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES pose_save
  CATKIN_DEPENDS
      roscpp
      rospy
      std_msgs
      message_runtime
      tf2
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(${PROJECT_NAME}_node src/pose_save.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(invert_pose_node src/invertpose.cpp)
add_dependencies(invert_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(invert_pose_node ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(use_geometry src/use_geometry.cpp)
add_dependencies(use_geometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(use_geometry ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pose_cal src/pose_cal.cpp)
add_dependencies(pose_cal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_cal ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(euler_quaternion src/euler_quaternion.cpp)
add_dependencies(euler_quaternion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(euler_quaternion ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(multi_map_pose_cal src/multi_map_pose_cal.cpp)
add_dependencies(multi_map_pose_cal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_map_pose_cal ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
