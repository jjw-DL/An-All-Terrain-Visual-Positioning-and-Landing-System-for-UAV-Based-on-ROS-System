cmake_minimum_required(VERSION 2.8.3)
project(search_keypoints)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(terrain_shooting src/terrain_shooting.cpp)
target_link_libraries(terrain_shooting ${catkin_LIBRARIES})


add_executable(search_keypoints src/search_keypoints.cpp)
target_link_libraries(search_keypoints ${catkin_LIBRARIES})





