cmake_minimum_required(VERSION 2.8.3)
project( serial_node )

 find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    std_msgs
  )
 
  catkin_package(
   CATKIN_DEPENDS
     serial
     std_msgs
 )


include_directories( ${catkin_INCLUDE_DIRS})

add_executable( serial_node src/serial_node.cpp)
target_link_libraries( serial_node ${catkin_LIBRARIES})
 
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(degree src/degree.cpp)
target_link_libraries(degree ${catkin_LIBRARIES})


