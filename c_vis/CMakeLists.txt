cmake_minimum_required(VERSION 2.8.3)
project(c_vis)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}	
)

add_executable(VI src/ocv.cpp)
target_link_libraries(VI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Linea src/linea.cpp)
target_link_libraries(Linea ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

