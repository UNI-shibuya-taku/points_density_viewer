cmake_minimum_required(VERSION 3.0.2)
project(points_density_viewer)

SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  nav_msgs
  sensor_msgs
  tf
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES point_cloud_storer
  # CATKIN_DEPENDS message_filters nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(points_density_viewer src/points_density_viewer.cpp)
target_link_libraries(points_density_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(compare_points_density_viewer src/compare_points_density_viewer.cpp)
target_link_libraries(compare_points_density_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(points_density_ploter src/points_density_ploter.cpp)
target_link_libraries(points_density_ploter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(scan_match_score src/scan_match_score.cpp)
target_link_libraries(scan_match_score
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)