cmake_minimum_required(VERSION 2.8.3)
project(rotors_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    rotors_comm
    rotors_control
    )


find_package(Eigen REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)


## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
    LIBRARIES cl_rrt
    CATKIN_DEPENDS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs rotors_comm rotors_control
    DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_LIBRARIES}

)

#find_package(octomap REQUIRED)
#include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_libraries(${OCTOMAP_LIBRARIES})

#find_package(ompl REQUIRED)
#include_directories(${OMPL_INCLUDE_DIRS})
#link_libraries(${OMPL_LIBRARIES})

add_library(cl_rrt
  src/library/cl_rrt.cpp
)
target_link_libraries(cl_rrt ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(cl_rrt ${catkin_EXPORTED_TARGETS})

add_executable(RRT_planner src/RRT_planner.cpp)
target_link_libraries(RRT_planner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(RRT_planner ${catkin_EXPORTED_TARGETS})

add_executable(CL_RRT_test src/CL_RRT_test.cpp)
target_link_libraries(CL_RRT_test cl_rrt ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(CL_RRT_test ${catkin_EXPORTED_TARGETS})

install(TARGETS cl_rrt
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS RRT_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
