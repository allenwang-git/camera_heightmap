cmake_minimum_required(VERSION 3.0.2)
project(camera_heightmap)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros roscpp
        sensor_msgs
        cv_bridge
        image_transport
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories("include")
include_directories(${OpenCV_INCLUDE_DIRS})

FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.hpp)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(pclload src/pclload.cpp)
add_executable(pclsave src/pclsave.cpp)
add_executable(downsample src/downsample.cpp )
add_executable(downsample435 src/downsample435.cpp )
add_executable(xlimit src/xlimit.cpp)
add_executable(ylimit src/ylimit.cpp)
add_executable(zlimit src/zlimit.cpp ${CURRENT_INCLUDE})
add_executable(pcdprocess src/pcdprocess.cpp ${CURRENT_INCLUDE})
add_executable(hsload src/hsload.cpp ${CURRENT_INCLUDE})
add_executable(pcdprocess435 src/pcdprocess435.cpp ${CURRENT_INCLUDE})
add_executable(depthprocess src/depthprocess.cpp ${CURRENT_INCLUDE})
#add_executable(pclfilted src/downsample.cpp src/xlimit.cpp src/ylimit.cpp src/zlimit.cpp)
target_link_libraries(downsample435 ${catkin_LIBRARIES})
target_link_libraries(downsample ${catkin_LIBRARIES})
target_link_libraries(pclsave ${catkin_LIBRARIES})
target_link_libraries(pclload ${catkin_LIBRARIES})
target_link_libraries(ylimit ${catkin_LIBRARIES})
target_link_libraries(xlimit ${catkin_LIBRARIES})
target_link_libraries(zlimit ${catkin_LIBRARIES})
target_link_libraries(zlimit lcm)
target_link_libraries(pcdprocess ${catkin_LIBRARIES})
target_link_libraries(pcdprocess lcm)
target_link_libraries(pcdprocess435 ${catkin_LIBRARIES})
target_link_libraries(pcdprocess435 lcm)
target_link_libraries(depthprocess ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(travprocess lcm)
#target_link_libraries(pclfilted ${catkin_LIBRARIES})
target_link_libraries(hsload ${catkin_LIBRARIES})
target_link_libraries(hsload lcm)
