cmake_minimum_required(VERSION 2.8.3)
project(v4r_ellipses)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  geometry_msgs
  roscpp
  rospy
  tf
  nodelet
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  ellipse.msg
  ellipses.msg
  marker.msg
  markers.msg
  TransformArrayStamped.msg)
  
generate_messages(DEPENDENCIES geometry_msgs)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/EllipsesDetection.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v4r_ellipses
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_geometry image_transport roscpp rospy tf
  DEPENDS eigen opencv
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(v4r_utils 
   src/v4r_utils/contour.cpp 
   src/v4r_utils/canny.cpp
   src/v4r_utils/camera.cpp
   src/v4r_utils/ellipse_refinement.cpp
)

## Declare a cpp library
add_library(v4r_ellipses
   src/${PROJECT_NAME}/v4r_ellipses.cpp
   src/${PROJECT_NAME}/v4r_ellipses_draw.cpp
   src/${PROJECT_NAME}/v4r_ellipses_parameters.cpp
)

add_library(v4r_ellipses_nodelet
   src/v4r_ellipses_nodelet.cpp
   src/v4r_ellipses_nodelet_parameters.cpp
)

## Declare a cpp executable
add_executable(v4r_ellipses_node src/v4r_ellipses_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(v4r_ellipses_node v4r_ellipses_generate_messages_cpp)
add_dependencies(v4r_ellipses_nodelet ${PROJECT_NAME}_gencfg) 

## Specify libraries to link a library or executable target against
target_link_libraries(v4r_ellipses_nodelet
   v4r_ellipses v4r_utils ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
target_link_libraries(v4r_ellipses_node
   v4r_ellipses_nodelet ${catkin_LIBRARIES}
)