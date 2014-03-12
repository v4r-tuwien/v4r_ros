cmake_minimum_required(VERSION 2.8.3)
project(v4r_uvc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure image_transport camera_info_manager message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread signals program_options)



find_package( OpenCV REQUIRED )

add_definitions(-DUSE_SDL -DLINUX -D_GNU_SOURCE=1 -D_REENTRANT )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Sphere.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/CameraGeneral.cfg
  cfg/CameraLogitech.cfg
  cfg/CameraLogitechSphere.cfg
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
  INCLUDE_DIRS include
  LIBRARIES v4r_uvc v4r_uvc_ros
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/SDL
  ./
  )

## Declare a cpp library
add_library(v4r_uvc
  src/luvcview/color.c
  src/luvcview/utils.c
  src/luvcview/v4l2uvc.c
)
target_link_libraries(v4r_uvc m SDL v4l2)

add_library(v4r_uvc_ros
  src/v4r_uvc/v4r_uvc.cpp 
  src/v4r_uvc/v4r_uvc_ros.cpp
)

target_link_libraries(v4r_uvc_ros m v4l2 v4r_uvc v4r_uvc_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS}
)

## Declare a cpp executable
add_executable(v4r_uvc_general_node src/v4r_uvc_general_node.cpp)
add_executable(v4r_uvc_logitech_node src/v4r_uvc_logitech_node.cpp)
add_executable(v4r_uvc_logitech_sphere_node src/v4r_uvc_logitech_sphere_node.cpp)

add_executable(update_dynamic_reconfigure src/v4r_uvc/v4r_uvc.cpp src/update_dynamic_reconfigure.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(v4r_uvc_general_node ${PROJECT_NAME}_gencfg) 
add_dependencies(v4r_uvc_logitech_node ${PROJECT_NAME}_gencfg) 
add_dependencies(v4r_uvc_logitech_sphere_node ${PROJECT_NAME}_gencfg) 

## Specify libraries to link a library or executable target against
target_link_libraries(v4r_uvc_general_node v4r_uvc_ros
  ${catkin_LIBRARIES})
  
target_link_libraries(v4r_uvc_logitech_node v4r_uvc_ros
  ${catkin_LIBRARIES})
  
target_link_libraries(v4r_uvc_logitech_sphere_node v4r_uvc_ros
  ${catkin_LIBRARIES})
  
target_link_libraries(update_dynamic_reconfigure m v4l2 v4r_uvc 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS})
  