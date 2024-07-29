cmake_minimum_required(VERSION 3.0.2)
project(zerog_utilities)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    moveit_servo
    interactive_markers
    moveit_core
    moveit_ros_perception
    moveit_ros_planning_interface
    pluginlib
    roscpp
    std_msgs
    moveit_visual_tools
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    message_filters
    sensor_msgs
    pcl_ros
    pcl_conversions
    visualization_msgs
    message_generation # MRM: 17-06-2023: Added this for custom message generation: manipulability index stamped
    dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#     FILES
#     ManipulabilityIndexStamped.msg # MRM: 17-07-2023: Added this for custom message generation: manipulability index stamped
#     JacobianStamped.msg
#     EigenValuesStamped.msg
#     ConditionNumberStamped.msg
#     EnergyStamped.msg
#     InertiaMatrixStamped.msg
#  )

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
   std_msgs  # MRM: 17-07-2023: Need this as dependancy
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/TfInfo.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zerog_utils
CATKIN_DEPENDS geometry_msgs message_runtime message_filters tf2 tf2_ros tf2_geometry_msgs moveit_servo interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs sensor_msgs dynamic_reconfigure
#  DEPENDS system_lib
)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${Boost_INCLUDE_DIR} )
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obtain_waypts_transform_target src/obtain_waypts_transform_target.cpp)
target_link_libraries(obtain_waypts_transform_target ${catkin_LIBRARIES})

add_executable(obtain_waypts_transform_chaser src/obtain_waypts_transform_chaser.cpp)
target_link_libraries(obtain_waypts_transform_chaser ${catkin_LIBRARIES})

add_executable(obtain_waypts_transform src/obtain_waypts_transform.cpp)
target_link_libraries(obtain_waypts_transform ${catkin_LIBRARIES})

add_executable(reference_transformer_target src/reference_transformer_target.cpp)
target_link_libraries(reference_transformer_target ${catkin_LIBRARIES})

add_executable(reference_transformer_chaser src/reference_transformer_chaser.cpp)
target_link_libraries(reference_transformer_chaser ${catkin_LIBRARIES})

add_executable(tf_info_node src/tf_info_node.cpp)
target_link_libraries(tf_info_node ${catkin_LIBRARIES})

add_executable(create_waypoints src/create_waypoints.cpp)
target_link_libraries(create_waypoints ${catkin_LIBRARIES} ${tf2_LIBRARIES})

add_executable(reference_transformer src/reference_transformer.cpp)
target_link_libraries(reference_transformer ${catkin_LIBRARIES})


add_executable(twist_tf_info_node src/twist_tf_info_node.cpp)
add_dependencies(twist_tf_info_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_tf_info_node ${catkin_LIBRARIES})
