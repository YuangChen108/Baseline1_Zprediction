cmake_minimum_required(VERSION 3.0.2)
project(planning)
set(USE_ROS true)
set(USE_SS_DBUS false)

# add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

################ Find Pkg ROS ################
if(USE_ROS)
  find_package(Eigen3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
    quadrotor_msgs
    mapping
    roscpp
    nodelet
    pcl_ros
    pcl_conversions
    image_transport
    cv_bridge
  )

  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS quadrotor_msgs roscpp nodelet pcl_ros pcl_conversions
    LIBRARIES ${PROJECT_NAME} mapping traj_opt planner data_callback traj_server tl_fsm
    DEPENDS EIGEN3
  )

  include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
endif()

################ Find Pkg SSDBUS ################
if(USE_SS_DBUS)
  find_package(ss_one REQUIRED)
  find_package(asio REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
  )
endif()

############## traj opt lib #############
add_library(traj_opt
  # src/traj_opt/traj_opt_perching.cc
  # src/traj_opt/traj_opt_perching_yaw.cc
  src/traj_opt/traj_opt_perching_yaw_full_q.cc
  # src/traj_opt/traj_opt_perching_yaw_softmin.cc
  # src/traj_opt/traj_opt_perching_yaw_softmin_margin.cc
  # src/traj_opt/traj_opt_s3_tracking.cc
  src/traj_opt/traj_opt_s3_tracking_yaw.cc
  src/traj_opt/traj_opt_s3_goal.cc
  # src/traj_opt/traj_opt_s3_goal_yaw.cc
  src/traj_opt/traj_opt_visible_deform.cc
  # src/traj_opt/traj_opt_visible_deform_incre.cc
  src/traj_opt/traj_opt_util.cc
)
target_link_libraries(traj_opt
  ${catkin_LIBRARIES}
)

############## tlplanner lib #############
add_library(tl_planner
  src/planning/tlplanner.cpp
)
target_link_libraries(tl_planner
  traj_opt
  ${catkin_LIBRARIES}
)

############## plan node lib #############
add_library(planner
  src/node/plan_node.cpp
)
target_link_libraries(planner
  tl_planner
  traj_opt
  ${catkin_LIBRARIES}
)

############## datacallback node lib #############
if(USE_ROS)
add_library(data_callback
  src/node/data_callback/data_callback.cpp
  src/node/data_callback/data_callback_ros_interface.cpp
)
target_link_libraries(data_callback
  ${catkin_LIBRARIES}
)
endif()
if(USE_SS_DBUS)
add_library(data_callback
  src/node/data_callback/data_callback.cpp
  src/node/data_callback/data_callback_ssdbus_interface.cpp
)
target_link_libraries(data_callback
  ${catkin_LIBRARIES}
)
endif()

############## traj_server node lib #############
if(USE_ROS)
add_library(traj_server_node
  src/node/traj_server/traj_server_node.cpp
  src/node/traj_server/traj_server_ros_interface.cpp
)
target_link_libraries(traj_server_node
  ${catkin_LIBRARIES}
)
endif()
if(USE_SS_DBUS)
add_library(traj_server_node
  src/node/traj_server/traj_server_node.cpp
  src/node/traj_server/traj_server_ssdbus_interface.cpp
)
target_link_libraries(traj_server_node
  ${catkin_LIBRARIES}
)
endif()

############## FSM node lib #############
add_library(tl_fsm
  src/node/tl_fsm_node.cpp
)
target_link_libraries(tl_fsm
  planner
  data_callback
  traj_server_node
  ${catkin_LIBRARIES}
)

############## MAIN node #############
add_executable(planning src/node/main_node.cpp)
target_link_libraries(planning
  tl_fsm 
  ${catkin_LIBRARIES}
)
add_dependencies(planning ${catkin_EXPORTED_TARGETS})
