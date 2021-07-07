cmake_minimum_required(VERSION 2.8.3)
project(bebop_simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  mav_msgs
  nav_msgs
  std_msgs
  roscpp
  sensor_msgs
  cmake_modules
  message_generation
  rospy
)

find_package(
  Eigen REQUIRED
)

add_message_files(
  FILES
  ReferenceAngles.msg
  PosController.msg
  AttitudeController.msg
  Control.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mav_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES position_controller extendedKalmanFilter position_controller_with_bebop waypoint_filter pos_controller pos_controller_2 pos_controller_3
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs std_msgs message_runtime
  DEPENDS Eigen 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/plot_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/plot_3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(position_controller
   src/library/position_controller.cpp
)

add_library(position_controller_with_bebop
   src/library/position_controller_with_bebop.cpp
)

add_library(extendedKalmanFilter
   src/library/extendedKalmanFilter.cpp
)

add_library(waypoint_filter
   src/library/waypoint_filter.cpp
)

add_library(pos_controller
   src/library/pos_controller.cpp
)

add_library(pos_controller_2
   src/library/pos_controller_2.cpp
)

add_library(pos_controller_3
   src/library/pos_controller_3.cpp
)

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller_with_bebop ${catkin_LIBRARIES})
add_dependencies(position_controller_with_bebop ${catkin_EXPORTED_TARGETS})

target_link_libraries(extendedKalmanFilter ${catkin_LIBRARIES})
add_dependencies(extendedKalmanFilter ${catkin_EXPORTED_TARGETS})

target_link_libraries(waypoint_filter ${catkin_LIBRARIES})
add_dependencies(waypoint_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(pos_controller ${catkin_LIBRARIES})
add_dependencies(pos_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(pos_controller_2 ${catkin_LIBRARIES})
add_dependencies(pos_controller_2 ${catkin_EXPORTED_TARGETS})

target_link_libraries(pos_controller_3 ${catkin_LIBRARIES})
add_dependencies(pos_controller_3 ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node
  position_controller extendedKalmanFilter waypoint_filter ${catkin_LIBRARIES})

add_executable(position_controller_with_bebop_node src/nodes/position_controller_with_bebop_node.cpp)
add_dependencies(position_controller_with_bebop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_with_bebop_node
  position_controller_with_bebop extendedKalmanFilter waypoint_filter  ${catkin_LIBRARIES})

add_executable(hovering_example src/nodes/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_example src/nodes/waypoint_example.cpp)
target_link_libraries(waypoint_example ${catkin_LIBRARIES})
add_dependencies(waypoint_example ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_gen src/nodes/waypoint_gen.cpp)
target_link_libraries(waypoint_gen ${catkin_LIBRARIES})
add_dependencies(waypoint_gen ${catkin_EXPORTED_TARGETS})

add_executable(launcher src/nodes/launcher.cpp)
target_link_libraries(launcher ${catkin_LIBRARIES})
add_dependencies(launcher ${catkin_EXPORTED_TARGETS})

add_executable(launcher_2 src/nodes/launcher_2.cpp)
target_link_libraries(launcher_2 ${catkin_LIBRARIES})
add_dependencies(launcher_2 ${catkin_EXPORTED_TARGETS})

add_executable(launcher_3 src/nodes/launcher_3.cpp)
target_link_libraries(launcher_3 ${catkin_LIBRARIES})
add_dependencies(launcher_3 ${catkin_EXPORTED_TARGETS})

add_executable(csv_odometry src/nodes/csv_odometry.cpp)
target_link_libraries(csv_odometry ${catkin_LIBRARIES})
add_dependencies(csv_odometry ${catkin_EXPORTED_TARGETS})

add_executable(csv_control src/nodes/csv_control.cpp)
target_link_libraries(csv_control ${catkin_LIBRARIES})
add_dependencies(csv_control ${catkin_EXPORTED_TARGETS})

add_executable(quaternion_to_rpy src/nodes/quaternion_to_rpy.cpp)
target_link_libraries(quaternion_to_rpy ${catkin_LIBRARIES})
add_dependencies(quaternion_to_rpy ${catkin_EXPORTED_TARGETS})

add_executable(pos_controller_node src/nodes/pos_controller_node.cpp)
add_dependencies(pos_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pos_controller_node
  pos_controller extendedKalmanFilter waypoint_filter ${catkin_LIBRARIES})

add_executable(pos_controller_node_2 src/nodes/pos_controller_node_2.cpp)
add_dependencies(pos_controller_node_2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(pos_controller_node_2
  pos_controller_2 extendedKalmanFilter waypoint_filter ${catkin_LIBRARIES})

add_executable(pos_controller_node_3 src/nodes/pos_controller_node_3.cpp)
add_dependencies(pos_controller_node_3 ${catkin_EXPORTED_TARGETS})
target_link_libraries(pos_controller_node_3
  pos_controller_3 extendedKalmanFilter waypoint_filter ${catkin_LIBRARIES})

install(TARGETS position_controller extendedKalmanFilter position_controller_with_bebop waypoint_filter pos_controller pos_controller_2 pos_controller_3
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller position_controller_with_bebop_node hovering_example waypoint_example waypoint_gen launcher launcher_2 launcher_3 quaternion_to_rpy pos_controller_node pos_controller_node_2 pos_controller_node_3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DIRECTORY include/bebop_simulator_msgs/
  DIRECTORY include/bebop_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
