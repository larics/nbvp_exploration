cmake_minimum_required(VERSION 2.8.3)
project(interface_nbvp_rotors)

find_package(catkin REQUIRED COMPONENTS
  nbvplanner
  trajectory_msgs
  mav_msgs
  tf
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf
)

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)

add_dependencies(exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_rsc src/test_rsc.cpp)
target_link_libraries(test_rsc
  ${catkin_LIBRARIES}
)
add_executable(tf_node src/tf2_broadcaster.cpp)
target_link_libraries(tf_node
  ${catkin_LIBRARIES}
)
