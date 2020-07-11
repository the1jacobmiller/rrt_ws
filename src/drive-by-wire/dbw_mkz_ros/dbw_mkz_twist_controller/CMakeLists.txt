cmake_minimum_required(VERSION 2.8.3)
project(dbw_mkz_twist_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  dbw_mkz_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
  cfg/TwistTest.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_controller src/twist_controller_node.cpp src/TwistControllerNode.cpp)
add_dependencies(twist_controller dbw_mkz_msgs_gencpp)
add_dependencies(twist_controller ${PROJECT_NAME}_gencfg)
target_link_libraries(twist_controller
  ${catkin_LIBRARIES}
)

install(TARGETS twist_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
