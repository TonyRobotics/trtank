cmake_minimum_required(VERSION 2.8.3)
project(tank_driver)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(hello_world_node src/hello_world_node.cpp)
add_dependencies(hello_world_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hello_world_node
  ${catkin_LIBRARIES}
)

add_executable(listener src/listener.cpp)
add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(listener
  ${catkin_LIBRARIES}
)

