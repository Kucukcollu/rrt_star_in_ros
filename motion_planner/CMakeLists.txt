cmake_minimum_required(VERSION 3.0.2)
project(motion_planner)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(ompl REQUIRED)

find_package(catkin 
  REQUIRED COMPONENTS 
  geometry_msgs 
  roscpp
  nav_msgs
  visualization_msgs
  std_msgs
  message_generation
)

# Custom service
add_service_files(
  FILES
  PlanningService.srv
)

# messages that are used in the service
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  nav_msgs 
  geometry_msgs
  visualization_msgs
  std_msgs
  message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

# Header files
FILE(GLOB_RECURSE HeaderFiles "include/*.hpp")
add_custom_target(headers SOURCES ${HeaderFiles})

# Motion Planner
add_executable(motion_planner src/main.cpp src/MotionPlanner.cpp)
target_link_libraries(motion_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(motion_planner ${catkin_EXPORTED_TARGETS})

# Obstacle Layer
add_executable(markers src/markers.cpp)
target_link_libraries(markers ${catkin_LIBRARIES})