cmake_minimum_required(VERSION 2.8.3)
project(evaluation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_BUILD_TYPE Debug)

add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  pedsim
  pedsim_msgs
  pedsim_srvs
  pedsim_utils
  ped_navigation
  message_generation
)
find_package(Eigen3 REQUIRED COMPONENTS system)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})



add_message_files(
  FILES
  evaluation_sending.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES evaluation
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${quad_tree_lib_INCLUDE_DIRS}
)



set(XML_UTIL_RELATED
        src/xml_utils/tinystr.cpp
	src/xml_utils/tinyxml.cpp
	src/xml_utils/tinyxmlerror.cpp
	src/xml_utils/tinyxmlparser.cpp
)

set(PED_EVALUATOR
    src/evaluator.cpp
    src/evaluation/success_checker.cpp
    src/evaluation/collision_checker.cpp)

set(FRAME_COMPARATION
    src/compare_frame.cpp)

add_executable(evaluator_all src/evaluation.cpp ${XML_UTIL_RELATED})
add_executable(take_evaluation ${PED_EVALUATOR})
add_executable(take_comparation ${FRAME_COMPARATION} ${XML_UTIL_RELATED})
add_dependencies(evaluator_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(take_evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(take_comparation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluator_all ${catkin_LIBRARIES}
 )
target_link_libraries(take_evaluation ${catkin_LIBRARIES}
 )
target_link_libraries(take_evaluation quad_tree_lib)
target_link_libraries(take_comparation ${catkin_LIBRARIES}
 )
