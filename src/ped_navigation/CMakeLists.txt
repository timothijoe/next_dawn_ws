cmake_minimum_required(VERSION 3.0.2)
project(ped_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")


find_package(catkin REQUIRED
    geometry_msgs
    roscpp
    rospy
    std_msgs
    pedsim_msgs
    nav_msgs
    visualization_msgs
    message_generation
    std_srvs pedsim_srvs
    roslib)
find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(Qt5  REQUIRED COMPONENTS  Widgets XmlPatterns)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)


if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})



add_message_files(
  FILES

  d_triangle.msg
  d_triangles.msg
  map_utils.msg
  m_ped_xvi.msg
  m_static_map.msg
  m_density_map.msg
  m_velocity_map.msg
  m_cluster_map.msg
  p_opt_trajectory.msg

)

generate_messages(
  DEPENDENCIES geometry_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quad_tree_lib
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime message_generation roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(MAP_SOURCES
    src/map_generator/map_generator.cpp
    src/map_utils/base_map.cpp
    src/map_utils/density_map.cpp
    src/map_utils/static_map.cpp
    src/map_utils/velocity_map.cpp
    src/map_utils/cluster_map.cpp
    )

set(MAP_SOURCESS
    src/map_generator/og_map_builder.cpp
    src/map_utils/base_map.cpp
    src/map_utils/density_map.cpp
    src/map_utils/static_map.cpp
    src/map_utils/velocity_map.cpp
    src/map_utils/cluster_map.cpp
    )

set(DECISION_MAKER
    src/lattice/decision_maker.cpp
    src/lattice/map_receiver.cpp
    src/lattice/diff_drive_generator/motion_model_diff_drive.cpp
    src/lattice/diff_drive_generator/trajectory_generator_diff_drive.cpp
    src/lattice/lattice_lookup_table/lookup_table_utils.cpp)

set(DECISION_MAKERR
    src/task_manager/taskmanager.cpp
    src/task_manager/flowsegments.cpp
    src/lattice/map_receiver.cpp
    src/lattice/diff_drive_generator/motion_model_diff_drive.cpp
    src/lattice/diff_drive_generator/trajectory_generator_diff_drive.cpp
    src/lattice/lattice_lookup_table/lookup_table_utils.cpp)

set(POINT_SAMPLER
    src/particle_filter/point_sampler.cpp
    src/task_manager/flowsegments.cpp
    src/lattice/map_receiver.cpp
    src/lattice/diff_drive_generator/motion_model_diff_drive.cpp
    src/lattice/diff_drive_generator/trajectory_generator_diff_drive.cpp
    src/lattice/lattice_lookup_table/lookup_table_utils.cpp)

set(LOCAL_PLANNER
    src/lattice/local_lattice_planner.cpp
    src/lattice/lattice_base.cpp
    src/lattice/map_receiver.cpp
    src/lattice/diff_drive_generator/motion_model_diff_drive.cpp
    src/lattice/diff_drive_generator/trajectory_generator_diff_drive.cpp
    src/lattice/lattice_lookup_table/lookup_table_utils.cpp)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_library(quad_tree_lib
    src/map_generator/quad_tree.cpp)

add_library(delaunay_triangle_lib
    src/graph/delaunator.cpp)



add_library(lookup_table_generator_lib
    src/lattice/lattice_lookup_table/lookup_table_generator.cpp
    src/lattice/lattice_lookup_table/lookup_table_utils.cpp)

add_library(motion_model_diff_drive
    src/lattice/diff_drive_generator/motion_model_diff_drive.cpp)
add_library(trajectory_generator_diff_drive
    src/lattice/diff_drive_generator/trajectory_generator_diff_drive.cpp)

add_dependencies(quad_tree_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(delaunay_triangle_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(motion_model_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lookup_table_generator_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_generator_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(motion_model_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(trajectory_generator_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(map_utils ${MAP_SOURCES})
add_executable(map_utilss ${MAP_SOURCESS})
add_executable(decision_maker ${DECISION_MAKER})
add_executable(decision_makerr ${DECISION_MAKERR})
add_executable(point_sampler ${POINT_SAMPLER})
add_executable(local_planner ${LOCAL_PLANNER})
add_executable(lookup_table_generator src/lattice/lattice_lookup_table/lookup_table_generator_node.cpp)

add_dependencies(map_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_utilss ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(decision_maker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(decision_makerr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(point_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lookup_table_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_utils
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    quad_tree_lib)
target_link_libraries(map_utilss
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    quad_tree_lib)

target_link_libraries(decision_maker
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    delaunay_triangle_lib)
target_link_libraries(decision_makerr
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    delaunay_triangle_lib)
target_link_libraries(point_sampler
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    delaunay_triangle_lib)
target_link_libraries(local_planner
    ${catkin_LIBRARIES} ${OpenCV_LIBS}
    delaunay_triangle_lib)

target_link_libraries(lookup_table_generator
  ${catkin_LIBRARIES}
  lookup_table_generator_lib
  trajectory_generator_diff_drive
  motion_model_diff_drive
)
