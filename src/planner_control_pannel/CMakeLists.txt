cmake_minimum_required(VERSION 2.8.3)
project(planner_control_pannel)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs std_srvs pedsim_msgs pedsim_srvs pedsim_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5  REQUIRED COMPONENTS  Widgets  PrintSupport)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/qt_app_node.cpp
  src/control_pannel.cpp
  src/com_interface.cpp
  src/qcustomplot.cpp
  src/qledlabel.cpp
)

set(MOC_FILES
    include/com_interface.h
    include/qcustomplot.h
    include/qledlabel.h
    )

set(FORMS
  src/control_pannel.ui
)
qt5_wrap_cpp(MOC_SRCS_UI ${MOC_FILES})
qt5_wrap_ui(UI_HEADERS ${FORMS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planner_contro_pannel
#  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/planner_contro_pannel.cpp
# )
add_executable(${PROJECT_NAME}_node ${SOURCES} ${UI_HEADERS} ${MOC_SRCS_UI})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node  Qt5::Widgets Qt5::PrintSupport )
