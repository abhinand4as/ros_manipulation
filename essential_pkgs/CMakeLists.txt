cmake_minimum_required(VERSION 2.8.3)
project(essential_pkgs)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_ros_planning_interface
             cmake_modules
             eigen_conversions
	     rviz_visual_tools
	     moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_ros_planning_interface
    eigen_conversions
)
find_package(Eigen REQUIRED)

find_package(kinova_driver REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})

add_executable(work_scene_dev src/work_scene_dev.cpp )
target_link_libraries(work_scene_dev ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS work_scene_dev DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mg_interface src/mg_interface.cpp )
target_link_libraries(mg_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS mg_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
