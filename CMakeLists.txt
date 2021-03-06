cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(discrete SHARED discrete.cc A_star/A_star.cpp A_star/node.cpp)
target_link_libraries(discrete ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(kinPoint SHARED kinPoint.cc V_graph/V_graph.cpp)
target_link_libraries(kinPoint ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(dynamicPoint SHARED dynamicPoint.cc DIMT_RRT/WaypointDynamicFollower.cpp V_graph/V_graph.cpp)
target_link_libraries(dynamicPoint ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(kinCar SHARED kinCar.cc V_graph/V_graph.cpp)
target_link_libraries(kinCar ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(diffDrivePP SHARED diffDrivePP.cc DIFF_DRIVE/DDWPFollower.cpp V_graph/V_graph.cpp)
target_link_libraries(diffDrivePP ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(dynCar SHARED dynCar.cc V_graph/V_graph.cpp)
target_link_libraries(dynCar ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
