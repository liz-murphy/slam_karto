cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
message("CMAKE module path: " ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
    vertigo
)
find_package(Eigen REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(G2O REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_executable(slam_karto src/slam_karto.cpp src/g2o_solver.cpp src/vertigo_switchable_solver.cpp src/vertigo_maxmix_solver.cpp)
message("Catkin Libraries: " ${catkin_LIBRARIES})
target_link_libraries(slam_karto ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES})

install(TARGETS slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
