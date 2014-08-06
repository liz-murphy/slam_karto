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
    vertigo_slam
)
find_package(Eigen REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(G2O REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_library(g2o_calibration ${G2O_LIB_TYPE} calibration/gm2dl_io.cpp calibration/edge_se2_pure_calib.cpp calibration/closed_form_calibration.cpp calibration/sclam_pure_calibration.cpp calibration/sclam_helpers.cpp calibration/types_sclam2d.cpp src/g2o/types/data/robot_laser_sclam.cpp)
#add_executable(slam_karto src/g2o/g2o_solver.cpp src/g2o/vertigo_switchable_solver.cpp src/g2o/vertigo_maxmix_solver.cpp src/slam_karto.cpp)
add_library(slam_karto_g2o src/g2o/g2o_solver.cpp src/g2o/vertigo_switchable_solver.cpp src/g2o/vertigo_maxmix_solver.cpp src/spa_solver.cpp)
add_executable(slam_karto src/slam_karto.cpp)
add_executable(sclam calibration/sclam_pure_calibration.cpp)
message("G2O Libraries: " ${G2O_LIBRARIES})
target_link_libraries(g2o_calibration ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES})
target_link_libraries(slam_karto_g2o ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES} g2o_calibration)
target_link_libraries(slam_karto slam_karto_g2o)
target_link_libraries(sclam g2o_calibration)
install(TARGETS slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
