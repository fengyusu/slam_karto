cmake_minimum_required(VERSION 3.1)
project(slam_karto)


set(CMAKE_CXX_STANDARD 14)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    rosconsole
    roscpp
    sensor_msgs
#    open_karto
#    sparse_bundle_adjustment
    tf
    visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(Cholmod)
find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} 
  src
  src/spa
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  )
  

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package()

add_subdirectory(src/g2o_karto)
add_subdirectory(src/open_karto)


add_executable(slam_karto src/slam_karto.cpp
                          src/g2o_solver.cpp
                          src/ceres_solver.cpp
                          src/spa_solver.cpp src/spa/spa2d.cpp src/spa/csparse.cpp)
target_link_libraries(slam_karto 
${catkin_LIBRARIES}
${CSPARSE_LIBRARY}
${CERES_LIBRARIES}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
karto
g2o_karto
 )

install(TARGETS slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(teleopt_nav src/karto_teleopt.cpp)
target_link_libraries(teleopt_nav ${catkin_LIBRARIES})
