cmake_minimum_required(VERSION 3.1)
project(g2o_karto)


#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

ADD_LIBRARY(g2o_karto SHARED
  edge_se2_pointxy.cpp  se2.h                vertex_point_xy.h
  edge_se2.cpp    edge_se2_pointxy.h    vertex_se2.cpp
  edge_se2.h      rand.h                vertex_point_xy.cpp  vertex_se2.h
  parameter_se2_offset.h parameter_se2_offset.cpp
  types_tutorial_slam2d.h types_tutorial_slam2d.cpp
  g2o_tutorial_slam2d_api.h
        )

TARGET_LINK_LIBRARIES(g2o_karto g2o_core g2o_csparse_extension)
