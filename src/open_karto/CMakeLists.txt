cmake_minimum_required(VERSION 3.1)
project(open_karto)


find_package(Boost REQUIRED COMPONENTS thread)


add_library(karto SHARED 
            Karto.cpp Mapper.cpp
            ScanMatcherBasedOptimize.cpp
            Karto.h Mapper.h Exception.h Macros.h Types.h Math.h
)
target_link_libraries(karto ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
