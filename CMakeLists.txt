cmake_minimum_required(VERSION 3.8)
project(stereoVO)

set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O0 -pthread" )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS})

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
        )

add_library(${PROJECT_NAME} SHARED
        src/ORBextractor.cpp
        src/config.cpp
        src/Frame.cpp
        src/g2o_types.cpp
        src/map.cpp
        src/mappoint.cpp
        src/camera.cpp
        src/visual_odometry.cpp
        )
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS})

add_executable(stereo Examples/stereo_vo.cpp)
target_link_libraries(stereo ${PROJECT_NAME})

#[[
set(SOURCE_FILES main.cpp)
add_executable(VO ${SOURCE_FILES})
target_link_libraries(VO ${PROJECT_NAME})]]
