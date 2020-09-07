cmake_minimum_required(VERSION 3.0)
project(Path_Planning)

set (CMAKE_CXX_STANDARD 14)

set( CMAKE_BUILD_TYPE "Release" )
#set(CMAKE_BUILD_TYPE "Debug")

# for tx2
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
# for x86
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -march=native")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(catkin_DIR /opt/ros/kinetic/)
find_package(catkin REQUIRED COMPONENTS
        mavros
        mavros_msgs
        roscpp
        sensor_msgs
        geometry_msgs
        std_msgs
        octomap_msgs
        octomap_rviz_plugins)

## for python binding
#find_package(Boost COMPONENTS)
#find_package(PythonInterp REQUIRED)
#find_package(PythonLibs REQUIRED)

# PCL related
include_directories()
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
#        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/geometry/include
        ${PROJECT_SOURCE_DIR}/rrt_star/include
        ${PROJECT_SOURCE_DIR}/pointcloud/include
        ${PROJECT_SOURCE_DIR}/control/include
        ${PROJECT_SOURCE_DIR}/helper/
        ${PROJECT_SOURCE_DIR}/matplotlib-cpp
)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

# for matplotlib-cpp
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)


add_library(geometry
        ${PROJECT_SOURCE_DIR}/geometry/src/cuboid.cpp
        ${PROJECT_SOURCE_DIR}/geometry/src/bresenham.cpp)

add_library(pcl_func
        ${PROJECT_SOURCE_DIR}/pointcloud/src/clustering.cpp
        ${PROJECT_SOURCE_DIR}/pointcloud/src/downsample.cpp
        ${PROJECT_SOURCE_DIR}/pointcloud/src/pcl_helper.cpp)

add_library(rrt_star
        ${PROJECT_SOURCE_DIR}/rrt_star/src/rrt_star.cpp)

add_library(control
        ${PROJECT_SOURCE_DIR}/control/src/control.cpp)

add_executable(rrt_test rrt_test.cpp)
target_link_libraries(rrt_test
        ${THIRD_PARTY_LIBS}
        ${PYTHON_LIBRARIES}
        control
        rrt_star
        geometry
        pcl_func
        )


