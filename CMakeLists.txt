cmake_minimum_required(VERSION 2.8.3)
project(lodestar_odometry)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_BUILD_TYPE release)

find_package(PCL  REQUIRED)
set(_include_dirs "include;/usr/include;/usr/include/opencv4")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    genmsg
    tf_conversions
    cv_bridge
    image_transport
    )
 find_package(OpenCV 4)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3 ")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED NO_MODULE)


catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3 
    CATKIN_DEPENDS roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs
    LIBRARIES ${PROJECT_NAME}
    )



include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

 set (${PROJECT_NAME}_LIB_SRCS
   src/${PROJECT_NAME}/utils.cpp
   src/${PROJECT_NAME}/pointnormal.cpp
   src/${PROJECT_NAME}/registration.cpp
   src/${PROJECT_NAME}/n_scan_normal.cpp
   src/${PROJECT_NAME}/odometrykeyframefuser.cpp
   src/${PROJECT_NAME}/eval_trajectory.cpp
   src/${PROJECT_NAME}/statistics.cpp
   src/${PROJECT_NAME}/lodestar.cpp
 )

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${CERES_LIBRARIES})

add_executable(lodestar_odom src/lodestar_odom.cpp)
target_link_libraries(lodestar_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})
