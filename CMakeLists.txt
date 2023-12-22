cmake_minimum_required(VERSION 3.8)
project(fdilink_ahrs)

if(CMAKE_BUILD_TYPE
   STREQUAL
   "Release"
)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -fPIC")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++17 -fPIC")
endif()

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${Eigen3_INCLUDE_DIRS})

## crc_table
ament_auto_add_library(crc_table src/crc_table.cpp)

ament_auto_add_executable(ahrs_driver src/ahrs_driver.cpp)
target_link_libraries(ahrs_driver crc_table)

# ament_auto_add_executable(imu_tf src/imu_tf.cpp)

ament_auto_package(INSTALL_TO_SHARE launch)
