cmake_minimum_required (VERSION 2.8.11)
project (CeresSLAM)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest)
find_package(Ceres REQUIRED)

include_directories(include/unicycle_landmarks
    include/utils
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

#add_library(utils STATIC
#)

# One dimension 
add_library(one_dimension STATIC
    src/one_dimension/one_dimension_sim.cpp
)
target_include_directories(one_dimension PUBLIC
  ${CERES_INCLUDE_DIRS}
    include)
  target_link_libraries(one_dimension ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Build Examples
include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_executable(one_dimension_example
    src/examples/one_dimension.cpp
    )
target_link_libraries(one_dimension_example one_dimension ${YAML_CPP_LIBRARIES})

add_executable(online_slam_1d
    src/examples/online_slam_1d.cpp
    )
target_link_libraries(online_slam_1d one_dimension ${YAML_CPP_LIBRARIES})

# Unicycle
add_library(unicycle_landmarks STATIC
    src/unicycle_sim/unicycle_sim.cpp
)
target_include_directories(unicycle_landmarks PUBLIC
    include)
target_link_libraries(unicycle_landmarks ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Build Examples
include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_executable(unicycle_full_slam
    src/unicycle_sim/full_slam.cpp
    )
target_link_libraries(unicycle_full_slam unicycle_landmarks ${YAML_CPP_LIBRARIES})

