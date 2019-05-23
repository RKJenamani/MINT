cmake_minimum_required(VERSION 2.8.3)

project(MINT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)

find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})

find_package(aikido 0.0.2 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kunzretimer
    robot
    rviz
    statespace
)
include_directories(${aikido_INCLUDE_DIRS})

find_package(libherb REQUIRED)
include_directories(${libherb_INCLUDE_DIRS})

find_package(pr_assets REQUIRED)
include_directories(${pr_assets_INCLUDE_DIRS})

find_package(pr_tsr REQUIRED)
include_directories(${pr_tsr_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS program_options graph)
include_directories(${Boost_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

find_package(magi REQUIRED)

include_directories(include)

#add_library(${PROJECT_NAME}
#  src/LPAStar.cpp
#  src/PRLStackPlanner.cpp)

add_executable(LazySP src/LazySP.cpp)
target_link_libraries(LazySP
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${EIGEN_LIBRARIES}
  libherb
  magi)
  
install(TARGETS LazySP RUNTIME DESTINATION bin)



