cmake_minimum_required(VERSION 2.8.3)
project(MINT)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  DEPENDS Boost
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIR})

#link_directories(
#  ${catkin_LIBRARY_DIRS}
#)

#link_libraries(
#  ${catkin_LIBRARIES}
#  ${ompl_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${DART_LIBRARIES}
#)

#add_library(${PROJECT_NAME}
#  src/MINT.cpp
#  src/utils/BisectPerm.cpp
#)

#install(TARGETS
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# For example scripts
# Generic 2D example scipt. API - 1
add_executable(example2DMINT_image examples/example2DMINT_image.cpp)
target_link_libraries(example2DMINT_image 
  ${OpenCV_LIBS}  
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
  )

install(TARGETS example2DMINT_image RUNTIME DESTINATION bin)

# Generic 2D example scipt. API - 2
#add_executable(example2DMINT examples/example2DMINT.cpp)
#target_link_libraries(example2DMINT 
#  ${OpenCV_LIBS}
#  ${Boost_LIBRARIES}
#  ${DART_LIBRARIES}
#  ${OMPL_LIBRARIES}
#  )
#install(TARGETS example2DMINT RUNTIME DESTINATION bin)

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
#FILE(GLOB_RECURSE LibFiles "include/*.hpp")
#add_custom_target(headers SOURCES ${LibFiles})
