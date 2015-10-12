cmake_minimum_required(VERSION 2.8.3)
project(nostop_gazebo_simulator)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  base_math
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
    DEPENDS
      gazebo_ros
      roscpp
      base_math
    CATKIN_DEPENDS
    INCLUDE_DIRS include/${PROJECT_NAME}/
    LIBRARIES ${PROJECT_NAME}
)

set(Gazebo_SOURCES
	src/GazeboDriver.cpp
	src/GazeboSubscriber.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED ${Gazebo_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} base_math lemon)

add_executable(${PROJECT_NAME}_node ${Gazebo_SOURCES} src/GazeboSimulator.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} base_math lemon)
