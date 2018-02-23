cmake_minimum_required(VERSION 2.8.3) 
project(dm245m_driver)

find_path(LIBMODBUS_INCLUDE_DIRS
          NAMES modbus.h
          HINTS /usr/include/modbus /usr/local/include/modbus)

find_library(LIBMODBUS_LIBRARIES
             NAMES libmodbus.so)

if (LIBMODBUS_LIBRARIES)
  message( STATUS "Libmodbus found")
else()
  message( FATAL_ERROR "Libmodbus Not found")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(
  INCLUDE_DIRS include src
  ${BOOST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_executable(dm245m_demo_node src/dm245m_demo_node.cpp  src/DM245M.cpp src/DM245MDemo.cpp)
target_link_libraries(dm245m_demo_node ${catkin_LIBRARIES} ${LIBMODBUS_LIBRARIES})

