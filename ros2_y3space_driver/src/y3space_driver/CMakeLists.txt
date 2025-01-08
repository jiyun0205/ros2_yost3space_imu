cmake_minimum_required(VERSION 3.16)
project(y3space_driver)

# if(NOT CMAKE_CXX_STANDARD)
#     set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(__GLIBC_USE=1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rcl_interfaces REQUIRED)

# Include directories
include_directories(
  include
  include/y3space_driver
  include/serial
  include/serial/impl
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
)

# Add the serial source files as a library
add_library(serial STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../serial/impl/unix.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/../serial/serial.cc
  )

# Create the serial_interface library
add_library(serial_interface
            ${CMAKE_CURRENT_SOURCE_DIR}/src/SerialInterface.cpp)

# Create the main executable
add_executable(y3space_driver
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Y3SpaceDriver.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

target_link_libraries(serial_interface
  serial)

# Link dependencies to the executable
ament_target_dependencies(y3space_driver
  rclcpp
  std_msgs
  sensor_msgs
)

# Link the libraries, including serial and serial_interface
target_link_libraries(y3space_driver
  serial 
  serial_interface 
  ${Boost_LIBRARIES}
)

# Install the targets
install(TARGETS y3space_driver serial_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
