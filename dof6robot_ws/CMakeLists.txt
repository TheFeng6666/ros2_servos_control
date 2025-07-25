cmake_minimum_required(VERSION 3.16)
project(dof6robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#find libsub
include(FindPkgConfig)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

if(LIBUSB1_FOUND)
  include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
else()
  message( FATAL_ERROR "Failed to find libusb-1.0" )
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/dof6robot_hardware_interface.cpp
)

# include directories and link libraries
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/fashionstar-uart-servo-cpp-sdk/include>
)

# 添加库搜索路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/fashionstar-uart-servo-cpp-sdk/build/lib)

# 正确链接库
target_link_libraries(${PROJECT_NAME}
  fsuartservo
  ${LIBUSB1_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface dof6robot_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY hardware/include/ fashionstar-uart-servo-cpp-sdk/include/
  DESTINATION include
)


install(
  DIRECTORY description/urdf description/ros2_control description/description
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
