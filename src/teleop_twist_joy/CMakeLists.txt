cmake_minimum_required(VERSION 3.5)
project(teleop_twist_joy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(edna_interfaces REQUIRED)


add_library(
  ${PROJECT_NAME} 
  SHARED 
  src/teleop_twist_joy.cpp)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}")

ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "nav_msgs"
  "edna_interfaces"

)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "teleop_twist_joy::TeleopTwistJoy")

add_executable(${PROJECT_NAME}_node src/teleop_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()
