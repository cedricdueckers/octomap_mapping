cmake_minimum_required(VERSION 3.5)
project(octomap_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OCTOMAP REQUIRED)

find_package(Boost REQUIRED COMPONENTS date_time)

find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_auto_add_library(octomap_server_static SHARED
  src/octomap_server_static.cpp
)

target_link_libraries(octomap_server_static
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(octomap_server_static
  PUBLIC
    ${OCTOMAP_INCLUDE_DIRS}
)

rclcpp_components_register_node(octomap_server_static
  PLUGIN "octomap_server::OctomapServerStatic"
  EXECUTABLE octomap_server_static_node
)

ament_auto_add_library(octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(octomap_server
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server_node
)

ament_auto_add_library(octomap_server_multilayer SHARED
  src/octomap_server.cpp
  src/octomap_server_multilayer.cpp
)

target_link_libraries(octomap_server_multilayer
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(octomap_server_multilayer
  PLUGIN "octomap_server::OctomapServerMultilayer"
  EXECUTABLE octomap_server_multilayer_node
)

ament_auto_add_library(tracking_octomap_server SHARED
  src/octomap_server.cpp
  src/tracking_octomap_server.cpp
)

target_link_libraries(tracking_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(tracking_octomap_server
  PLUGIN "octomap_server::TrackingOctomapServer"
  EXECUTABLE tracking_octomap_server_node
)

ament_auto_add_library(color_octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(color_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(color_octomap_server
  PLUGIN "octomap_server::ColorOctomapServer"
  EXECUTABLE color_octomap_server_node
)

target_compile_definitions(color_octomap_server PUBLIC COLOR_OCTOMAP_SERVER)

ament_auto_add_library(octomap_saver SHARED
  src/octomap_saver.cpp
)

target_link_libraries(octomap_saver
  ${OCTOMAP_LIBRARIES}
)

rclcpp_components_register_node(octomap_saver
  PLUGIN "octomap_server::OctomapSaver"
  EXECUTABLE octomap_saver_node
)


ament_auto_add_library(octomap_to_ply SHARED
  src/octomap_server.cpp
  src/octomap_to_ply.cpp
)

target_link_libraries(octomap_to_ply
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(octomap_to_ply
  PLUGIN "octomap_server::OctomapToPly"
  EXECUTABLE octomap_to_ply_node
)

rclcpp_components_register_nodes(octomap_to_ply "octomap_server::OctomapToPly")

ament_auto_add_library(RgbOctomapNode SHARED
  src/RgbOctomapNode.cpp
)

target_link_libraries(RgbOctomapNode
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(RgbOctomapNode
  PUBLIC
    ${OCTOMAP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

rclcpp_components_register_node(RgbOctomapNode
  PLUGIN "octomap_server::RgbOctomapNode"
  EXECUTABLE rgb_octomap_node
)

add_executable(rgb_octomap_node_exec src/rgb_octomap_node_main.cpp)
target_link_libraries(rgb_octomap_node_exec RgbOctomapNode ${rclcpp_LIBRARIES})
ament_target_dependencies(rgb_octomap_node_exec rclcpp sensor_msgs octomap_msgs)
install(TARGETS rgb_octomap_node_exec DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)

install(PROGRAMS
  scripts/octomap_eraser_cli.py
  DESTINATION lib/${PROJECT_NAME}
)

