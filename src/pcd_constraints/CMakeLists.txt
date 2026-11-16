# Build description of pcd_constraints:
#   - pcd_constraints_core: plain C++ library (HalfPlane type + clustering algorithms), no ROS.
#     Exported, so other packages (cbf_safety_filter) can use the HalfPlane type.
#   - scan_to_constraint_parameters: generated ParamListener from src/scan_to_constraint_params.yaml
#   - executable scan_to_constraint_app
# Every new dependency needs a find_package() here AND a <depend> in package.xml.
cmake_minimum_required(VERSION 3.10)
project(pcd_constraints)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(collision_avoidance_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- Algorithm library: plain C++, no ROS (exported) ----
add_library(${PROJECT_NAME}_core
  src/clustering/halfplane_algorithm.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)

# ---- Parameters (generated ParamListener) ----
generate_parameter_library(scan_to_constraint_parameters src/scan_to_constraint_params.yaml)

# ---- Node: scan -> constraints ----
add_executable(scan_to_constraint_app
  app/scan_to_constraint_app.cpp
  src/nodes/scan_to_constraint_node.cpp
)
target_link_libraries(scan_to_constraint_app ${PROJECT_NAME}_core scan_to_constraint_parameters)
ament_target_dependencies(scan_to_constraint_app
  rclcpp sensor_msgs nav_msgs geometry_msgs shape_msgs collision_avoidance_msgs tf2_ros)

# ---- Install ----
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS scan_to_constraint_app DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# lets other packages do find_package(pcd_constraints) + link pcd_constraints::pcd_constraints_core
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
