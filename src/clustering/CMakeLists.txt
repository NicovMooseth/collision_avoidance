cmake_minimum_required(VERSION 3.10)
project(clustering)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pcd_constraints REQUIRED)
find_package(map_obstacles REQUIRED)

# ---- Algorithm library: plain C++, no ROS (exported) ----
add_library(${PROJECT_NAME}_core src/clustering.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen pcd_constraints::pcd_constraints_core map_obstacles::map_obstacles_core)

# ---- Parameters (generated ParamListeners) ----
generate_parameter_library(${PROJECT_NAME}_parameters src/clustering_params.yaml)
generate_parameter_library(map_parameters src/map_params.yaml)

# ---- Node: clustering ----
add_executable(clustering_app
  app/clustering_app.cpp
  src/nodes/clustering_node.cpp
)
target_include_directories(clustering_app PRIVATE include)
target_link_libraries(clustering_app ${PROJECT_NAME}_core ${PROJECT_NAME}_parameters map_parameters)
ament_target_dependencies(clustering_app rclcpp ament_index_cpp sensor_msgs nav_msgs)

# ---- Install ----
# only the library headers (include/clustering/); include/nodes/ belongs to the node and stays private
install(DIRECTORY include/clustering/ DESTINATION include/clustering)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS clustering_app DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# lets other packages do find_package(clustering) + link clustering::clustering_core
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pcd_constraints map_obstacles)

ament_package()
