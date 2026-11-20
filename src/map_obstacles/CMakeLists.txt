# Build description of map_obstacles:
#   - map_obstacles_core: plain C++ library (MapObstacle type, CSV loading, closed-shape check), no ROS.
#     Exported, so other packages (cbf_safety_filter, clustering) can load the offline map.
#   - maps/: the offline map CSV files, installed to share/map_obstacles/maps (the nodes resolve relative map paths
#     against share/map_obstacles)
# Every new dependency needs a find_package() here AND a <depend> in package.xml.
cmake_minimum_required(VERSION 3.10)
project(map_obstacles)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcd_constraints REQUIRED)

# ---- Algorithm library: plain C++, no ROS (exported) ----
add_library(${PROJECT_NAME}_core src/map_obstacles.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen pcd_constraints::pcd_constraints_core)

# ---- Install ----
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

# lets other packages do find_package(map_obstacles) + link map_obstacles::map_obstacles_core
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pcd_constraints)

ament_package()
