# Build description of cbf_safety_filter:
#   - cbf_safety_filter_core: plain C++ library (CBF + EKF drone model), no ROS.
#     Uses the HalfPlane type from pcd_constraints.
#   - cbf_parameters: generated ParamListener from src/cbf_params.yaml
#   - executable cbf_app (loads the map obstacles with the clustering library)
# Every new dependency needs a find_package() here AND a <depend> in package.xml.
cmake_minimum_required(VERSION 3.10)
project(cbf_safety_filter)

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
find_package(geometry_msgs REQUIRED)
find_package(collision_avoidance_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(qpmad REQUIRED)
find_package(pcd_constraints REQUIRED)
find_package(clustering REQUIRED)  # MapObstacle / LidarObstacle and loadMapObstacles (node only, not the core)

# ---- Algorithm library: plain C++, no ROS ----
add_library(${PROJECT_NAME}_core
  src/cbf_filter.cpp
  src/ekf/drone_model.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen qpmad::qpmad pcd_constraints::pcd_constraints_core)

# ---- Parameters (generated ParamListener) ----
generate_parameter_library(cbf_parameters src/cbf_params.yaml)

# ---- Node: CBF safety filter ----
add_executable(cbf_app
  app/cbf_app.cpp
  src/nodes/cbf_node.cpp
)
target_link_libraries(cbf_app ${PROJECT_NAME}_core cbf_parameters)
ament_target_dependencies(cbf_app
  rclcpp ament_index_cpp sensor_msgs nav_msgs geometry_msgs collision_avoidance_msgs qpmad clustering)

install(TARGETS cbf_app DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
