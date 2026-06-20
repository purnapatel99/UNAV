cmake_minimum_required(VERSION 3.8)
project(unav_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(unav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mpc_controller
  src/mpc_solver.cpp
  src/mpc_controller_node.cpp
)

target_include_directories(mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(mpc_controller
  rclcpp
  px4_msgs
  unav_msgs
)

install(TARGETS mpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
