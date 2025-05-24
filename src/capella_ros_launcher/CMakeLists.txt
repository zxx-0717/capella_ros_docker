cmake_minimum_required(VERSION 3.5)
project(capella_ros_launcher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(teb_local_planner REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rviz2 REQUIRED)

pkg_check_modules(Eigen REQUIRED eigen3)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
include_directories(
  ${Eigen_INCLUDE_DIRS}
)

#do_calib node
add_executable(dsf_base_node src/dsf_base.cpp src/dsf_base_node.cpp)
ament_target_dependencies(dsf_base_node 
                          rclcpp
                          capella_ros_msg
                          tf2_ros
                          geometry_msgs
                          nav_msgs
                          angles
                          tf2_geometry_msgs)

target_link_libraries(dsf_base_node
                      ${colcon_LIBRARIES}
)

install(TARGETS
        dsf_base_node
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_odom_node src/test_odom.cpp)
ament_target_dependencies(test_odom_node 
                          rclcpp
                          nav_msgs
                          tf2_geometry_msgs
                          capella_ros_msg
                          tf2_ros)

target_link_libraries(test_odom_node
                      ${colcon_LIBRARIES}
)

install(TARGETS
        test_odom_node
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(auto_clear_costmap_node src/auto_clear_costmap_node.cpp)

ament_target_dependencies(auto_clear_costmap_node rclcpp nav2_msgs)

install(TARGETS
  auto_clear_costmap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch param rviz map urdf config
  DESTINATION share/${PROJECT_NAME})

ament_package()
