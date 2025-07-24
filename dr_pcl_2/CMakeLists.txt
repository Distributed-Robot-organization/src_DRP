cmake_minimum_required(VERSION 3.8)
project(dr_pcl_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(obstacles_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_msgs REQUIRED)
#find_package(pcl_transforms REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs)
find_package(pcl_ros REQUIRED)
# find_package(interfaces_pkg REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_filter_node
   src/pcl_filter.cpp)

ament_target_dependencies(pcl_filter_node rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)


install(TARGETS
  pcl_filter_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
