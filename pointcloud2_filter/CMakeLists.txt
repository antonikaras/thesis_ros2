cmake_minimum_required(VERSION 3.5)
project(pointcloud2_filter)

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
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# pcl_ros2_test
add_executable(pcl_ros2_test src/pcl_ros2_test.cpp)
ament_target_dependencies(pcl_ros2_test rclcpp std_msgs pcl_conversions)
target_link_libraries(pcl_ros2_test ${PCL_LIBRARIES})

# pcl_filter
add_executable(pcl_filter src/pcl_filter.cpp)
#add_library(pcl_filter SHARED src/pcl_filter.cpp)
ament_target_dependencies(pcl_filter rclcpp tf2_msgs std_msgs pcl_conversions
tf2_geometry_msgs tf2_msgs)
target_link_libraries(pcl_filter ${PCL_LIBRARIES})

install(TARGETS
  pcl_ros2_test
  pcl_filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)


ament_package()
