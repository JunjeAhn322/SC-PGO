cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

# Use C++17 standard and set release flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Find ROS2 and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# ascanRegistration executable
add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  tf2_ros
  tf2_msgs
  image_transport
  cv_bridge
)
target_link_libraries(ascanRegistration ${PCL_LIBRARIES})

# alaserOdometry executable
add_executable(alaserOdometry src/laserOdometry.cpp)
ament_target_dependencies(alaserOdometry 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  tf2_ros
  tf2_msgs
  image_transport
  cv_bridge
)
target_link_libraries(alaserOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# alaserMapping executable
add_executable(alaserMapping src/laserMapping.cpp)
ament_target_dependencies(alaserMapping 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  tf2_ros
  tf2_msgs
  image_transport
  cv_bridge
)
target_link_libraries(alaserMapping ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# alaserPGO executable
add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
target_compile_options(alaserPGO PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(alaserPGO 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  tf2_ros
  tf2_msgs
  image_transport
  cv_bridge
)
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam
)

# kittiHelper executable
add_executable(kittiHelper src/kittiHelper.cpp)
ament_target_dependencies(kittiHelper 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  tf2_ros
  tf2_msgs
  image_transport
  cv_bridge
)
target_link_libraries(kittiHelper ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Install targets
install(TARGETS
  ascanRegistration
  alaserOdometry
  alaserMapping
  alaserPGO
  kittiHelper
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally install launch files if you have them in a "launch" directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
