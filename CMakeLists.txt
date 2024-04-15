cmake_minimum_required(VERSION 3.8)
project(studierbot_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include/)
include_directories(include/interface/)


add_executable(studierbot_drive_node src/studierbot_drive_node.cpp
                                     src/interface/SocketCAN.cpp
                                     src/interface/SocketCANObserver.cpp
                                     src/MotorControllerCAN.cpp
                                     )

add_executable(simple_joy2vel_node src/simple_joy2vel_node.cpp)

ament_target_dependencies(studierbot_drive_node rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(simple_joy2vel_node rclcpp geometry_msgs sensor_msgs)




install(TARGETS
  studierbot_drive_node
  simple_joy2vel_node
  DESTINATION lib/${PROJECT_NAME})



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
