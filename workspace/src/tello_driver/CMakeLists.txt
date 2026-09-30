cmake_minimum_required(VERSION 3.8)
project(tello_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tello_msg REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(tello_driver src/tello_driver.cpp)

ament_target_dependencies(tello_driver
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  cv_bridge image_transport tello_msg
)
target_link_libraries(tello_driver ${OpenCV_LIBS})

add_executable(flight_node src/flight_node.cpp)

ament_target_dependencies(flight_node
  rclcpp std_msgs tello_msg
)

add_executable(teleop_node src/teleop_node.cpp)

ament_target_dependencies(teleop_node
  rclcpp std_msgs geometry_msgs tello_msg
)

install(TARGETS tello_driver flight_node teleop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz  DESTINATION share/${PROJECT_NAME})

ament_package()
