cmake_minimum_required(VERSION 3.8)
project(hand_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui calib3d)

# libmediapipe.so dari instalasi pip mediapipe
set(MEDIAPIPE_SO
  "/home/arilix/.local/lib/python3.10/site-packages/mediapipe/tasks/c/libmediapipe.so")
if(NOT EXISTS "${MEDIAPIPE_SO}")
  message(FATAL_ERROR
    "libmediapipe.so tidak ditemukan: ${MEDIAPIPE_SO}\n"
    "Install: pip install mediapipe 'numpy<2'")
endif()
message(STATUS "MediaPipe C lib: ${MEDIAPIPE_SO}")

# C++ executable
add_executable(hand_control_node
  src/hand_control_node.cpp
)

target_include_directories(hand_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(hand_control_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge
)

target_link_libraries(hand_control_node
  ${OpenCV_LIBS}
  "${MEDIAPIPE_SO}"
)

set_target_properties(hand_control_node PROPERTIES
  BUILD_RPATH  "/home/arilix/.local/lib/python3.10/site-packages/mediapipe/tasks/c"
  INSTALL_RPATH "/home/arilix/.local/lib/python3.10/site-packages/mediapipe/tasks/c"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS hand_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
