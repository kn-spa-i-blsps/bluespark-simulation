cmake_minimum_required(VERSION 3.16)
project(underwater_camera_proc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gazebo Harmonic versions (from the ROS 2 Jazzy vendor packages).
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(yaml-cpp REQUIRED)

add_executable(underwater_camera_proc src/main.cpp)
target_link_libraries(underwater_camera_proc
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  opencv_core opencv_imgproc
  yaml-cpp
)

# Binary goes to workspace/bin (git-ignored).
set_target_properties(underwater_camera_proc PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../bin"
)
