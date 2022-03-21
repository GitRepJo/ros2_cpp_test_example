cmake_minimum_required(VERSION 3.5)
project(ros2_cpp_test_example)

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

add_executable(sub_pub src/sub_pub.cpp)
ament_target_dependencies(sub_pub rclcpp std_msgs)

install(TARGETS
  sub_pub
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  test/expected_data.yaml test/input_data.yaml test/sub_pub_launch_testing.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
   find_package(launch_testing_ament_cmake)
  add_launch_test(test/sub_pub_launch_testing.py)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()