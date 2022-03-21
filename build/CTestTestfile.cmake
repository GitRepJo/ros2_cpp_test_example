# CMake generated Testfile for 
# Source directory: /home/jo/dev_ws/src/ros2_cpp_test_example
# Build directory: /home/jo/dev_ws/src/ros2_cpp_test_example/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_sub_pub_launch_testing.py "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/jo/dev_ws/src/ros2_cpp_test_example/build/test_results/ros2_cpp_test_example/test_sub_pub_launch_testing.py.xunit.xml" "--package-name" "ros2_cpp_test_example" "--output-file" "/home/jo/dev_ws/src/ros2_cpp_test_example/build/launch_test/CHANGEME.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/jo/dev_ws/src/ros2_cpp_test_example/test/sub_pub_launch_testing.py" "--junit-xml=/home/jo/dev_ws/src/ros2_cpp_test_example/build/test_results/ros2_cpp_test_example/test_sub_pub_launch_testing.py.xunit.xml" "--package-name=ros2_cpp_test_example")
set_tests_properties(test_sub_pub_launch_testing.py PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/jo/dev_ws/src/ros2_cpp_test_example/build" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;125;ament_add_test;/home/jo/dev_ws/src/ros2_cpp_test_example/CMakeLists.txt;36;add_launch_test;/home/jo/dev_ws/src/ros2_cpp_test_example/CMakeLists.txt;0;")
