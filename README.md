# Ros2 Cpp test example

This is an example to test a subscriber/publisher node (dut device under test) with the launch_test command.    
A test node is set up and publishes a message to the dut that is defined in a .yaml file.    
The test node then receives the answer and compares it to the expected data, which is also defined in a .yaml file.    
For more information see https://github.com/ros2/launch_ros.

## Create a cpp package with a ros2 publisher/subscriber node.

Create a cpp package in your working directory:

```
cd /home/$USER/dev_ws/src    

ros2 pkg create --build-type ament_cmake ros2_cpp_test_example`
```
Create a file inside the package src directory:   

```
cd /home/$USER/dev_ws/src/ros2_cpp_test_example/src  && touch sub_pub.cpp
```

Add the following text to the sub_pub.cpp file to create a node:

```
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberPublisher : public rclcpp::Node
{
  public:
    SubscriberPublisher()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic", 10, std::bind(&SubscriberPublisher::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::String>(
      "output_topic",10);
    
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      if (msg->data == "Hello World"){
        auto message_out = std_msgs::msg::String();
        message_out.data = "Greetings";
        publisher_->publish(message_out);
      }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

In package.xml add the description, maintainer and license information and the following dependencies:
```
<depend>rclcpp</depend>    
<depend>std_msgs</depend>
```
In CMakeList.txt add 
```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub_sub src/pub_sub.cpp)
ament_target_dependencies(pub_sub rclcpp std_msgs)
 
install(TARGETS
  pub_sub
  DESTINATION lib/${PROJECT_NAME})
```
## Create the launch test
Create a test directory, create a launchfile as well as a yaml file.    
The content of the .yaml file will be used to test the node under test (dut -> device under test).     
The launch file will launch the tests defined in the file.     
```
cd /home/$USER/dev_ws/src/ros2_cpp_test_example && mkdir test    

cd test && touch sub_pub_launch_testing.py input_data.yaml expected_data.yaml
```
Add the yaml files and the test launch file to the install path by adding following lines to CMakeList.txt:

```
install(FILES
  test/expected_data.yaml test/input_data.yaml test/sub_pub_launch_testing.py
  DESTINATION lib/${PROJECT_NAME})
```

Open the sub_pub_launch_testing.py file and add the following code:

```

from lib2to3.pgen2.token import EQUAL
import time
import unittest
import inspect
import yaml

import os
import ament_index_python

import rclpy
from rclpy.node import Node
import std_msgs.msg
from std_msgs.msg import String

import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.util

import pytest

@pytest.mark.launch_test
def generate_test_description():
    """ Specifiy nodes or processes to launch for test
        :param 
        :return dut [ros2 node] node to be tested (device under test)
        :return ...,... specifications for launch_testing
    Multiple nodes that are to be tested can be launched"""     
    
    # dut -> device under test is the node to be tested in this example
    dut = Node(
        package='ros2_cpp_test_example',
        executable='sub_pub',
        name='sub_pub',
    )
    context = {'dut': dut  }

    return (launch.LaunchDescription([   
        dut,
        launch_testing.actions.ReadyToTest()]
        ) , context
    )
    
class TestProcessOutput(unittest.TestCase):
    """Details to use this class in the context of launch_testing:
        nodes: https://github.com/ros2/launch_ros
        process: https://github.com/ros2/launch/tree/master/launch_testing"""

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()
     
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('input_output_node')

    def tearDown(self):
        self.node.destroy_node()
    

    def timer_callback(self):
        """ Reads a file and publish the data from this file to ros2
                :param -
                :return - 
            """    
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('ros2_cpp_test_example'),
        'lib/ros2_cpp_test_example',
        'input_data.yaml'
        )
        
        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)
        
        msg = String()
        msg.data = data['msg']['data']
        self.publisher_.publish(msg)
        self.node.get_logger().info('Publishing: "%s"' % msg.data)


    def test_dut_output(self, dut, proc_output):
        """ Listen for a message published by dut and compare message to expected value
                :param 
                :return dut [ros2 node] node to be tested (device under test)
                :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
                :return - 
            """     
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function

        # Publish data to dut
        self.publisher_ = self.node.create_publisher(String, 'input_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('ros2_cpp_test_example'),
        'lib/ros2_cpp_test_example',
        'expected_data.yaml'
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        expected_data = data['msg']['data']
        
        # Setup for listening to dut messages
        received_data =  []
        sub = self.node.create_subscription(
            String,
            'output_topic',
            lambda msg: received_data.append(str(msg.data)),
            10
        )
        
        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        
            if received_data == []:
                test_data = ""
                
            else:

                print (f"\n[{function_name}] [INFO] expected_data:\n"+ str(expected_data))
                print (f"\n[{function_name}] [INFO] received_data:\n"+ str(received_data[0]))
                test_data = received_data[0]
            
            # test actual output for expected output
            self.assertEqual(str(test_data),expected_data)
             
        finally:
            self.node.destroy_subscription(sub) 
```

You can now run the launch test for the dut:

```
cd /home/$USER/dev_ws/src/ros2_cpp_test_example      

launch_test test/sub_pub_launch_testing.py 
```

## Use colcon test to start testing

Add the following lines to your CMakeList.txt inside "if(BUILD_TESTING)":  

```
find_package(launch_testing_ament_cmake)    
add_launch_test(test/sub_pub_launch_testing.py)
```
You can now run the launch test by using:        
```
colcon test --packages-select ros2_cpp_test_example`
```
in your development workspace.

## Add ros2 to visual studio code (if used)

Also see https://www.allisonthackston.com/articles/vscode-docker-ros2.html    

Add required ros2 dependencies to enable intellisense    
Create a file c_cpp_properties in a new .vscode directory in the base directory of the package    
   
```
cd /home/$USER/dev_ws/src/ros2_cpp_test_example    

mkdir .vscode && cd .vscode    

touch c_cpp_properties.json 
```
Add the following text to it
```
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/foxy/include"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "clang-x64"
        }
    ],
    "version": 4
}
```
To use this file, open VSCode inside the base directory of your package:    
```
/home/$USER/dev_ws/src/ros2_cpp_test_example code .
```
## Make the colcon command available to vscode (if used)
   

Create a file tasks.json 
```
touch tasks.json
``` 
Add the following code to tasks.json

Choose the package you want to build and use the name with the option --packages-select. 
Only the chosen packages will be built.
```
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "group": "build",
            "type": "shell",
            "command": "cd /home/$USER/dev_ws && colcon build --packages-select ros2_cpp_test_example --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
        },
        {
            "label": "test",
            "group": "test",
            "type": "shell",
            "command": "cd /home/$USER/dev_ws && colcon test --packages-select ros2_cpp_test_example"
        }
    ]
}
```
You can now run the build task inside VSCode    
To use this file, open VSCode inside the base directory of your package      
```
/home/$USER/dev_ws/src/ros2_cpp_test_example code .
```
