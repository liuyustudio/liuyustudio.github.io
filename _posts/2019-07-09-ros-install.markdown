---
layout: post
title:  "Ubuntu 16.04 ROS Install"
date:   2019-07-09 11:51:16 +0800
categories: ROS Ubuntu Install
---

# ROS

Robot Operating System Web Site: [http://www.ros.org](http://www.ros.org)

<!-- TOC -->

- [1. environment](#1-environment)
    - [1.1. install](#11-install)
        - [1.1.1. 操作系统、网络、代理安装、配置](#111-操作系统网络代理安装配置)
        - [1.1.2. ROS 安装](#112-ros-安装)
- [2. Tutorials](#2-tutorials)
    - [2.1. Installing and Configuring ROS Environment](#21-installing-and-configuring-ros-environment)
    - [2.2. Navigating the ROS Filesystem](#22-navigating-the-ros-filesystem)
    - [2.3. Creating a ROS Package](#23-creating-a-ros-package)
    - [2.4. Building a ROS Package](#24-building-a-ros-package)
    - [2.5. Understanding ROS Nodes](#25-understanding-ros-nodes)
    - [2.6. Understanding ROS Topics](#26-understanding-ros-topics)
    - [2.7. Understanding ROS Services and Parameters](#27-understanding-ros-services-and-parameters)
    - [2.8. Using rqt_console and roslaunch](#28-using-rqt_console-and-roslaunch)
    - [2.9. Using rosed to edit files in ROS](#29-using-rosed-to-edit-files-in-ros)
    - [2.10. Creating a ROS msg a](#210-creating-a-ros-msg-a)
    - [2.11. Writing a Simple Publisher and Subscriber (C++)](#211-writing-a-simple-publisher-and-subscriber-c)
    - [2.12. Writing a Simple Publisher and Subscriber (Python)](#212-writing-a-simple-publisher-and-subscriber-python)
    - [2.13. Examining the Simple Publisher and Subscriber](#213-examining-the-simple-publisher-and-subscriber)
    - [2.14. Writing a Simple Service and Client (C++)](#214-writing-a-simple-service-and-client-c)
    - [2.15. Writing a Simple Service and Client (Python)](#215-writing-a-simple-service-and-client-python)
    - [2.16. Examining the Simple Service and Client](#216-examining-the-simple-service-and-client)
    - [2.17. Recording and playing back data](#217-recording-and-playing-back-data)
    - [2.18. Getting started with roswtf](#218-getting-started-with-roswtf)
    - [2.19. Navigating the ROS wiki](#219-navigating-the-ros-wiki)

<!-- /TOC -->

## 1. environment

### 1.1. install

仅以 Ubuntu 为底层操作系统。

ROS 版本 | 官方支持 Ubuntu 版本 | Installation Guide
---|---|---
ROS Kinetic Kame | Ubuntu Wily (15.10) and Ubuntu Xenial (16.04 LTS) | [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)
ROS Melodic Morenia | Ubuntu Artful (17.10), Bionic (18.04 LTS) and Debian Stretch | [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

#### 1.1.1. 操作系统、网络、代理安装、配置

略

#### 1.1.2. ROS 安装

以 Kenetic 版本为例

- install ROS platform

  ```sh
  # setup sources list
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  # setup apt keys
  # available server:
  #   - hkp://keyserver.ubuntu.com:80
  #   - hkp://pgp.mit.edu:80
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

  # install
  #---------------------------------------------------------
  # Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
  #
  #   sudo apt install ros-melodic-desktop-full
  #
  #---------------------------------------------------------
  # Desktop Install: ROS, rqt, rviz, and robot-generic libraries
  #
  #   sudo apt install ros-melodic-desktop
  #
  #---------------------------------------------------------
  # ROS-Base: (Bare Bones) ROS package, build, and communication libraries. No GUI tools.
  #
  #   sudo apt install ros-melodic-ros-base
  #
  #---------------------------------------------------------
  # Individual Package
  #
  #   sudo apt install ros-melodic-PACKAGE
  # e.g.
  #   sudo apt install ros-melodic-slam-gmapping
  #
  # To find available packages, use:
  #
  #   apt search ros-melodic
  #

  sudo apt install ros-melodic-desktop-full
  ```

- Initialize rosdep

  ```sh
  sudo rosdep init
  rosdep update
  ```

- ROS Environment Setup

  ```sh
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

- Dependencies for building packages

  ```sh
  sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
  ```

## 2. Tutorials

### 2.1. Installing and Configuring ROS Environment

[http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

- Install ROS
  略
- Managing Your Environment
  略
- Create a ROS Workspace

  ```sh
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make

  source devel/setup.bash

  echo $ROS_PACKAGE_PATH
  # /home/youruser/catkin_ws/src:/opt/ros/kinetic/share
  ```

### 2.2. Navigating the ROS Filesystem

[http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)

- install ros-tutorials

```sh
# sudo apt-get install ros-<distro>-ros-tutorials
# <distro>: indigo, kinetic, lunar, etc.
sudo apt-get install ros-kinetic-ros-tutorials
```

### 2.3. Creating a ROS Package

[http://wiki.ros.org/ROS/Tutorials/CreatingPackage](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

use the **catkin_create_pkg** script to create a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:

```sh
cd ~/catkin_ws/src

# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

Building a catkin workspace and sourcing the setup file:

```sh
cd ~/catkin_ws

catkin_make

# To add the workspace to ROS environment
source ~/catkin_ws/devel/setup.bash
```

Review **first-order** dependencies via **rospack**:

```sh
rospack depends1 beginner_tutorials
```

list corresponding **package.xml**:

```sh
roscd beginner_tutorials
cat package.xml
```

output:

```xml
<package format="2">
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

List indirect dependencies:

```sh
rospack depends beginner_tutorials
```

### 2.4. Building a ROS Package

[http://wiki.ros.org/ROS/Tutorials/BuildingPackages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

```sh
# import environment settings
# source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
#
# For Kinetic for instance
source /opt/ros/kinetic/setup.bash
```

Note:

- catkin_make combines the calls to cmake and make in the standard CMake workflow:

  ```sh
  # In a CMake project
  mkdir build
  cd build
  cmake ..
  make
  make install  # (optionally)

  #===========================

  # In a catkin workspace
  catkin_make
  catkin_make install  # (optionally)
  ```

- build in a different place:

  ```sh
  # In a catkin workspace
  catkin_make --source my_src
  catkin_make install --source my_src  # (optionally)
  ```

### 2.5. Understanding ROS Nodes

[http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)

- roscore
- rosnode
  - rosnode list
  - rosnode info /rosout
  - rosnode ping my_turtle
- rosrun
  - rosrun turtlesim turtlesim_node
  - rosrun turtlesim turtlesim_node __name:=my_turtle
- rosnode
  - rosnode cleanup

### 2.6. Understanding ROS Topics

[http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

- start roscore
- turtlesim
  - start turtlesim
    rosrun turtlesim turtlesim_node
  - turtle keyboard teleoperation
    rosrun turtlesim turtle_teleop_key
- rostopic

  cmd | desc
  ---|---
  rostopic bw   | display bandwidth used by topic
  rostopic echo | print messages to screen
  rostopic hz   | display publishing rate of topic
  rostopic list | print information about active topics
  rostopic pub  | publish data to topic
  rostopic type | print topic type

  - rostopic echo /turtle1/cmd_vel
  - rostopic list -v
  - rostopic type
    - rostopic type /turtle1/cmd_vel
    - rosmsg show geometry_msgs/Twist
    - rostopic type /turtle1/cmd_vel | rosmsg show
  - rostopic pub
    - rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    - rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
  - rostopic echo /turtle1/pose
  - rostopic hz
    - rostopic hz /turtle1/pose
  - rosrun rqt_plot rqt_plot

### 2.7. Understanding ROS Services and Parameters

[http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)

- rosservice
  - rosservice
    cmd | desc
    ---|---
    rosservice list | print information about active services
    rosservice call | call the service with the provided args
    rosservice type | print service type
    rosservice find | find services by service type
    rosservice uri  | print service ROSRPC uri
  - rosservice list
  - rosservice type [service]
    rosservice type /clear
    rosservice type /spawn | rossrv show
  - rosservice call [service] [args]
    rosservice call /clear
    rosservice call /spawn 2 2 0.2 ""
- rosparam
  - rosparam
    cmd | desc
    ---|---
    rosparam set    | set parameter
    rosparam get    | get parameter
    rosparam load   | load parameters from file
    rosparam dump   | dump parameters to file
    rosparam delete | delete parameter
    rosparam list   | list parameter names
  - rosparam list
  - rosparam set [param_name]

    ```cmd
    rosparam set /background_r 150
    rosservice call /clear
    ```

  - rosparam get [param_name]

    ```cmd
    rosparam get /background_g
    rosparam get /
    ```

  - rosparam dump [file_name] [namespace]

    ```cmd
    rosparam dump params.yaml
    ```

  - rosparam load [file_name] [namespace]

    ```cmd
    rosparam load params.yaml copy
    rosparam get /copy/background_b
    ```

### 2.8. Using rqt_console and roslaunch

[http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)

- test rqt_console and ros logger system

  ```sh
  rosrun rqt_console rqt_console
  rosrun rqt_logger_level rqt_logger_level

  rosrun turtlesim turtlesim_node
  rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
  ```

- using roslaunch

```sh
 cd ~/catkin_ws
source devel/setup.bash
roscd beginner_tutorials

mkdir launch
cd launch

vi turtlemimic.launch
```

  NOTE: The directory to store launch files doesn't necessarily have to be named launch. In fact you don't even need to store them in a directory. roslaunch command automatically looks into the passed package and detects available launch files. However, this is considered good practice.

```xml
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

- test roslaunching

```sh
roslaunch beginner_tutorials turtlemimic.launch

rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

### 2.9. Using rosed to edit files in ROS

[http://wiki.ros.org/ROS/Tutorials/UsingRosEd](http://wiki.ros.org/ROS/Tutorials/UsingRosEd)

- rosed [package_name] [filename]
  - rosed roscpp Logger.msg
- change default editor of rosed
  - export EDITOR='nano -w'

### 2.10. Creating a ROS msg a

[http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

- Brief

  msg files are stored in the **msg** directory of a package, and srv files are stored in the **srv** directory.

  - msg

    msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

    msgs are just simple text files with a field type and field name per line. field type:

    - int8, int16, int32, int64 (plus uint*)
    - float32, float64
    - string
    - time, duration
    - other msg files
    - variable-length array[] and fixed-length array[C]

    There is also a special type in ROS: Header, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have Header header.

    example:

    ```msg
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    ```

  - srv

    an srv file describes a service. It is composed of two parts: a request and a response.

    srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. Here is an example of a srv file:

    ```srv
    int64 A
    int64 B
    ---
    int64 Sum
    ```

- Using msg

  ```sh
  roscd beginner_tutorials
  mkdir msg
  echo "int64 num" > msg/Num.msg
  ```

  Note that at build time, we need "message_generation", while at runtime, we only need "message_runtime".

  In package.xml:

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

  Modify **CMakeLists.txt**:

  ```makefile
  # Add message_generation to the list of COMPONENTS.
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
  )

  # Also make sure you export the message runtime dependency.
  catkin_package(
    ...
    CATKIN_DEPENDS message_runtime ...
    ...)

  # add message filter
  add_message_files(
    FILES
    Num.msg
  )

  # enable generate_message() function
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
  ```

  Use **rosmsg** show message info:

  ```sh
  rosmsg show beginner_tutorials/Num
  # or
  rosmsg show Num
  ```

- Using srv

  ```sh
  # create a srv
  roscd beginner_tutorials
  mkdir srv

  # copy an existing one from another package by roscp
  # roscp [package_name] [file_to_copy_path] [copy_path]
  roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
  ```

  In **package.xml**:

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

  In **CMakeLists.txt**:

  ```makefile
  # Add message_generation to the list of COMPONENTS.
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
  )

  # Import service file - AddTwoInts.srv
  add_service_files(
    FILES
    AddTwoInts.srv
  )
  ```

  Use **rossrv** show message info:

  ```sh
  rossrv show beginner_tutorials/AddTwoInts
  ```

- Common step for msg and srv

  Changes in CMakeLists.txt:

  ```makefile
  # add any packages you depend on which contain .msg files that your messages use like this:
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
  ```

  make our package again:

  ```sh
  roscd beginner_tutorials
  cd ../..
  catkin_make install
  cd -
  ```

  Any .msg file in the msg directory will generate code for use in all supported languages:

  language |                                 path
  -------- | --------------------------------------------------------------------
  C++      | ~/catkin_ws/devel/include/beginner_tutorials
  Python2  | ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg
  Lisp     | ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg

  Similarly, any .srv files in the srv directory will have generated code in supported languages.

  language |                                 path
  -------- | --------------------------------------------------------------------
  C++      | ~/catkin_ws/devel/include/beginner_tutorials
  Python2  | ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/srv
  Lisp     | ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/srv

### 2.11. Writing a Simple Publisher and Subscriber (C++)

[http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

```sh
roscd beginner_tutorials
mkdir -p src
```

- src/talker.cpp

  ```cpp
  #include "ros/ros.h"
  #include "std_msgs/String.h"

  #include <sstream>

  int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_talker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("topic_chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
      std_msgs::String msg;

      std::stringstream ss;
      ss << "cpp-talker: hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }

    return 0;
  }
  ```

- src/listener.cpp

  ```cpp
  #include "ros/ros.h"
  #include "std_msgs/String.h"

  void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("cpp-listener: I heard: [%s]", msg->data.c_str());
  }

  int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("topic_chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
  }
  ```

- CMakeLists.txt

  ```makefile
  cmake_minimum_required(VERSION 2.8.3)
  project(beginner_tutorials)

  ## Find catkin and any catkin packages
  find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

  ## Declare ROS messages and services
  add_message_files(DIRECTORY msg FILES Num.msg)
  add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

  ## Generate added messages and services
  generate_messages(DEPENDENCIES std_msgs)

  ## Declare a catkin package
  catkin_package()

  add_executable(talker src/talker.cpp)
  target_link_libraries(talker ${catkin_LIBRARIES})
  add_dependencies(talker beginner_tutorials_generate_messages_cpp)

  add_executable(listener src/listener.cpp)
  target_link_libraries(listener ${catkin_LIBRARIES})
  add_dependencies(listener beginner_tutorials_generate_messages_cpp)
  ```

```sh
# build
cd ~/catkin_ws
catkin_make

# start talker & listener via rosrun.
# Should be started in different consoles.
rosrun rosrun beginner_tutorials talker
rosrun rosrun beginner_tutorials listener
```

### 2.12. Writing a Simple Publisher and Subscriber (Python)

[http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

```sh
roscd beginner_tutorials
mkdir -p scripts
```

- scripts/talker.py

  ```py
  #!/usr/bin/env python
  import rospy
  from std_msgs.msg import String

  def talker():
      pub = rospy.Publisher('topic_chatter-py', String, queue_size=10)
      rospy.init_node('python_talker', anonymous=True)
      rate = rospy.Rate(10) # 10hz
      while not rospy.is_shutdown():
          hello_str = "python talker: hello world %s" % rospy.get_time()
          rospy.loginfo(hello_str)
          pub.publish(hello_str)
          rate.sleep()

  if __name__ == '__main__':
      try:
          talker()
      except rospy.ROSInterruptException:
          pass
  ```

- scripts/listener.py

  ```py
  #!/usr/bin/env python
  import rospy
  from std_msgs.msg import String

  def callback(data):
      rospy.loginfo(rospy.get_caller_id() + "python listener: I heard %s", data.data)

  def listener():
      rospy.init_node('python_listener', anonymous=True)

      rospy.Subscriber("topic_chatter-py", String, callback)

      rospy.spin()

  if __name__ == '__main__':
      listener()
  ```

```sh
chmod +x scripts/talker.py scripts/listener.py

# build
cd ~/catkin_ws
catkin_make

# start talker & listener via rosrun.
# Should be started in different consoles.
rosrun rosrun beginner_tutorials talker
rosrun rosrun beginner_tutorials listener
```

### 2.13. Examining the Simple Publisher and Subscriber

[http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

### 2.14. Writing a Simple Service and Client (C++)

[http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

```sh
roscd beginner_tutorials
mkdir -p src
```

- src/add_two_ints_server.cpp

  ```cpp
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"

  bool add(beginner_tutorials::AddTwoInts::Request  &req,
          beginner_tutorials::AddTwoInts::Response &res)
  {
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
  }
  ```

- src/add_two_ints_client.cpp

  ```cpp
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"
  #include <cstdlib>

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3) {
      ROS_INFO("usage: add_two_ints_client X Y");
      return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    return 0;
  }
  ```

- CMakeLists.txt

  ```makefile
  #......
  add_executable(add_two_ints_server src/add_two_ints_server.cpp)
  target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
  add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

  add_executable(add_two_ints_client src/add_two_ints_client.cpp)
  target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
  add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
  #......
  ```

```sh
cd ~/catkin_ws
catkin_make

roscore

rosrun beginner_tutorials add_two_ints_server

rosrun beginner_tutorials add_two_ints_client 1 2
```

### 2.15. Writing a Simple Service and Client (Python)

[asdf](asdf)

```sh
roscd beginner_tutorials
mkdir -p scripts
```

- scripts/add_two_ints_server.py

  ```py
  #!/usr/bin/env python

  from beginner_tutorials.srv import *
  import rospy

  def handle_add_two_ints(req):
      print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
      return AddTwoIntsResponse(req.a + req.b)

  def add_two_ints_server():
      rospy.init_node('add_two_ints_server')
      s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
      print "Ready to add two ints."
      rospy.spin()

  if __name__ == "__main__":
      add_two_ints_server()
  ```

- scripts/add_two_ints_client.py

  ```py
  #!/usr/bin/env python

  import sys
  import rospy
  from beginner_tutorials.srv import *

  def add_two_ints_client(x, y):
      rospy.wait_for_service('add_two_ints')
      try:
          add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
          resp1 = add_two_ints(x, y)
          return resp1.sum
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def usage():
      return "%s [x y]"%sys.argv[0]

  if __name__ == "__main__":
      if len(sys.argv) == 3:
          x = int(sys.argv[1])
          y = int(sys.argv[2])
      else:
          print usage()
          sys.exit(1)
      print "Requesting %s+%s"%(x, y)
      print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
  ```

```sh
roscd beginner_tutorials
chmod +x scripts/add_two_ints_server.py scripts/add_two_ints_client.py

cd ~/catkin_ws
catkin_make
```

### 2.16. Examining the Simple Service and Client

[http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)

- c++:

  ```sh
  # start service
  rosrun beginner_tutorials add_two_ints_server

  # start client
  rosrun beginner_tutorials add_two_ints_client 1 3
  ```

- python:

  ```sh
  # start service
  rosrun beginner_tutorials add_two_ints_server.py

  # start client
  rosrun beginner_tutorials add_two_ints_client.py
  ```

### 2.17. Recording and playing back data

[http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)

start siulator in seperate terminals:

```sh
# terminal 1
roscore

# terminal 2
rosrun turtlesim turtlesim_node

# terminal 3
rosrun turtlesim turtle_teleop_key
```

recording

```sh
mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -a
```

playback

```sh
rosbag info <your bagfile>

rosbag play <your bagfile>
rosbag play -r 2 <your bagfile>
```

```sh
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

The -O argument tells rosbag record to log to a file named subset.bag, and the topic arguments cause rosbag record to only subscribe to these two topics.

### 2.18. Getting started with roswtf

[http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)

### 2.19. Navigating the ROS wiki

[http://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki](http://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki)





