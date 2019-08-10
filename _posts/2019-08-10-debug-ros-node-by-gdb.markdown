---
layout: post
title:  "Debug ROS node by GDB"
date:   2019-08-10 21:38:00 +0800
categories: linux ssh
---

Debug ROS node by GDB

- compile node to debug version

  ```sh
  catkin_make -DCMAKE_BUILD_TYPE=Debug
  ```

- start ROS node(interface/interface, for example) with GDB

  ```sh
  rosrun --prefix 'gdb -ex run --args' interface interface
  ```
