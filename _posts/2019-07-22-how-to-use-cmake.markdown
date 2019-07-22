---
layout: post
title:  "How to use CMake"
date:   2019-07-22 21:54:00 +0800
categories: ROS
---

CMake 使用

## 1. 生成 Debug、Release 版本

参考自[bytefreaks.net](http://bytefreaks.net/programming-2/cc-how-do-you-set-gdb-debug-flag-g-with-cmake)

### 1.1. Solution 1: Modify the CMakeLists.txt file

Add the following line to your CMakeLists.txt file to set the compilation mode to Debug (non-optimized code with debug symbols):

```cfg
set(CMAKE_BUILD_TYPE Debug)
```

Add the following line to your CMakeLists.txt file to set the compilation mode to RelWithDebInfo (optimized code with debug symbols):

```cfg
set(CMAKE_BUILD_TYPE RelWithDebInfo)
```

### 1.2. Solution 2: Add a command line argument to cmake command

Modify as follows your cmake command to set the compilation mode to Debug (non-optimized code with debug symbols):

```cfg
cmake -DCMAKE_BUILD_TYPE=Debug <path and other arguments>
```

Modify as follows your cmake command to set the compilation mode to RelWithDebInfo (optimized code with debug symbols):

```cfg
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo <path and other arguments>
```

### 1.3. Bonus material

The difference between Debug and RelwithDebInfo modes is that RelwithDebInfo optimizes the code similarly to the behavior of Release mode. It produces fully optimized code, but also creates the symbol table and the debug metadata to give the debugger as much input as it is possible to map the execution back to the original code at any time.

Code build with RelwithDebInfo mode should not have it’s performance degraded in comparison to Release mode, as the symbol table and debug metadata that are generated do not live in the executable code section, so they should not affect the performance when the code is run without a debugger attached.

## 2. 复制文件

参考自[stackoverflow](https://stackoverflow.com/a/34800667)

You may consider using configure_file with the COPYONLY option:

```cfg
configure_file(<input> <output> COPYONLY)
```

Unlike file(COPY ...) it creates a file-level dependency between input and output, that is:

```txt
If the input file is modified the build system will re-run CMake to re-configure the file and generate the build system again.
```

## 3. 生成动态库

参考: [https://stackoverflow.com/a/45843676](https://stackoverflow.com/a/45843676)

```cmake
cmake_minimum_required(VERSION 3.9)
project(mylib VERSION 1.0.1 DESCRIPTION "mylib description")
include(GNUInstallDirs)
add_library(mylib SHARED src/mylib.c)
set_target_properties(mylib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER api/mylib.h)
configure_file(mylib.pc.in mylib.pc @ONLY)
target_include_directories(mylib PRIVATE .)
install(TARGETS mylib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_BINARY_DIR}/mylib.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
```
