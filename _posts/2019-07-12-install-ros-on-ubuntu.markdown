---
layout: post
title:  "Install ROS on Ubuntu"
date:   2019-07-12 11:06:00 +0800
categories: Linux Ubuntu ROS
---

## 1. 前言

### 1.1. 安装说明

以下的安装步骤均参考自 ROS 官网: [https://www.ros.org](https://www.ros.org)。

### 1.2. 关于版本

日前正与曾使用 ROS 系统的朋友聊这个系统，
为方便朋友无缝使用此环境，所以决定：

- 以 Ubuntu 16.04 LTS 为底层操作系统。
- ROS 使用 Kinetic Kame 版本。

在其它操作系统上安装不同版本的 ROS 的说明在 [ROS 官网](https://www.ros.org) 上有详尽说明，在此不再复述。

## 2. 安装步骤

### 2.1. OS 安装与配置

略

### 2.2. IDE 选择、安装与配置

略

### 2.3. ROS 安装

#### 2.3.1. install ROS platform

- setup sources list

  ```sh
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  ```

- setup apt keys

  available key server:

  - hkp://keyserver.ubuntu.com:80
  - hkp://pgp.mit.edu:80

  ```sh
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key   C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  ```

- install ROS

  安装 ROS 有三种选择：

  安装模式 | 命令 | 说明
  ---|---|---
  Desktop-Full Install | sudo apt install ros-kinetic-desktop-full | ROS, rqt, rviz, robot-generic libraries, 2D/3D   imulators and 2D/3D perception
  Desktop Install | sudo apt install ros-kinetic-desktop | ROS, rqt, rviz, and robot-generic libraries
  ROS-Base | sudo apt install ros-kinetic-ros-base | ROS package, build, and communication libraries. No GUI tools.

  - 模式一为官方推荐模式，如安装 PC 端 ROS 开发环境，建议用此模式。
  - 模式二是模式一的减版，为熟悉 ROS 开发，且知道自己项目需要什么模块的开发人员使用。
  - 模式三是安装 ROS 的基础框架，一般安装在 ROS 终端运行设备上。

  为方便，此项目的 PC 端开发、测试环境选择第一种安装模式：

  ```sh
  sudo apt update
  sudo apt install ros-kinetic-desktop-full
  ```

#### 2.3.2. Initialize rosdep

```sh
sudo rosdep init
rosdep update
```

#### 2.3.3. ROS Environment Setup

```sh
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3.4. Install Dependencies for building packages

```sh
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
