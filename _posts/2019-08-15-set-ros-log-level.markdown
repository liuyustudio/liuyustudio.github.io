---
layout: post
title:  "Set ROS Log Level"
date:   2019-08-15 20:44:00 +0800
categories: ros log
---

Set ROS Log Level

ref: rosconsole - Configuration ([http://wiki.ros.org/rosconsole#Configuration](http://wiki.ros.org/rosconsole#Configuration))

in config file: **$ROS_ROOT/config/rosconsole.config**

We can change default global log level
or set log level for specified ROS project.

For example, set log leve of project 'pilot', 'interface' and 'cli' to DEBUG:

```conf
#
#   rosconsole will find this file by default at $ROS_ROOT/config/rosconsole.config
#
#   You can define your own by e.g. copying this file and setting
#   ROSCONSOLE_CONFIG_FILE (in your environment) to point to the new file
#
log4j.logger.ros=INFO
log4j.logger.ros.roscpp.superdebug=WARN

# For ROS Car Debug message output
log4j.logger.ros.pilot=DEBUG
log4j.logger.ros.interface=DEBUG
log4j.logger.ros.cli=DEBUG
```
