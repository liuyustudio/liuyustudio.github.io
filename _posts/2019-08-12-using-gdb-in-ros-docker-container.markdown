---
layout: post
title:  "Using GDB in ROS Docker Container"
date:   2019-08-12 07:00:00 +0800
categories: ros gdb docker
---

## How to use GDB in ROS Docker Container

I have create a docker container for testing my ROS Car project.
Everything is fine except I can't interrupt ROS node in GDB by press Ctrl-C:

```sh
(gdb) liuyu@rosCar:/$ rosrun --prefix 'gdb -ex run --args' interface interface
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from /car/devel/lib/interface/interface...done.
(gdb) r
Starting program: /car/devel/lib/interface/interface
warning: Error disabling address space randomization: Operation not permitted
[ INFO] [1565575659.551420042]: Info: init interface.
[ INFO] [1565575659.551468314]: Info: init uds.
[ INFO] [1565575659.551565410]: Info: ROS Car Interface Ready.
^C


```

After searching on internet, I got the answer.

The root cause of this issue is(ref: [https://stackoverflow.com/a/35860616](https://stackoverflow.com/a/35860616)):

```txt
For whatever reason, your user account doesn't have permission to disable the kernel's address space layout randomisation for this process. By default, gdb turns this off because it makes some sorts of debugging easier (in particular, it means the address of stack objects will be the same each time you run your program). Read more here([http://visualgdb.com/gdbreference/commands/set_disable-randomization](http://visualgdb.com/gdbreference/commands/set_disable-randomization)).

You can work around this problem by disabling this feature of gdb with set disable-randomization off.

As for getting your user the permission needed to disable ASLR, it probably boils down to having write permission to /proc/sys/kernel/randomize_va_space. Read more here([https://askubuntu.com/questions/318315/how-can-i-temporarily-disable-aslr-address-space-layout-randomization](https://askubuntu.com/questions/318315/how-can-i-temporarily-disable-aslr-address-space-layout-randomization)).
```

The solution for me is(ref: [https://stackoverflow.com/a/46676907](https://stackoverflow.com/a/46676907)):

```txt
If you're using Docker, you probably need the --security-opt seccomp=unconfined option (as well as enabling ptrace):

docker run --cap-add=SYS_PTRACE --security-opt seccomp=unconfined
```

Finnaly, I use following command to create my ROS docker container for solve current issue:

```sh
docker run -it --hostname rosCar --name rosCar --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v /workspace/project/rosCar/car:/car roscar:ros-kinetic-base bash
```

In this new created ROS docker container, GDB works again:

```sh
rosrun --prefix 'gdb -ex run --args' interface interface
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from /car/devel/lib/interface/interface...done.
Starting program: /car/devel/lib/interface/interface
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
[New Thread 0x7ffff15da700 (LWP 127)]
[New Thread 0x7ffff0dd9700 (LWP 128)]
[New Thread 0x7fffebfff700 (LWP 129)]
[New Thread 0x7fffeb7fe700 (LWP 134)]
[ INFO] [1565576986.453850646]: Info: init interface.
[ INFO] [1565576986.453923330]: Info: init uds.
[New Thread 0x7fffeaffd700 (LWP 135)]
[ INFO] [1565576986.454122216]: Info: ROS Car Interface Ready.
[ INFO] [1565576986.454143848]: ROS Car Interface Sleeping...
^C
Thread 1 "interface" received signal SIGINT, Interrupt.
0x00007ffff717dc1d in nanosleep () at ../sysdeps/unix/syscall-template.S:84
84      ../sysdeps/unix/syscall-template.S: No such file or directory.
(gdb)
```
