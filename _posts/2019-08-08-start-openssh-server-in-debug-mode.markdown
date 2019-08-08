---
layout: post
title:  "Start openssh-server in debug mode"
date:   2019-08-08 23:11:00 +0800
categories: linux ssh
---

Start openssh-server in debug mode and listen on different tcp port:

- copy openssh-server config file

  ```sh
  sudo cp -r /etc/ssh/sshd_config /tmp/sshd_config
  ```

- set ssh sever listen port in new created config file

  ```sh
  vim /tmp/ssh/sshd_config
  ```

- start ssh server in debug mode

  ```sh
  /usr/sbin/sshd -dD /tmp/ssh/sshd_config
  ```
