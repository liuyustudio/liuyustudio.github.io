---
layout: post
title:  "Update hosts' ip for ipset"
date:   2019-07-23 12:01:00 +0800
categories: ipset iptables script
---

I have create a github repository([LueyEscargot/ipsetUtils](LueyEscargot/ipsetUtils)) for update specified hosts' dynamic IP into ipset.

In this util, I create host db file([host.db](https://github.com/LueyEscargot/ipsetUtils/blob/master/updater/host.db)) and insert hosts' name into it, as shown below:

```cfg
www.cnn.com
www.yahoo.com
www.namesilo.com
```

After host.db has been created, a script named [updater.sh](https://github.com/LueyEscargot/ipsetUtils/blob/master/updater/updater.sh) will read hosts' name(or domain name) from this db file and update corresponding ips into ipset.

the script are listed below:

```sh
#!/bin/bash

HOST_DB_FILE=host.db
SETNAME=hostDynamicIpList
INTERVAL=600

TIMEOUT=`echo "$(( ${INTERVAL} * 150 / 100 ))"`

echo "start update hosts' dynamic ip for ipset"

# create ipset's SET
ipset create ${SETNAME} hash:ip timeout ${TIMEOUT} comment --exist

while [ true ]
do
  hostList=`cat ${HOST_DB_FILE}`
  for host in ${hostList}
  do
    # update IPs
    ips=`getent hosts ${host} | awk '{ print $1 }'`
    for ip in ${ips}
    do
      ipset add ${SETNAME} ${ip} -exist comment "${host}"
    done
  done

  sleep ${INTERVAL}
done
```

For take actions, there are two iptabes' rules:

- Allow incomming connections which from **hostDynamicIpList** ips.
- Allow outgoing connections which to **hostDynamicIpList** ips.

```sh
iptables -I INPUT 1 -m set --match-set hostDynamicIpList src -j ACCEPT
iptables -I OUTPUT 1 -m set --match-set hostDynamicIpList dst -j ACCEPT
```
