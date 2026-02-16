#!/bin/sh
echo "*** starting Xantrex XC Pro Marine Service ***"
exec 2>&1
. /etc/profile.d/profile.sh
exec softlimit -d 100000000 -s 1000000 -a 100000000 /usr/bin/python3 /data/xantrex-monitor/xantrex-service.py 

