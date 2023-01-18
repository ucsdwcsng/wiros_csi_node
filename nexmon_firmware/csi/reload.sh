#!/bin/sh
echo "r $HASRELOAD"
cd "$(dirname "$0")"
killall send.sh
if [ -e "/tmp/home/root/reload.txt" ];
then
	echo "Device has already reloaded"
	cat /tmp/home/root/reload.txt
	exit 0
else
	echo "reloaded" >> /tmp/home/root/reload.txt
fi
/sbin/rmmod dhd ; /sbin/insmod ./dhd.ko ; brctl addif br0 eth6
brctl show br0
