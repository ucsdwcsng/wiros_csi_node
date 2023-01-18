#!/bin/bash
CH=$1
BW=$2
SS=$3
MAC=$4
cd "$(dirname "$0")"

if [ "$SS" = "" ]; then
  echo "Missing the number of spatial streams"
  exit 1
fi

use_mac="False"
if [ "$MAC" != "" ]; then
  use_mac="True"
fi

SS_hx=0

case $SS in
1)
  SS_hx=1
  ;;
2)
  SS_hx=3
  ;;
3)
  SS_hx=7
  ;;
4)
  SS_hx=f
  ;;
*)
  echo "Invalid spatial stream"
  exit 1
  ;;
esac

#determine if 2.4 or 5
IFACE="eth6"
if [ "$CH" -lt "15" ]; then
  echo "Detected 2.4G channel"
  IFACE="eth5"
fi

/usr/sbin/wl -i ${IFACE} up
/usr/sbin/wl -i ${IFACE} radio on
/usr/sbin/wl -i ${IFACE} country UG
chspec=$(/usr/sbin/wl -i ${IFACE} chanspec ${CH}/${BW} | tr ' ' '\n' | grep "0x" | tr -d '\n')
/usr/sbin/wl -i ${IFACE} monitor 1
/sbin/ifconfig ${IFACE} up


#change -N to have more spatial streams
echo ${SS_hx}

if [ "$use_mac" = "False" ]; then
  CONFIG=$(./makecsiparams -e 1 -c ${CH}/${BW} -C 0xf -N 0x${SS_hx} -d 0x50)
else
  #dhd.ko has been modified such that only the first two bytes of MAC addresses are compared in the filter
  CONFIG=$(./makecsiparams -e 1 -m ${MAC} -c ${CH}/${BW} -C 0xf -N 0x${SS_hx} -d 0x50)
fi
LEN=38
./nexutil -I ${IFACE} -s500 -b -l${LEN} -v ${CONFIG} 


# ./setdumpparameters.sh 2 0 $IFACE

/usr/sbin/wl -i ${IFACE} chanspec ${CH}/${BW}