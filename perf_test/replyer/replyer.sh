#!/bin/bash

PANID=1234
SRC=2
DST=1

NETDEV=wpan0
CHAN=0
MAC=cafebabedeadbeaf

iz add ${1} ${NETDEV} ${MAC}
iz set ${NETDEV} ${PANID} ${SRC} ${CHAN}
ifconfig ${NETDEV} up
echo every dot - received and replyed packet. Press CTRL-C to stop
./replyer ${PANID} ${SRC} ${DST}
