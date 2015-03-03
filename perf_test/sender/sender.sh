#!/bin/bash

PANID=1234
SRC=1
DST=2

NETDEV=wpan0
CHAN=0
MAC=deadbeafcafebabe

iz add ${1} ${NETDEV} ${MAC}
iz set ${NETDEV} ${PANID} ${SRC} ${CHAN}
ifconfig ${NETDEV} up
echo every dot - received and replyed packet. Press CTRL-C to stop
./sender ${PANID} ${SRC} ${DST}
