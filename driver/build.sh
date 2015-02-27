#!/bin/sh

export ARCH=arm
export CROSS_COMPILE=armv5tel-softfloat-linux-gnueabi-
export KDIR=/cimc/build/ravion-kernel
export INSTALL_MOD_PATH=/cimc/root/armv5tel-generic/exports

make $*