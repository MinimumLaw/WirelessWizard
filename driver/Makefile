#
#
#

KDIR ?= /lib/modules/`uname -r`/build/
INSTALL_MOD_PATH ?= /

obj-m += wpan_usb.o
mirs_wpan-objs := wpan_usb.o

MAKE_ARGS = -C ${KDIR} M=${PWD} INSTALL_MOD_PATH=${INSTALL_MOD_PATH}

all: modules

modules:
	make ${MAKE_ARGS} modules

clean:
	make ${MAKE_ARGS} clean

modules_install: modules
	sudo make ${MAKE_ARGS} modules_install
