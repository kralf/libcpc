##########################
# start configuration part

# enable the interface for you want a driver to be compiled
CPC_PCI_SUPPORTED=true
CPC_CARD_SUPPORTED=false
CPC_XT_SUPPORTED=false
CPC_USB_SUPPORTED=true
CPC_ECO_SUPPORTED=false

# set the maximum count of CAN channels supported by the library
CPC_CAN_MAX_DEVICE=10

SOCKETCAN_SUPPORT=false

# where your kernel sources are
KERNELDIR  = /lib/modules/`uname -r`/build

# the path where user application search for cpc.h and cpclib.h
CPC_H_PATH = /usr/include/linux

GCC    = gcc
DEPMOD = /sbin/depmod

MANPATH= /usr/share/man

# UDEV - only relevant if you are using udev system
# Path to your configuration directory for udev
UDEV_CONF_PATH = /etc/udev

# enable this if you don't use devfs nor udev
CREATE_INODES=false

CLIBFLAGS = -DCAN_MAX_DEVICE=$(CPC_CAN_MAX_DEVICE)

OUTPUT_FILE = /dev/null

# end configuration part
##########################

PWD := $(shell pwd)

LIBVER = 2.42

KERNEL_SRC ?= /usr/src/linux-headers-$(shell uname -r)
KERNEL_MODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)
CLIBFLAGS += -fPIC -DCPC_LIB_VERSION=\"$(LIBVER)\" -DCPC_LIB_COMPAT_MODE

PREFIX = /usr/local

obj-m := cpc-usb.o

cpc-usb-objs := cpc_usb.o sja2m16c.o

all: module lib

module: cpc_usb.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

lib: cpclib.c cpc_async.c ethercan.c ini_parser.c
	$(CC) $(CLIBFLAGS) -shared -o libcpc.so cpclib.c cpc_async.c \
	ethercan.c ini_parser.c
	ln -sf $(PWD)/libcpc.so ../can/libcpc.so

install: module-install lib-install

module-install: cpc-usb.ko
	install -c -o root -g root -m 644 cpc-usb.ko \
	$(KERNEL_MODS)/kernel/drivers/usb/misc
	depmod -ae
	install -c -o root -g root -m 644 udev.rules \
	/etc/udev/rules.d/41-cpc-usb.rules
	udevadm control --reload-rules

lib-install: libcpc.so
	install -c -m 755 libcpc.so $(PREFIX)/lib

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~ *.a *.so *.d
	rm -rf Module.symvers
