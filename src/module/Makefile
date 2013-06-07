##########################
# where your kernel sources are
KERNELDIR ?= /lib/modules/`uname -r`/build

GCC = gcc
DEPMOD = /sbin/depmod

# end configuration part
##########################

PWD := $(shell pwd)

KERNELSRC ?= /usr/src/linux-headers-$(shell uname -r)
KERNELMODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)

obj-m := cpc-usb.o

cpc-usb-objs := cpc_usb.o sja2m16c.o

all: module

module: cpc_usb.c
	@$(MAKE) -C $(KERNELSRC) SUBDIRS=$(PWD) M=$(PWD) modules

install: module-install

module-install: cpc-usb.ko
	install -c -o root -g root -m 644 cpc-usb.ko \
	$(KERNELMODS)/kernel/drivers/usb/misc
	$(DEPMOD) -ae

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~ *.a *.so *.d
	rm -rf Module.symvers Module.markers modules.order
