include $(PWD)/Config

KERNEL_SRC ?= /usr/src/linux-source-$(shell uname -r)
KERNEL_MODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)

obj-m := cpc-usb.o

cpc-usb-objs := cpc-usb.o sja2m16c.o

all: cpc-usb.c sja2m16c.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

install: cpc-usb.ko
	install -c -o root -g root -m 644 cpc-usb.ko \
	$(KERNEL_MODS)/kernel/drivers/usb/misc
	depmod -ae

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
