include $(PWD)/Config

KERNEL_SRC ?= /usr/src/linux-source-$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)

obj-m := cpc-usb.o

all: cpc-usb.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
