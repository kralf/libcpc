include $(PWD)/Config

KERNEL_SRC ?= /usr/src/linux-headers-$(shell uname -r)
KERNEL_MODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)
CLIBFLAGS += -DCPC_LIB_VERSION=\"$(LIBVER)\" -D_CPC_LIB_COMPAT_MODE

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

lib-install: libcpc.so
	install -c -o root -g root -m 644 libcpc.so /usr/lib

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~ *.a *.so *.d
