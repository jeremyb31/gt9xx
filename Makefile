MODULE_NAME := goodix_backport

MODULE_FILENAME := goodix

LINUXINCLUDE := -I$(PWD)/include $(LINUXINCLUDE)

$(MODULE_NAME)-y	:= $(MODULE_FILENAME).o

obj-m			+= $(MODULE_NAME).o

KVER  := $(shell uname -r)
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

install: $(MODULE_NAME).ko $(MODULE_NAME).mod.c
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules_install
	/sbin/depmod -a ${KVER}

uninstall:
	/bin/bash restore.sh $(MODULE_NAME)
	/sbin/depmod -a ${KVER}

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

