#
# Makefile for the kernel serial device drivers.
#
# Advantech eAutomation Division
#


UNAME := $(shell uname -r)
KDIR := /home/dell/Documents/Code/linux
PWD := $(shell pwd)
ADVMOD := adv950


SERIAL_NAME= ttyAP
MAX_TTY= 64 
ADV_TTY_MAJOR= 30
 
SERIAL_FLAGS= -DSERIAL_NAME=\"$(SERIAL_NAME)\" \
              -DADV_TTY_MAJOR=$(ADV_TTY_MAJOR) \
              -DADV_TTY_CUMAJOR=$(ADV_TTY_CUMAJOR)       
	      

SERIAL_FLAGS:= $(SERIAL_FLAGS)

KERNELVERSION := $(shell cd $(KDIR); make kernelversion)
#KERNELVERSION := 6.0.6
split_vers := $(subst ., ,$(KERNELVERSION))
VERSION := $(word 1, $(split_vers))
PATCHLEVEL := $(word 2, $(split_vers))

# SUBDIRS support is removed since Linux 5.3
# need to use KBUILD_EXTMOD instead
# https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?id=7e35b42591c058b91282f95ce3b2cf0c05ffe93d
define mod_select
	if test $(VERSION) -ge 6; then
		echo KBUILD_EXTMOD=;
	elif test $(VERSION) -eq 5 && test $(PATCHLEVEL) -ge 3; then
		echo KBUILD_EXTMOD=;
	else
		echo SUBDIRS=;
	fi
endef


ifneq ($(KERNELRELEASE),)
# When building from kernel tree - preferred way.
EXTRA_CFLAGS  = $(SERIAL_FLAGS)
obj-m := adv950.o
#adv950-objs := 8250.o 8250_pci.o serial_core.o
adv950-objs := 8250.o 8250_pci.o
else
# When building from adv950 source directory - this should be avoided.
all:	
#	$(MAKE) ARCH=i386 -w -C $(KDIR) SUBDIRS=$(PWD) modules
	$(MAKE) ARCH=i386 -w -C $(KDIR) $(shell $(mod_select))$(PWD) modules
endif


install:
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
	 rmmod $(ADVMOD) ; fi)
	@insmod $(ADVMOD).ko

node: default install
	@echo "Creating serial device nodes $(SERIAL_NAME) ..."
	@chmod 777 ../advmknod
	@../advmknod $(SERIAL_NAME) \
	 $$(awk '$$2=="$(SERIAL_NAME)" {print $$1}' /proc/devices) $(MAX_TTY)
	@echo "Done"

uninstall:
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
	 rmmod $(ADVMOD) ; fi)

test:
	@cd ../getconfig; make; ./getconfig $(SERIAL_NAME) $(MAX_TTY)

clean:
	@rm -f *.o
	@rm -f *.ko
	@rm -f *.mod.c modules.order  Module.symvers Module.markers
	@rm -f .*.cmd
	@rm -rf .tmp_versions
	@rm -rf *~
	@cd ../getconfig; make clean
