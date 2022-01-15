HOST_ARCH       ?= $(shell uname -m | sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/)
ARCH            ?= arm

CFLAGS_minimig-irq.o := -fno-stack-protector
KERNEL_SRC=/home/markw/fpga/svn/Linux-Kernel_MiSTer/
ifdef KERNEL_SRC
  KERNEL_SRC_DIR  := $(KERNEL_SRC)
else
  KERNEL_SRC_DIR  ?= /lib/modules/$(shell uname -r)/build
endif

ifeq ($(ARCH), arm)
 ifneq ($(HOST_ARCH), arm)
   CROSS_COMPILE  ?= arm-linux-gnueabihf-
 endif
endif
ifeq ($(ARCH), arm64)
 ifneq ($(HOST_ARCH), arm64)
   CROSS_COMPILE  ?= aarch64-linux-gnu-
 endif
endif

minimig-irq-obj           := minimig-irq.o
obj-$(CONFIG_MINIMIG_IRQ) += $(minimig-irq-obj)

ifndef MINIMIGIRQ_MAKE_TARGET
  KERNEL_VERSION_LT_5 ?= $(shell awk '/^VERSION/{print int($$3) < 5}' $(KERNEL_SRC_DIR)/Makefile)
  ifeq ($(KERNEL_VERSION_LT_5), 1)
    MINIMIGIRQ_MAKE_TARGET ?= modules
  else
    MINIMIGIRQ_MAKE_TARGET ?= minimig-irq.ko
  endif
endif

all:
	echo ${CROSS_COMPILE}
	echo ${KERNEL_SRC_DIR}
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) obj-m=$(minimig-irq-obj) $(MINIMIGIRQ_MAKE_TARGET) KSRC=$(KERNEL_SRC_DIR)

modules_install:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) obj-m=$(minimig-irq-obj) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

