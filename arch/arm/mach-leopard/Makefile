#
# Makefile for the linux kernel.
#
# Common support
obj-y := common.o cpu.o timer.o asoc_serial.o dma.o \
	 sirq.o gpio.o powergate.o debug.o dvfslevel.o board_opt.o dump_stack.o \
	 bootdev.o  bootafinfo.o secure_storage.o  performance.o timer_monitor.o

obj-y += clock_tree/
obj-y += xml/
obj-y += atc260x/
obj-y += umido/
obj-$(CONFIG_BACKLIGHT_ACTIONS) += pwm.o
obj-$(CONFIG_I2C_ASOC) += asoc_i2c.o
obj-$(CONFIG_SPI_ASOC) += asoc_spi.o
obj-$(CONFIG_SMP) += platsmp.o headsmp.o hotplug.o stand-hotplug.o stand-hotplug.o

obj-$(CONFIG_MACH_LEOPARD_FPGA) += board-leopardfpga.o
obj-$(CONFIG_MACH_LEOPARD_EVB) += board-leopardevb.o
obj-$(CONFIG_PM) += pm.o sleep_leopard.o
obj-$(CONFIG_PINCTRL_ASOC) += pinctrl-asoc.o pinctrl-leopard.o pinctrl-asoc.o
obj-$(CONFIG_CPU_IDLE) += cpuidle.o sleep-ilde.o
obj-$(CONFIG_HIBERNATION)	+= hibernate.o swsusp.o user_dump_stack.o
obj-$(CONFIG_LEOPARD_THERMAL)		+= leopard_thermal.o
# obj-$(CONFIG_CPU_FREQ)		+= cpufreq.o
