ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/accelgyro/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

#inlcude SPI Interface Start
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif
#include SPI Interface End

obj-$(CONFIG_MTK_BMI160_SPI)	+=  bmi160_accgyro.o

