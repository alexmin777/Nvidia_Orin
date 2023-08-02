ccflags-y += -I$(srctree)/drivers/media/platform/tegra
ccflags-y += -I$(srctree)/drivers/video/tegra/camera

obj-$(CONFIG_SER_VIDEOMAX9295B) += max9295B.o
obj-$(CONFIG_DES_VIDEOMAX96712) += maxim96712.o

new_ar0233-objs += serializer.o
new_ar0233-objs += deserializer.o
new_ar0233-objs += ar0233.o
obj-$(CONFIG_VIDEO_DRIVER_AR0233) += new_ar0233.o
