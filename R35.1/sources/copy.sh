#!/bin/sh
#scp kernel_out/drivers/media/i2c/max96712.ko kernel_out/drivers/media/i2c/nv_ar0233.ko kernel_out/drivers/media/i2c/nv_max9295.ko kernel_out/arch/arm64/boot/Image kernel_out/drivers/media/i2c/gmsl/ar0233.ko alex@192.168.137.245:~/alex
#scp kernel_out/arch/arm64/boot/dts/nvidia/tegra234-p3701-0000-p3737-0000.dtb alex@192.168.137.245:~/alex

scp kernel_out/drivers/media/i2c/gmsl/max9295B.ko kernel_out/drivers/media/i2c/gmsl/ar0233.ko alex@192.168.137.245:~/alex
