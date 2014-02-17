#!/bin/bash
#../../../../bootable/bootloader/uboot-imx/tools/mkimage  -A arm -O linux -T kernel -C none -a 0x90008000 -e 0x90008000 -n "Android Linux Kernel" -d ./zImage ./uImage


../../../../bootable/bootloader/uboot-imx-mx53/tools/mkimage    -A arm -O linux -T kernel -C none -a 0x70008000 -e 0x70008000 -n "AndroidLinuxKernel" -d  ./zImage  ./uImage

exit 0
