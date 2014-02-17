#!/bin/bash
../../../../bootable/bootloader/uboot-imx-rgb24/tools/mkimage  -A arm -O linux -T kernel -C none -a 0x90008000 -e 0x90008000 -n "Android Linux Kernel" -d ./zImage ./uImage
exit 0
