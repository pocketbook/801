#!/bin/bash
../../../../bootable/bootloader/uboot-imx-mx508/tools/mkimage  -A arm -O linux -T kernel -C none -a 0x70008000 -e 0x70008000 -n "Android Linux Kernel" -d ./zImage ./uImage
exit 0
