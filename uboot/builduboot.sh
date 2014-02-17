#!/bin/bash

echo "set xrzebook uboot build environment"
export ARCH=arm

export CROSS_COMPILE=/opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-

echo "configure uboot parameter"
make mx50_rgb_linux_config

echo "build start"
make

echo "exit build"
exit 0
