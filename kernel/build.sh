#!/bin/sh

./make.sh mrproper && \
./make.sh $(basename `pwd` | grep -oh '[0-9]\{3\}')_defconfig && \
./make.sh uImage && \
./make.sh modules && \
./make.sh modules_install && \
echo "All done" 
./make.sh tags >/dev/null &
