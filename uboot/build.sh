#!/bin/sh

set -x

# get result of preprosesor work
if [ -n "$1" ]
then 
	TARGET=$1
	OUT_TARGET=${TARGET%".i"}"_cpp.c"
	./make.sh $1
	file_extention=$(basename "$TARGET" | sed 's/^.*\.//')
	if [ $file_extention != "i" ] ; then exit 0 ; fi
	cat $TARGET | grep -v ^# | uniq > $OUT_TARGET
	astyle $OUT_TARGET
	exit 0
fi

# adds hg revision to uboot
echo -n "." > localversion-1
hg id -i >> localversion-1

./make.sh mrproper
./make.sh 801_config
./make.sh tags
./make.sh

