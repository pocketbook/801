#ifndef __WAVEFORM_COMMAND_H
#define __WAVEFORM_COMMAND_H



#define BOOTARGS_COMMAND  "bootargs"
#define BOOTARGS_RECOVERY_COMMAND  "bootargs_android_recovery"

#define BOOTARGS_WAVEFORM_COMMAND_DATA0 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E156_WS0501 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA1 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220SCM init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA2 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C011_WA1001 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA3 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C012_WA2001 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA4 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E324_WA1801 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA5 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C124_WS0B01 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA6 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C105_WN5E21 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA7 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C015_WA0301 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA8 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E162_WN6621 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA9 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E120_WN5401 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA10 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C130_WS0801 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA11 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E305_WA0501 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA12 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C015_WJ2601 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA13 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C015_WJA301 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA14 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C050_WJ1B01 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA15 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_E157_WA1206 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA16 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E60_V220_C031_WJB201 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"
#define BOOTARGS_WAVEFORM_COMMAND_DATA17 "setenv bootargs console=ttymxc0 video=mxcepdcfb:E80_V250 init=/init  root=/dev/mmcblk0p4 rootfs=ext4 androidboot.console=ttymxc0"


#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA0 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E156_WS0501 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA1 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220SCM init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA2 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C011_WA1001 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA3 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C012_WA2001 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA4 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E324_WA1801 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA5 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C124_WS0B01 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA6 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C105_WN5E21 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA7 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C015_WA0301 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA8 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E162_WN6621 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA9 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E120_WN5401 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA10 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C130_WS0801 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA11 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E305_WA0501 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA12 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C015_WJ2601 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA13 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C015_WJA301 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA14 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C050_WJ1B01 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA15 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_E157_WA1206 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA16 "setenv bootargs ${bootargs}  video=mxcepdcfb:E60_V220_C031_WJB201 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "
#define BOOTARGS_ANDROID_RECOVERY_WAVEFORM_COMMAND_DATA17 "setenv bootargs ${bootargs}  video=mxcepdcfb:E80_V250 init=/init root=/dev/mmcblk0p4 rootfs=ext4 "


#endif
