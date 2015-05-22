#!/system/bin/sh


# make sure init.d is ok
mount -o remount,rw /system
chmod -R 775 /system/etc/init.d
find /system/etc/init.d -type f -name '*' -exec chown 0:2000 {} \;
#chgrp -R 2000 /system/etc/init.d
mount -o remount,ro /system
sync

# disable KSM and use zcache instead
echo 0 > /sys/kernel/mm/ksm/run

# startup time tweaks:

# enable smartdimmer
echo "1" > /sys/devices/tegradc.0/smartdimmer/enable

#I/O tweaks
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /cache /cache
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /data /data
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /sd-ext /sd-ext
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /devlog /devlog
echo "2048" > /sys/block/mmcblk0/bdi/read_ahead_kb;
echo "2048" > /sys/block/mmcblk0/queue/read_ahead_kb;

# activate delayed config to override ROM
/system/xbin/busybox nohup /system/bin/sh /elitekernel/elitekernel_delayed.sh 2>&1 >/dev/null &


