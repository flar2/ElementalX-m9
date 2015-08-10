#!/system/bin/sh


# make sure init.d is ok
mount -o remount,rw /system
chmod -R 775 /system/etc/init.d
find /system/etc/init.d -type f -name '*' -exec chown 0:2000 {} \;
#chgrp -R 2000 /system/etc/init.d
mount -o remount,ro /system
sync

# feed urandom data to /dev/random to avoid system blocking (potential security risk, use at own peril!)
cp -f /elitekernel/rngd /data/local/rngd
chmod 0755 /data/local/rngd
/data/local/rngd --rng-device=/dev/urandom --random-device=/dev/random --background --feed-interval=10 --fill-watermark=3072 --random-step=128

# disable KSM and use zcache instead
echo 1 > /sys/kernel/mm/ksm/run

# startup time tweaks:

#I/O tweaks
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /cache /cache
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /data /data
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /sdcard /sdcard
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /sdcard2 /sdcard2
mount -o async,remount,noatime,nodiratime,delalloc,noauto_da_alloc,barrier=0,nobh /devlog /devlog
echo "1024" > /sys/block/mmcblk0/bdi/read_ahead_kb;
echo "1024" > /sys/block/mmcblk0/queue/read_ahead_kb;
echo "1024" > /sys/block/mmcblk1/bdi/read_ahead_kb;
echo "1024" > /sys/block/mmcblk1/queue/read_ahead_kb;
echo "1024" > /sys/block/mmcblk0rpmb/bdi/read_ahead_kb;
echo "1024" > /sys/block/mmcblk0rpmb/queue/read_ahead_kb;

# activate delayed config to override ROM
/system/xbin/busybox nohup /system/bin/sh /elitekernel/elitekernel_delayed.sh 2>&1 >/dev/null &


