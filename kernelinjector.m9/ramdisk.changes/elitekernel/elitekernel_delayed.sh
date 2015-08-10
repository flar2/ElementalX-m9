#!/system/bin/sh

sleep 60 
# do the configuration again to override ROM and tegra hardcoded stuff

# run EliteKernel tweaks (overrides ROM tweaks)
echo "sio" > /sys/block/mmcblk0/queue/scheduler

# set governors
echo "elementalx" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo "elementalx" > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
echo "elementalx" > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
echo "elementalx" > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor
echo 1 > /sys/devices/system/cpu/cpu4/online	
echo "elementalx" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor 
echo 1 > /sys/devices/system/cpu/cpu5/online	
echo "elementalx" > /sys/devices/system/cpu/cpu5/cpufreq/scaling_governor 
echo 1 > /sys/devices/system/cpu/cpu6/online	
echo "elementalx" > /sys/devices/system/cpu/cpu6/cpufreq/scaling_governor 
echo 1 > /sys/devices/system/cpu/cpu7/online	
echo "elementalx" > /sys/devices/system/cpu/cpu7/cpufreq/scaling_governor

# set default speeds 0=LP cluster; 4=HP cluster
echo "1248000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
echo "384000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
echo "1344000" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq
echo "384000" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq

# force hard freq limits
echo 1248000 > /sys/power/pnpmgr/cluster/little/cpu0/thermal_freq
echo 1248000 > /sys/power/pnpmgr/thermal/thermal_final_lcpu
echo 1344000 > /sys/power/pnpmgr/cluster/big/cpu0/thermal_freq
echo 1344000 > /sys/power/pnpmgr/thermal/thermal_final_bcpu
echo 600000 > /sys/power/pnpmgr/thermal/thermal_final_gpu
echo 0 > /sys/power/pnpmgr/touch_boost
echo 0 > /sys/power/pnpmgr/touch_boost_duration
echo 0 > /sys/power/pnpmgr/long_duration_touch_boost
echo 0 > /sys/power/pnpmgr/long_duration_touch_boost_duration

# B0110 L1000 (3:2)
echo 104 > /sys/power/pnpmgr/thermal/thermal_cpus_offlined

# B1110 L1000 (3:1)
# echo 232 > /sys/power/pnpmgr/thermal/thermal_cpus_offlined
# B1010 L1010 (2:2)
# echo 170 > /sys/power/pnpmgr/thermal/thermal_cpus_offlined
# B0000 L0000 (4:4)
# echo 0 > /sys/power/pnpmgr/thermal/thermal_cpus_offlined

echo 40 > /sys/devices/system/cpu/cpu0/sched_budget
echo 40 > /sys/devices/system/cpu/cpu1/sched_budget
echo 40 > /sys/devices/system/cpu/cpu2/sched_budget
echo 40 > /sys/devices/system/cpu/cpu3/sched_budget

echo 100 > /sys/devices/system/cpu/cpu4/sched_budget
echo 100 > /sys/devices/system/cpu/cpu5/sched_budget
echo 100 > /sys/devices/system/cpu/cpu6/sched_budget
echo 100 > /sys/devices/system/cpu/cpu7/sched_budget

# set vm tweaks
sysctl -w vm.min_free_kbytes=8192
sysctl -w vm.vfs_cache_pressure=20
sysctl -w vm.swappiness=10
sysctl -w vm.page-cluster=0
sysctl -w vm.dirty_expire_centisecs=2400
sysctl -w vm.dirty_writeback_centisecs=600
sysctl -w vm.dirty_ratio=15
sysctl -w vm.dirty_background_ratio=20
sysctl -w vm.oom_kill_allocating_task=0
sysctl -w vm.panic_on_oom=0
sysctl -w vm.overcommit_memory=1
sysctl -w vm.overcommit_ratio=20
sysctl -w kernel.panic_on_oops=1
sysctl -w kernel.panic=10

# sio tweaks
echo "2" > /sys/block/mmcblk0/queue/iosched/writes_starved
echo "80" > /sys/block/mmcblk0/queue/iosched/sync_read_expire
echo "400" > /sys/block/mmcblk0/queue/iosched/sync_write_expire
echo "240" > /sys/block/mmcblk0/queue/iosched/async_read_expire
echo "800" > /sys/block/mmcblk0/queue/iosched/async_write_expire

# temporary workaround for stock OTA updater wakelock bugs
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$Receiver
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$SecretCodeReceiver
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$ActiveReceiver
kill $(pidof com.google.android.gms)

touch /data/local/ek_delayed_tweaks

# start user init
# activate delayed config to override Kernel
/system/xbin/busybox nohup /system/bin/sh /data/local/userinit.sh 2>&1 >/dev/null &
/system/xbin/busybox nohup /system/bin/sh /data/local/zramswap.sh 2>&1 >/dev/null &


