#!/system/bin/sh

sleep 60 
# do the configuration again to override ROM and tegra hardcoded stuff

# run EliteKernel tweaks (overrides ROM tweaks)
echo "sio" > /sys/block/mmcblk0/queue/scheduler

# set governors
echo "smartmax" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# set default speeds (cpus activate in order 0-3-2-1)
echo "1500000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
echo "51000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq

# set ondemand prefs
echo "80" > /sys/devices/system/cpu/cpufreq/ondemand/up_threshold
echo "15" > /sys/devices/system/cpu/cpufreq/ondemand/down_differential
echo "1" > /sys/devices/system/cpu/cpufreq/ondemand/ignore_nice_load
echo "3000000" > /sys/devices/system/cpu/cpufreq/ondemand/input_boost_duration
echo "1" > /sys/devices/system/cpu/cpufreq/ondemand/io_is_busy
echo "5" > /sys/devices/system/cpu/cpufreq/ondemand/powersave_bias
echo "5" > /sys/devices/system/cpu/cpufreq/ondemand/sampling_down_factor
echo "30000" > /sys/devices/system/cpu/cpufreq/ondemand/sampling_rate
echo "10000" > /sys/devices/system/cpu/cpufreq/ondemand/sampling_rate_min
echo "1" > /sys/devices/system/cpu/cpufreq/ondemand/touch_poke
echo "51000" > /sys/devices/system/cpu/cpufreq/ondemand/two_phase_bottom_freq
echo "1" > /sys/devices/system/cpu/cpufreq/ondemand/two_phase_dynamic
echo "340000" > /sys/devices/system/cpu/cpufreq/ondemand/two_phase_freq
echo "3" > /sys/devices/system/cpu/cpufreq/ondemand/ui_counter
echo "20000" > /sys/devices/system/cpu/cpufreq/ondemand/ui_sampling_rate
echo "66" > /sys/devices/system/cpu/cpufreq/ondemand/ux_boost_threshold
echo "760000" > /sys/devices/system/cpu/cpufreq/ondemand/ux_freq
echo "20" > /sys/devices/system/cpu/cpufreq/ondemand/ux_loading

# set vm tweaks
sysctl -w vm.min_free_kbytes=8192
sysctl -w vm.vfs_cache_pressure=30
sysctl -w vm.swappiness=20
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

# minfree
echo "0,1,2,5,7,15" > /sys/module/lowmemorykiller/parameters/adj
echo "1536,3072,6144,10240,12288,18432" > /sys/module/lowmemorykiller/parameters/minfree

# temporary workaround for stock OTA updater wakelock bugs
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$Receiver
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$SecretCodeReceiver
pm disable com.google.android.gms/com.google.android.gms.update.SystemUpdateService\$ActiveReceiver
kill $(pidof com.google.android.gms)

touch /data/local/em_delayed_tweaks

# start user init
# activate delayed config to override Kernel
/system/xbin/busybox nohup /system/bin/sh /data/local/userinit.sh 2>&1 >/dev/null &
/system/xbin/busybox nohup /system/bin/sh /data/local/zramswap.sh 2>&1 >/dev/null &


