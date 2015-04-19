
#if defined(CONFIG_ARCH_MSM8994)
#define NFC_READ_RFSKUID 1
#define NFC_GET_BOOTMODE 1
#else
#define NFC_READ_RFSKUID 0
#define NFC_GET_BOOTMODE 0
#endif

#ifndef NFC_HIMA_OFF_MODE_CHARGING
#define NFC_HIMA_OFF_MODE_CHARGING 1
#endif

#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5





int pn544_htc_check_rfskuid(int in_is_alive);

int pn544_htc_get_bootmode(void);

void pn544_htc_parse_dt(struct device *dev);

void pn544_htc_off_mode_charging (void);


int pn544_htc_pvdd_on (void);
