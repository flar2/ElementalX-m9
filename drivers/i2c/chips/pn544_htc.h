/******************************************************************************
 *
 *  This is the interface file for the PN547 NFC HTC customization Functions
 *
 ******************************************************************************/

/*for htc platform specified functions*/
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

/* Define boot mode for NFC*/
#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5





/******************************************************************************
 *
 *	Function pn544_htc_check_rfskuid:
 *	Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *	Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn544_htc_check_rfskuid(int in_is_alive);

/******************************************************************************
 *
 *  Function pn544_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return  NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 ******************************************************************************/
int pn544_htc_get_bootmode(void);

/******************************************************************************
 *
 *  Function htc_parse_dt:
 *  Get platform required GPIO number from device tree
 *  For Power off sequence and OFF_MODE_CHARGING
 *
 ******************************************************************************/
void pn544_htc_parse_dt(struct device *dev);

/******************************************************************************
 *
 *  Function pn544_htc_off_mode_charging
 *  Turn of NFC_PVDD when bootmode = NFC_BOOT_MODE_OFF_MODE_CHARGING
 *
 ******************************************************************************/
void pn544_htc_off_mode_charging (void);


/******************************************************************************
 *
 *  Function pn544_htc_pvdd_on
 *  Turn on NFC_PVDD
 *
 ******************************************************************************/
int pn544_htc_pvdd_on (void);
