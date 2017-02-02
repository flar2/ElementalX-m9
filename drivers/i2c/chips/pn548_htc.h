/******************************************************************************
 *
 *  This is the interface file for the PN548 NFC HTC customization Functions
 *
 ******************************************************************************/

/*for htc platform specified functions*/

#define NFC_READ_RFSKUID 1
#define NFC_GET_BOOTMODE 1

/* Define boot mode for NFC*/
#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5


/******************************************************************************
 *
 *	Function pn548_htc_check_rfskuid:
 *	Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *	Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn548_htc_check_rfskuid(int in_is_alive);

/******************************************************************************
 *
 *  Function pn548_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return  NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 ******************************************************************************/
int pn548_htc_get_bootmode(void);


