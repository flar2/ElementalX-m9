/******************************************************************************
 *
 *  This is the implementation file for the PN547 NFC customization Functions
 *
 ******************************************************************************/


#include <linux/of_gpio.h>
#include <linux/types.h>
#include "pn544_htc.h"

#if NFC_READ_RFSKUID
#include <linux/htc_flags.h>
#define HAS_NFC_CHIP 0x7000000
#endif //NFC_READ_RFSKUID


#define D(x...)	\
	if (is_debug) \
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)


/******************************************************************************
 *
 *  Function pn544_htc_check_rfskuid:
 *  Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *  Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn544_htc_check_rfskuid(int in_is_alive){

#if NFC_READ_RFSKUID
	int nfc_rfbandid_size = 0;
	int i;
	unsigned int *nfc_rfbandid_info;
	struct device_node *nfc_rfbandid;
	nfc_rfbandid = of_find_node_by_path("/chosen/mfg");
	if (nfc_rfbandid){
		nfc_rfbandid_info = (unsigned int *) of_get_property(nfc_rfbandid,"skuid.rf_id",&nfc_rfbandid_size);
	}else {
		E("%s:Get skuid.rf_id fail, keep NFC by default\n",__func__);
		return 1;
	}
	if(nfc_rfbandid_size != 32) {  //32bytes = 4 bytes(int) * 8 rfbandid_info
		E("%s:Get skuid.rf_id size error, keep NFC by default\n",__func__);
		return 1;
	} else if (nfc_rfbandid_info == NULL) {
		E("%s:Get nfc_rfbandid_info NULL, keep NFC by default\n",__func__);
		return 1;
	}

	for ( i = 0; i < 8; i++) {
		if (nfc_rfbandid_info[i] == HAS_NFC_CHIP) {
			I("%s: Check skuid.rf_id done, device has NFC chip\n",__func__);
			return 1;
		}
	}
	I("%s: Check skuid.rf_id done, remove NFC\n",__func__);
	return 0;
#else //NFC_READ_RFSKUID
	return in_is_alive;
#endif //NFC_READ_RFSKUID
}


/******************************************************************************
 *
 *  Function pn544_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return 	NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 *
 *          Bootmode strig is defined in
 *          bootable/bootloader/lk/app/aboot/aboot.c
 *          bootable/bootloader/lk/app/aboot/htc/htc_board_info_and_setting.c
 *
 ******************************************************************************/
int pn544_htc_get_bootmode(void) {
	char sbootmode[30] = "default";
#if NFC_GET_BOOTMODE
	strncpy(sbootmode,htc_get_bootmode(),sizeof(sbootmode));
#endif  //NFC_GET_BOOTMODE
	if (strcmp(sbootmode, "offmode_charging") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_OFF_MODE_CHARGING\n",__func__);
		return NFC_BOOT_MODE_OFF_MODE_CHARGING;
	} else if (strcmp(sbootmode, "ftm") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_FTM\n",__func__);
		return NFC_BOOT_MODE_FTM;
	} else if (strcmp(sbootmode, "download") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_DOWNLOAD\n",__func__);
		return NFC_BOOT_MODE_DOWNLOAD;
	} else {
		I("%s: Check bootmode done NFC_BOOT_MODE_NORMAL mode = %s\n",__func__,sbootmode);
		return NFC_BOOT_MODE_NORMAL;
	}
}

/******************************************************************************
 *
 *  Function htc_parse_dt:
 *  Get platform required GPIO number from device tree
 *  For Power off sequence and OFF_MODE_CHARGING
 *
 ******************************************************************************/
void pn544_htc_parse_dt(struct device *dev) {
}

/******************************************************************************
 *
 *  Function pn544_htc_off_mode_charging
 *  Turn of NFC_PVDD when bootmode = NFC_BOOT_MODE_OFF_MODE_CHARGING
 *
 ******************************************************************************/
#if NFC_HIMA_OFF_MODE_CHARGING
extern void force_disable_PM8994_VREG_ID_L30(void);
#endif  //NFC_HIMA_OFF_MODE_CHARGING
void pn544_htc_off_mode_charging (void) {
#if NFC_HIMA_OFF_MODE_CHARGING
	force_disable_PM8994_VREG_ID_L30();
#endif  //NFC_HIMA_OFF_MODE_CHARGING
}


/******************************************************************************
 *
 *  Function pn544_htc_pvdd_on
 *  Turn on NFC_PVDD
 *
 ******************************************************************************/
int pn544_htc_pvdd_on (void) {
	return 1;
}
