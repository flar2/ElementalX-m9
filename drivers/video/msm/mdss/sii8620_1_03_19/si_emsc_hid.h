/*
 * SiI8620 Linux Driver eMSC HID stuff

Copyright (C) 2013-2014 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
See the GNU General Public License for more details at
http://www.gnu.org/licenses/gpl-2.0.html.
*/


#ifndef _SI_EMSC_HID_H_
#define _SI_EMSC_HID_H_

#include <linux/mod_devicetable.h>
#include <linux/hid.h>

#include "si_fw_macros.h"
#include "si_app_devcap.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_8620_internal_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#include "platform.h"

extern int	debug_level;

#define MHL3_HID_DBG_ERR(...) \
	MHL_TX_GENERIC_DBG_PRINT(DBG_MSG_LEVEL_ERR, __VA_ARGS__)
#define MHL3_HID_DBG_WARN(...) \
	MHL_TX_GENERIC_DBG_PRINT(DBG_MSG_LEVEL_WARN, __VA_ARGS__)
#define MHL3_HID_DBG_INFO(...) \
	MHL_TX_GENERIC_DBG_PRINT(DBG_MSG_LEVEL_INFO, __VA_ARGS__)

#define MHL3_HID_STARTED	(1 << 0)
#define MHL3_HID_CONNECTED	(1 << 1)
#define HID_FLAGS_WQ_ACTIVE	(1 << 2)
#define HID_FLAGS_WQ_CANCEL	(1 << 3)

#define MHL3_HID_PWR_ON		0x00
#define MHL3_HID_PWR_SLEEP	0x01

#define MHL3_HID_MAX_DESC_STR_LEN	256

#define MAX_HID_MESSAGE_CHANNELS	16


#define MHL3_NAME_SIZE	20
#define MHL3_MODULE_PREFIX "mhl3:"

struct mhl3_device_id {
	char name[MHL3_NAME_SIZE];
	kernel_ulong_t driver_data;	
};


#define HID_BURST_ID_LEN		(2 + 1)
#define HID_MSG_HEADER_LEN		2
#define HID_MSG_CHKSUM_LEN		2
#define HID_FRAG_HEADER_LEN		1
#define HID_ACK_PACKET_LEN		2

#define HID_FRAG_LEN_MAX		(EMSC_BLK_MAX_LENGTH - \
					(EMSC_BLK_STD_HDR_LEN + \
					HID_BURST_ID_LEN + \
					HID_FRAG_HEADER_LEN))

#define HID_FRAG_HB0_TYPE		0x80
#define HID_FRAG_HB0_TYPE_ACK		0x80
#define HID_FRAG_HB0_CNT_MSK		0x7F

#define HID_HB0_DEV_ID_MSK		0xF0
#define HID_HB0_ACHID_MSK		0x01
#define HID_ACHID_INT			0x01

#define EMSC_HID_HB1_ACK		0x80
#define EMSC_HID_HB1_MSG_CNT_FLD	0x7F

#define MHL3_HID_ACK			0x00
#define MHL3_REPORT			0x01
#define MHL3_GET_REPORT_DSCRPT		0x02
#define MHL3_REPORT_DSCRPT		0x03
#define MHL3_GET_MHID_DSCRPT		0x04
#define MHL3_MHID_DSCRPT		0x05
#define MHL3_GET_REPORT			0x06
#define MHL3_SET_REPORT			0x07
#define MHL3_DSCRPT_UPDATE		0x08

#define HID_ACK_SUCCESS			0x00
#define HID_ACK_NODEV			0x01
#define HID_ACK_NODATA			0x02
#define HID_ACK_WAIT			0x03
#define HID_ACK_TIMEOUT			0x04
#define HID_ACK_PROTV			0x05
#define HID_ACK_WRTYPE			0x06
#define HID_ACK_WRID			0x07
#define HID_ACK_WRFMT			0x08
#define HID_ACK_WRMFMT			0x09


#define MHL_RHID_REQUEST_HOST		0x00
#define MHL_RHID_RELINQUISH_HOST	0x01

#define MHL_RHID_NO_ERR			0x00
#define MHL_RHID_INVALID		0x01
#define MHL_RHID_DENY			0x02

#define OP_STATE_IDLE			0x00
#define OP_STATE_WAIT_MHID_DSCRPT	0x01
#define OP_STATE_WAIT_REPORT_DSCRPT	0x02
#define OP_STATE_WAIT_REPORT		0x03
#define OP_STATE_CONNECTED		0x04

struct mhl3_hid_desc {
	__u8	bMHL3HIDmessageID;
	__u16	wHIDVendorID;
	__u16	wHIDProductID;
	__u8	bCountryCode;
	__u16	wBcdHID;
	__u16	bBcdDevice;
	__u8	bDeviceClass;
	__u8	bDeviceSubClass;
	__u16	wLanguageID;
	__u8	bProductNameSize;
	__u8	bManufacturerNameSize;
	__u8	bSerialNumberSize;
} __packed;

static DEFINE_MUTEX(mhl3_hid_open_mutex);

struct mhl3_hid_global_data {
	
	uint8_t		is_host;	
	uint8_t		is_device;	
	uint8_t		want_host;	

	int		hid_receive_state;
	uint8_t		hb0;
	uint8_t		hb1;
	uint8_t		in_buf[4096];	
	int		msg_length;
};

struct hid_add_work_struct {
	struct work_struct	work;
	struct mhl3_hid_data	*mhid;
};

struct wq_indata_t {
	uint8_t		*ptr;
	int		buffer_size;
};

#define MAX_HID_INQUEUE		4

struct mhl3_hid_data {
	struct mhl_dev_context	*mdev;		
	struct hid_device	*hid;		

	uint8_t			*in_report_buf;	
	int			bufsize;	

	
	struct mhl3_hid_desc	*hdesc;		
	__u8			desc_product_name[MHL3_HID_MAX_DESC_STR_LEN];
	__u8			desc_mfg_name[MHL3_HID_MAX_DESC_STR_LEN];
	__u8			desc_serial_number[MHL3_HID_MAX_DESC_STR_LEN];

	uint8_t			id;		
	uint8_t			msg_count[2];	
	unsigned long		flags;		

	uint8_t			report_desc[1024];
	uint8_t			in_data[4096];	
	int			in_data_length;
	uint8_t			out_data[4096];	

	int			opState;	

	
	struct hid_add_work_struct	mhl3_work;
	struct semaphore	data_wait_lock;	

};

struct SI_PACK_THIS_STRUCT mhl_hid_report_msg {
	uint8_t	msg_id;
	uint8_t	type;
	uint8_t	id;	
	uint8_t	len_lo;
	uint8_t	len_hi;
	uint8_t	data;	
	};


void mhl_tx_hid_host_role_request(struct mhl_dev_context *context, int request);
void mhl_tx_hid_host_negotiation(struct mhl_dev_context *context);

int mhl3_hid_report_desc_parse(struct mhl3_hid_data *mhid);
void build_received_hid_message(struct mhl_dev_context *context,
		uint8_t *pmsg, int length);

void mhl3_hid_remove_all(struct mhl_dev_context *context);

int mhl3_mt_event(struct hid_device *hid, struct hid_field *field,
		struct hid_usage *usage, __s32 value);
int mhl3_mt_add(struct mhl3_hid_data *mhid, struct hid_device_id *id);
int mhl3_mt_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max);
int mhl3_mt_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max);
void mt_feature_mapping(struct hid_device *hdev,
		struct hid_field *field, struct hid_usage *usage);

void dump_array(int level, char *ptitle, uint8_t *pdata, int count);

#if (LINUX_KERNEL_VER >= 311)
void mt_input_configured(struct hid_device *hdev, struct hid_input *hi);
void mt_report(struct hid_device *hid, struct hid_report *report);
#endif

#endif 
