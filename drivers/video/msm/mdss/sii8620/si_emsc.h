/*
 * SiI8620 Linux Driver
 *
 * Copyright (C) 2013 Silicon Image, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 * This program is distributed AS-IS WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; INCLUDING without the implied warranty
 * of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
 * See the GNU General Public License for more details at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#ifndef _SI_EMSC_H_
#define _SI_EMSC_H_

#include <linux/mod_devicetable.h>

#define MHL3_HID_STARTED        (1 << 0)
#define MHL3_HID_CONNECTED	(1 << 1)

#define MHL3_HID_PWR_ON          0x00
#define MHL3_HID_PWR_SLEEP       0x01

#define mhl3_hid_dbg(context, fmt, arg...) \
	do { \
		if (debug) \
			dev_printk(KERN_DEBUG, context->mhl_dev, fmt, ##arg); \
	} while (0)
#define MHL3_HID_MAX_DESC_STR_LEN	(255*4)
#define MHL3_NAME_SIZE	20
#define MHL3_MODULE_PREFIX "mhl3:"
struct mhl3_device_id {
	char name[MHL3_NAME_SIZE];
	
	kernel_ulong_t driver_data __aligned(sizeof(kernel_ulong_t));
};
#define EMSC_RCV_BUFFER_DEFAULT     256
#define EMSC_BLOCK_MAX_LENGTH       256
#define BLK_STD_HEADER_LENGTH       2
#define BLK_CMD_MAX_LENGTH          (EMSC_BLOCK_MAX_LENGTH - \
	BLK_STD_HEADER_LENGTH)
#define BURST_ID_LENGTH             2
#define EMSC_WR_FIFO_SIZE           600	
#define EMSC_RD_FIFO_SIZE           600	
#define EMSC_HID_PAYLOAD                0x0062
#define EMSC_HID_PAYLOAD_HEADER_SIZE    3
#define EMSC_HID_FRAG_HEADER_SIZE       1	
#define EMSC_HID_HB_HEADER_SIZE         2	
#define EMSC_HID_MSG_HEADER_SIZE        3	
#define EMSC_HID_MSG_CHECKSUM_LEN       2
#define EMSC_BLK_RCV_BUFFER_INFO        0x0063
#define EMSC_BITS_PER_PIXEL_FMT         0x0064
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
#define HID_ACK_SUCCESS             0x00	
#define HID_ACK_NODEV               0x01
#define HID_ACK_NODATA              0x02	
#define HID_ACK_WAIT                0x03	
#define HID_ACK_TIMEOUT             0x04
#define HID_ACK_PROTV               0x05	
#define HID_ACK_WRTYPE              0x06	
#define HID_ACK_WRID                0x07	
#define HID_ACK_WRFMT               0x08	
#define HID_ACK_WRMFMT              0x09	
#define MHL_RHID_REQUEST_HOST		0x00	
#define MHL_RHID_RELINQUISH_HOST	0x01	
#define MHL_RHID_NO_ERR		0x00	
#define MHL_RHID_INVALID	0x01	
#define MHL_RHID_DENY		0x02	
struct mhl3_hid_desc {
	__u8 bMHL3HIDmessageID;
	__u16 wHIDVendorID;	
	__u16 wHIDProductID;	
	__u8 bCountryCode;
	__u16 wBcdHID;	
	__u16 bBcdDevice;
	__u8 bDeviceClass;
	__u8 bDeviceSubClass;
	__u16 wLanguageID;	
	__u8 bProductNameSize;
	__u8 bManufacturerNameSize;
	__u8 bSerialNumberSize;
} __packed;

static DEFINE_MUTEX(mhl3_hid_open_mutex);
struct mhl3_emsc_data {
	
	uint8_t is_host;	
	uint8_t is_device;	
	uint8_t want_host;	
	int hid_receive_state;
	int last_fragment_count;
	uint8_t hb0;
	uint8_t hb1;
	uint8_t in_buffer[4096];	
	int msg_length;
	struct semaphore data_wait_lock;	
};

struct hid_add_work_struct {
	struct work_struct work;
	struct mhl3_hid_data *mhid;
};

struct mhl3_hid_data {
	struct mhl_dev_context *context;	
	struct hid_device *hid;	
	uint8_t *in_report_buf;	
	int bufsize;	
	
	struct mhl3_hid_desc *hdesc;	
	__u8 desc_product_name[MHL3_HID_MAX_DESC_STR_LEN];
	__u8 desc_mfg_name[MHL3_HID_MAX_DESC_STR_LEN];
	__u8 desc_serial_number[MHL3_HID_MAX_DESC_STR_LEN];
	uint8_t id;	
	uint8_t ctrl_msg_cnt;	
	uint8_t intr_msg_cnt;	
	unsigned long flags;	
	uint8_t report_desc[1024];	
	uint8_t in_data[4096];	
	int in_data_length;
	uint8_t out_data[4096];	
	bool peer_wants_ack;
	
	struct hid_add_work_struct mhl3_work;
	bool hid_work_active;
};

void mhl_tx_hid_host_role_request(struct mhl_dev_context *context,
	int request);
void mhl_tx_hid_host_negotiation(struct mhl_dev_context *context);
void si_mhl_tx_emsc_received(struct mhl_dev_context *context);
int si_mhl_tx_emsc_add_hid_message(struct mhl_dev_context *context,
	uint8_t hb0, uint8_t hb1, uint8_t *msg_body, int msg_body_length);
void mhl3_hid_message_processor(struct mhl_dev_context *context, uint8_t hb0,
	uint8_t hb1, uint8_t *pmsg, int length);
int mhl3_hid_remove(struct mhl_dev_context *context, int device_id);
void mhl3_hid_remove_all(struct mhl_dev_context *context);
void dump_mhl_device_context(struct mhl_dev_context *context,
	const char *func);
void dump_array(char *ptitle, uint8_t * pdata, int count);
#endif 
