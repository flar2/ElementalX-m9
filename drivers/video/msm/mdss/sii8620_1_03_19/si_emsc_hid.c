/*
 * MHL3 HID Tunneling implementation
 *
 * Copyright (c) 2013-2014 Lee Mulcahy <william.mulcahy@siliconimage.com>
 * Copyright (c) 2013-2014 Silicon Image, Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 * This program is distributed AS-IS WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; INCLUDING without the implied warranty
 * of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
 * See the GNU General Public License for more details at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * This code is inspired by the "HID over I2C protocol implementation"
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * and the "USB HID support for Linux"
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/workqueue.h>

#include "si_fw_macros.h"
#include "si_app_devcap.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_8620_internal_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "si_mhl_callback_api.h"
#include "si_8620_drv.h"
#include "mhl_linux_tx.h"
#include "si_8620_regs.h"
#include "mhl_supp.h"
#include "platform.h"
#include "si_emsc_hid.h"

#define EMSC_RCV_MSG_START	0  
#define EMSC_RCV_MSG_NEXT	1  

void dump_array(int level, char *ptitle, uint8_t *pdata, int count)
{
	int i, buf_offset;
	int bufsize = 128;
	char *buf;

	if (level > debug_level)
		return;

	if (count > 1) {
		bufsize += count * 3;
		bufsize += ((count / 16) + 1) * 8;
	}

	buf = kmalloc(bufsize, GFP_KERNEL);
	if (!buf)
		return;

	buf_offset = 0;
	if (ptitle)
		buf_offset = scnprintf(&buf[0], bufsize,
			"%s (%d bytes):", ptitle, count);
	for (i = 0; i < count; i++) {
		if ((i & 0x0F) == 0)
			buf_offset += scnprintf(&buf[buf_offset],
				bufsize - buf_offset, "\n%04X: ", i);
		buf_offset += scnprintf(&buf[buf_offset],
			bufsize - buf_offset, "%02X ", pdata[i]);
	}
	buf_offset += scnprintf(&buf[buf_offset], bufsize - buf_offset, "\n");
	print_formatted_debug_msg(NULL, NULL, -1, buf);
	kfree(buf);
}

struct cbus_req *hid_host_role_request_done(struct mhl_dev_context *mdev,
	struct cbus_req *req, uint8_t data1)
{
	MHL_TX_DBG_ERR("\n")
	return req;
}

void mhl_tx_hid_host_role_request(struct mhl_dev_context *context, int request)
{
	MHL3_HID_DBG_INFO("RHID: Sending HID Host role %s message\n",
		(request == MHL_RHID_REQUEST_HOST) ?
		"REQUEST" : "RELINQUISH");
	
	context->mhl_ghid.is_host = false;
	context->mhl_ghid.is_device = false;
	context->mhl_ghid.want_host = false;

	si_mhl_tx_send_msc_msg(context, MHL_MSC_MSG_RHID, request,
		hid_host_role_request_done);
}

struct cbus_req *rhidk_done(struct mhl_dev_context *mdev,
	struct cbus_req *req, uint8_t data1)
{
	MHL_TX_DBG_ERR("\n")
	return req;
}

void mhl_tx_hid_host_negotiation(struct mhl_dev_context *mdev)
{
	uint8_t	rhidk_status = MHL_RHID_NO_ERR;

	if (mdev->msc_msg_sub_command == MHL_MSC_MSG_RHID) {
		if (mdev->msc_msg_data == MHL_RHID_REQUEST_HOST) {
			if (mdev->mhl_ghid.is_host)
				rhidk_status = MHL_RHID_DENY;
		} else if (mdev->msc_msg_data == MHL_RHID_RELINQUISH_HOST) {
			mdev->mhl_ghid.want_host = true;
		} else {
			rhidk_status = MHL_RHID_INVALID;
		}

		MHL3_HID_DBG_INFO(
			"RHID: Received HID Host role %s result: %d\n",
			(mdev->msc_msg_data == MHL_RHID_REQUEST_HOST) ?
				"REQUEST" : "RELINQUISH", rhidk_status);

		
		si_mhl_tx_send_msc_msg(mdev, MHL_MSC_MSG_RHIDK, rhidk_status,
			rhidk_done);

	} else if (mdev->msc_msg_sub_command == MHL_MSC_MSG_RHIDK) {
		if (mdev->msc_msg_data == MHL_RHID_NO_ERR) {
			if (mdev->msc_msg_last_data == MHL_RHID_REQUEST_HOST) {
				mdev->mhl_ghid.is_host = true;
				mdev->mhl_ghid.is_device = false;
				MHL3_HID_DBG_INFO("HID HOST role granted\n");
			} else {
				mdev->mhl_ghid.is_host = false;
				mdev->mhl_ghid.is_device = true;
				MHL3_HID_DBG_INFO("HID DEVICE role granted\n");
			}
		} else if (mdev->msc_msg_last_data == MHL_RHID_REQUEST_HOST) {
			mdev->mhl_ghid.is_host = false;
			mdev->mhl_ghid.is_device = true;
			MHL3_HID_DBG_INFO("HID HOST role DENIED\n");
		}
	}
}

static int si_mhl_tx_emsc_add_hid_message(struct mhl_dev_context *mdev,
	uint8_t hb0, uint8_t hb1, uint8_t *msg, int msg_len)
{
	int	i, cmd_size, fragment_count, msg_index, index;
	uint8_t	*payload;
	uint8_t	payload_size;
	uint16_t accum;
	uint16_t *pchksum;
	bool	first_fragment;

	i = msg_len + HID_MSG_HEADER_LEN + HID_MSG_CHKSUM_LEN;
	fragment_count = i / HID_FRAG_LEN_MAX;
	if ((fragment_count * HID_FRAG_LEN_MAX) != i)
		fragment_count++;

	
	cmd_size =
		HID_BURST_ID_LEN +
		HID_FRAG_HEADER_LEN +
		HID_MSG_HEADER_LEN +
		HID_MSG_CHKSUM_LEN +
		msg_len;

	first_fragment = true;
	msg_index = 0;
	index = 0;
	accum = 0;

	index = 0;
	while (fragment_count > 0) {
		payload_size = (cmd_size > EMSC_BLK_CMD_MAX_LEN) ?
			EMSC_BLK_CMD_MAX_LEN : cmd_size;

		
		payload = (uint8_t *)si_mhl_tx_get_sub_payload_buffer(
			mdev, payload_size);
		if (payload == NULL) {
			MHL3_HID_DBG_ERR(
				"%ssi_mhl_tx_get_sub_payload_buffer failed%s\n",
				ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		} else {
			payload[index++] = (uint8_t)(burst_id_HID_PAYLOAD >> 8);
			payload[index++] = (uint8_t)(burst_id_HID_PAYLOAD);
			payload[index++] = payload_size - HID_BURST_ID_LEN;
			payload[index++] = fragment_count - 1;
			payload_size -=
				(HID_BURST_ID_LEN + HID_FRAG_HEADER_LEN);
			if (fragment_count == 1)
				payload_size -= HID_MSG_CHKSUM_LEN;

			if (first_fragment) {
				first_fragment = false;
				payload[index++] = hb0;
				payload[index++] = hb1;
				payload_size -= HID_MSG_HEADER_LEN;

				accum = ((((uint16_t)hb1) << 8) | hb0);
				pchksum = (uint16_t *)&msg[0];
				for (i = 0; i < (msg_len / 2); i++)
					accum += pchksum[i];
				if (msg_len & 0x01)
					accum += ((uint16_t)msg[msg_len - 1]);
				accum += (msg_len + HID_MSG_HEADER_LEN);
			}
			memcpy(&payload[index], &msg[msg_index], payload_size);
			msg_index += payload_size;
			index += payload_size;

			if (fragment_count == 1) {
				payload[index++] = (uint8_t)accum;
				payload[index++] = (uint8_t)(accum >> 8);
			}
		}
		fragment_count--;
	}
	return 0;
}

static int send_hid_msg(struct mhl3_hid_data *mhid, int outlen, bool want_ack)
{
	struct mhl_dev_context *mdev = mhid->mdev;
	uint8_t hb0, hb1;
	int ret;

	hb0 = mhid->id << 4;

	if (want_ack) {
		hb1 = mhid->msg_count[0] | EMSC_HID_HB1_ACK;
		mhid->msg_count[0] =
			(mhid->msg_count[0] + 1) & EMSC_HID_HB1_MSG_CNT_FLD;
	}
	hb1 = mhid->msg_count[0] | (want_ack ? EMSC_HID_HB1_ACK : 0x00);
	mhid->msg_count[0] =
		(mhid->msg_count[0] + 1) & EMSC_HID_HB1_MSG_CNT_FLD;
	ret = si_mhl_tx_emsc_add_hid_message(mdev, hb0, hb1,
		mhid->out_data, outlen);
	if (ret == 0)
		si_mhl_tx_push_block_transactions(mdev);

	return ret;
}

static int send_hid_wait(struct mhl3_hid_data *mhid,
	int outlen, uint8_t *pin, int inlen, bool want_ack)
{
	struct mhl_dev_context *mdev = mhid->mdev;
	int count, ret;

	if (down_timeout(&mhid->data_wait_lock, 1*HZ)) {
		MHL3_HID_DBG_ERR("Could not acquire data_wait lock !!!\n");
		return -ENODEV;
	}
	MHL3_HID_DBG_INFO("Acquired data_wait_lock\n");

	if (down_interruptible(&mdev->isr_lock)) {
		MHL3_HID_DBG_ERR(
			"Could not acquire isr_lock for HID work queue\n");
		ret = -ERESTARTSYS;
		goto done_release;
	}

	
	if (mhid->flags & HID_FLAGS_WQ_CANCEL) {
		ret = -ENODEV;
		up(&mdev->isr_lock);
		goto done_release;
	}

	ret = send_hid_msg(mhid, outlen, want_ack);
	up(&mdev->isr_lock);
	if (ret == 0) {
		
		if (down_timeout(&mhid->data_wait_lock, 15*HZ)) {
			MHL3_HID_DBG_WARN("Timed out waiting for HID msg!\n");
			ret = -EBUSY;

			goto done_release;
		}

		
		if ((mhid->in_data[0] == MHL3_HID_ACK) &&
			(mhid->in_data[1] == HID_ACK_NODEV)) {
			ret = -ENODEV;
			goto done_release;
		}

		
		count = (mhid->in_data_length > inlen) ?
			inlen : mhid->in_data_length;
		memcpy(pin, mhid->in_data, count);
		ret = count;
	}

done_release:
	up(&mhid->data_wait_lock);
	return ret;
}

static int send_ack_packet(struct mhl_dev_context *mdev,
	uint8_t hb0, uint8_t hb1)
{
	uint8_t	*payload;

	payload = (uint8_t *)si_mhl_tx_get_sub_payload_buffer(
			mdev, HID_BURST_ID_LEN + HID_ACK_PACKET_LEN);
	if (payload == NULL) {
		MHL3_HID_DBG_ERR(
			"%ssi_mhl_tx_get_sub_payload_buffer failed%s\n",
			ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
	} else {
		payload[0] = (uint8_t)(burst_id_HID_PAYLOAD >> 8);
		payload[1] = (uint8_t)(burst_id_HID_PAYLOAD);
		payload[2] = HID_ACK_PACKET_LEN;
		payload[3] = hb1 | 0x80;
		payload[4] = hb0;
	}
	return 0;
}

static int mhl3_int_send_ack(struct mhl_dev_context *mdev,
	int reason, uint8_t hb0)
{
	int status;
	uint8_t out_data[2];

	MHL3_HID_DBG_WARN("HID_ACK reason code: %02X\n", reason);
	out_data[0] = MHL3_HID_ACK;
	out_data[1] = (reason < 0) ? HID_ACK_NODEV : reason;

	status = si_mhl_tx_emsc_add_hid_message(mdev, hb0, 0, out_data, 2);
	if (status < 0)
		MHL3_HID_DBG_ERR("MHID: Failed to send HID_ACK to device.\n");
	return status;
}

static int mhl3_send_ack(struct mhl3_hid_data *mhid, uint8_t reason)
{
	int status;
	struct mhl_dev_context *mdev = mhid->mdev;

	if (mhid == 0)
		return -ENODEV;

	MHL3_HID_DBG_WARN("%s - HID_ACK reason code: %02X\n", __func__, reason);
	MHL3_HID_DBG_ERR("mhid->mdev: %p\n", mhid->mdev);
	mhid->out_data[0] = MHL3_HID_ACK;
	mhid->out_data[1] = reason;

	if (down_interruptible(&mdev->isr_lock)) {
		MHL3_HID_DBG_ERR("Could not acquire isr_lock for HID work\n");
		return -ERESTARTSYS;
	}
	status = send_hid_msg(mhid, 2, false);
	up(&mdev->isr_lock);
	if (status < 0)
		MHL3_HID_DBG_ERR("Failed to send a HID_ACK to device.\n");
	return status;
}

static int mhl3_hid_get_report(struct mhl3_hid_data *mhid, u8 report_type,
	u8 report_id, unsigned char *pin, int inlen)
{
	int ret = 0;

	
	mhid->out_data[0] = MHL3_GET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	ret = send_hid_wait(mhid, 3, pin, inlen, false);
	if (ret < 0)
		MHL3_HID_DBG_ERR("Failed to retrieve report from device.\n");

	return ret;
}

static int mhl3_hid_set_report(struct mhl3_hid_data *mhid, u8 report_type,
	u8 report_id, unsigned char *pout, size_t outlen)
{
	struct mhl_dev_context *mdev = mhid->mdev;
	int ret = 0;

	mhid->out_data[0] = MHL3_SET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	memcpy(&mhid->out_data[3], pout, outlen);


	if (down_interruptible(&mdev->isr_lock)) {
		MHL3_HID_DBG_ERR("Could not acquire isr_lock for HID work\n");
		return -ERESTARTSYS;
	}
	ret = send_hid_msg(mhid, outlen + 3, false);
	up(&mdev->isr_lock);
	if (ret) {
		MHL3_HID_DBG_ERR("Failed to set a report to device.\n");
		return ret;
	}

	return outlen + 3;
}

static int mhl3_hid_set_power(struct mhl_dev_context *context, int power_state)
{
	int ret = 0;

	return ret;
}

static int mhl3_hid_alloc_buffers(struct mhl3_hid_data *mhid,
				  size_t report_size)
{
	mhid->in_report_buf = kzalloc(report_size, GFP_KERNEL);
	if (mhid->in_report_buf == NULL) {
		kfree(mhid->hdesc);
		mhid->hdesc = NULL;
		kfree(mhid->in_report_buf);
		mhid->in_report_buf = NULL;
		return -ENOMEM;
	}
	mhid->bufsize = report_size;

	return 0;
}

#if (LINUX_KERNEL_VER < 315)
static int mhl3_hid_get_raw_report(struct hid_device *hid,
	unsigned char report_number, __u8 *buf,
	size_t count, unsigned char report_type)
{
	struct mhl3_hid_data *mhid = hid->driver_data;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	ret = mhl3_hid_get_report(mhid,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, buf, count);
	return ret;
}

static int mhl3_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
	size_t count, unsigned char report_type)
{
	struct mhl3_hid_data *mhid = hid->driver_data;
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	if (report_id) {
		buf++;
		count--;
	}

	ret = mhl3_hid_set_report(mhid,
		report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
		report_id, buf, count);

	if (report_id && ret >= 0)
		ret++; 

	return ret;
}
#endif

static int mhl3_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

static void mhl3_hid_find_max_report(struct hid_device *hid, unsigned int type,
	unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = mhl3_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

#if 0
static void mhl3_hid_init_report(struct hid_report *report, u8 *buffer,
	size_t bufsize)
{
	struct hid_device *hid = report->device;
	struct mhl3_hid_data *mhid = hid->driver_data;
	unsigned int size, ret_size;

	size = mhl3_hid_get_report_length(report);
	if (mhl3_hid_get_report(mhid,
		report->type == HID_FEATURE_REPORT ? 0x03 : 0x01,
		report->id, buffer, size))
		return;

	ret_size = buffer[0] | (buffer[1] << 8);

	if (ret_size != size) {
		MHL3_HID_DBG_ERR("Error in %s size:%d / ret_size:%d\n",
			__func__, size, ret_size);
		return;
	}

	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}
#endif

#if 0
static void mhl3_hid_init_reports(struct hid_device *hid)
{
	struct mhl3_hid_data *mhid = hid->driver_data;
	struct hid_report *report;

	MHL3_HID_DBG_INFO("%s\n", __func__);
	list_for_each_entry(report,
		&hid->report_enum[HID_INPUT_REPORT].report_list, list)
		mhl3_hid_init_report(
			report, mhid->in_report_buf, mhid->bufsize);

	list_for_each_entry(report,
		&hid->report_enum[HID_FEATURE_REPORT].report_list, list)
		mhl3_hid_init_report(
			report, mhid->in_report_buf, mhid->bufsize);
}
#endif

static int mhl3_hid_open(struct hid_device *hid)
{
	struct mhl_dev_context *mdev = 0;
	struct mhl3_hid_data *mhid = 0;
	int ret = 0;

	if (hid)
		mhid = hid->driver_data;
	if (mhid == 0)
		return 0;
	mdev = mhid->mdev;

	mutex_lock(&mhl3_hid_open_mutex);
	if (!hid->open++) {
		ret = mhl3_hid_set_power(mdev, MHL3_HID_PWR_ON);
		if (ret) {
			hid->open--;
			goto done;
		}
	mhid->flags |= MHL3_HID_STARTED;
	}
done:
	mutex_unlock(&mhl3_hid_open_mutex);
	return ret;
}

static void mhl3_hid_close(struct hid_device *hid)
{
	struct mhl_dev_context *mdev = 0;
	struct mhl3_hid_data *mhid = 0;

	if (hid)
		mhid = hid->driver_data;
	if (mhid == 0)
		return;
	mdev = mhid->mdev;

	mutex_lock(&mhl3_hid_open_mutex);
	if (!--hid->open) {
		mhid->flags &= ~MHL3_HID_STARTED;
		mhl3_hid_set_power(mdev, MHL3_HID_PWR_SLEEP);
	}
	mutex_unlock(&mhl3_hid_open_mutex);
}

static int mhl3_hid_power(struct hid_device *hid, int lvl)
{
	int ret = 0;

	MHL3_HID_DBG_ERR("level: %d\n", lvl);

	switch (lvl) {
	case PM_HINT_FULLON:
		break;
	case PM_HINT_NORMAL:
		break;
	}
	return ret;
}

static struct hid_ll_driver mhl3_hid_ll_driver = {
	.open = mhl3_hid_open,
	.close = mhl3_hid_close,
	.power = mhl3_hid_power,
};

static int mhid_fetch_hid_descriptor(struct mhl3_hid_data *mhid)
{
	struct mhl3_hid_desc *hdesc;
	uint8_t *pdesc_raw;
	int ret = -ENODEV;
	int desc_len, raw_offset;

	
	desc_len = sizeof(struct mhl3_hid_desc) +
		sizeof(mhid->desc_product_name) +
		sizeof(mhid->desc_mfg_name) +
		sizeof(mhid->desc_serial_number);
	pdesc_raw = kmalloc(desc_len, GFP_KERNEL);
	if (!pdesc_raw) {
		MHL3_HID_DBG_ERR("Couldn't allocate raw descriptor memory\n");
		return -ENOMEM;
	}
	MHL3_HID_DBG_INFO("Fetching the HID descriptor\n");

	mhid->out_data[0] = MHL3_GET_MHID_DSCRPT;
	mhid->out_data[1] = 0;
	mhid->out_data[2] = 0;
	mhid->opState = OP_STATE_WAIT_MHID_DSCRPT;
	ret = send_hid_wait(mhid, 3, pdesc_raw, desc_len, false);
	if ((ret < 0) || (mhid->opState == OP_STATE_IDLE)) {
		MHL3_HID_DBG_ERR(
			"Failed to get MHID descriptor: %d.\n", ret);
		goto raw_cleanup;
	}

	hdesc = kmalloc(desc_len, GFP_KERNEL);
	if (!hdesc) {
		MHL3_HID_DBG_ERR("Couldn't allocate hdesc descriptor memory\n");
		ret = -ENOMEM;
		goto raw_cleanup;
	}

	

	
	memcpy(hdesc, pdesc_raw, sizeof(struct mhl3_hid_desc));
	raw_offset = sizeof(struct mhl3_hid_desc);

	
	if (hdesc->bMHL3HIDmessageID != 0x05) {
		MHL3_HID_DBG_ERR("Invalid MHID_DSCRPT data\n");
		ret = -EINVAL;
		goto hdesc_cleanup;
	}

	if (ret < (sizeof(struct mhl3_hid_desc) +
		hdesc->bProductNameSize +
		hdesc->bManufacturerNameSize +
		hdesc->bSerialNumberSize)) {

		memcpy(mhid->desc_product_name,
			&pdesc_raw[raw_offset], hdesc->bProductNameSize);
		raw_offset += hdesc->bProductNameSize;
		memcpy(mhid->desc_mfg_name,
			&pdesc_raw[raw_offset], hdesc->bManufacturerNameSize);
		raw_offset += hdesc->bManufacturerNameSize;
		memcpy(mhid->desc_serial_number,
			&pdesc_raw[raw_offset], hdesc->bSerialNumberSize);
	}

	kfree(mhid->hdesc);
	mhid->hdesc = hdesc;
	ret = 0;
	goto raw_cleanup;

hdesc_cleanup:
	kfree(hdesc);
raw_cleanup:
	kfree(pdesc_raw);
	return ret;
}


static struct hid_driver mhl3_hid_driver = {

	.name = "mhl3_hid",
};

static struct hid_driver mhl3_mt3_hid_driver = {

	.name = "mhl3_hid-mt",
	.event = mhl3_mt_event,
	.input_mapping = mhl3_mt_input_mapping,
	.input_mapped = mhl3_mt_input_mapped,
	.feature_mapping = mt_feature_mapping,
#if (LINUX_KERNEL_VER >= 311)
	.input_configured = mt_input_configured,
	.report = mt_report,
#endif
};


int mhl3_hid_report_desc_parse(struct mhl3_hid_data *mhid)
{
	unsigned int report_len;
	int ret;
	uint8_t	*report_desc_raw = NULL;
	struct hid_device *hdev;

	hdev = mhid->hid;
	report_desc_raw = kmalloc(HID_MAX_DESCRIPTOR_SIZE, GFP_KERNEL);
	if (!report_desc_raw) {
		MHL3_HID_DBG_ERR("Allocate raw report descriptor mem failed\n");
		ret = -ENOMEM;
		goto err;
	}
	mhid->out_data[0] = MHL3_GET_REPORT_DSCRPT;
	mhid->opState = OP_STATE_WAIT_REPORT_DSCRPT;
	ret = send_hid_wait(mhid, 1, report_desc_raw,
		HID_MAX_DESCRIPTOR_SIZE, false);
	if (ret < 0) {
		MHL3_HID_DBG_ERR(
			"Failed to get descriptor from device: %d.\n", ret);
		goto err;
	}
	if (mhid->opState == OP_STATE_IDLE)
		goto done;
	if (report_desc_raw[0] != MHL3_REPORT_DSCRPT) {
		MHL3_HID_DBG_ERR(
			"Invalid response to MHL3_GET_REPORT_DSCRPT\n");
		ret = -EINVAL;
		goto err;
	}
	report_len = report_desc_raw[2];
	report_len <<= 8;
	report_len |= report_desc_raw[1];
	report_len = (report_len < (ret-3)) ? report_len : (ret-3);
	ret = hid_parse_report(hdev, &report_desc_raw[3], report_len);
	if (ret)
		goto err;

#if (LINUX_KERNEL_VER >= 305)
	ret = hid_open_report(hdev);
	if (ret) {
		MHL3_HID_DBG_ERR("hid_open_report failed\n");
		goto err;
	}
#endif
	goto done;
err:
	MHL3_HID_DBG_ERR("WORK QUEUE mhl3_hid_report_desc_parse() FAIL\n");
done:
	kfree(report_desc_raw);
	return ret;
}

static void mhl3_disconnect_and_destroy_hid_device(struct mhl3_hid_data *mhid)
{
	if (mhid->hid) {
		if (mhid->hid->claimed & HID_CLAIMED_INPUT) {
			hidinput_disconnect(mhid->hid);
			mhid->hid->claimed &= ~HID_CLAIMED_INPUT;
		}
		hid_destroy_device(mhid->hid);
		mhid->hid = 0;
	}
}

static void mhid_add_work(struct work_struct *workdata)
{
	struct mhl3_hid_data *mhid =
		((struct hid_add_work_struct *)workdata)->mhid;
	struct mhl_dev_context *mdev = mhid->mdev;
	struct hid_device *hdev;
	struct hid_device_id id = {0};
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;
	int ret;

	mhid->flags |= HID_FLAGS_WQ_ACTIVE;
	MHL3_HID_DBG_ERR("WORK QUEUE function executing\n");

	mdev->mhl_hid[mhid->id] = mhid;
	ret = mhid_fetch_hid_descriptor(mhid);	
	if ((ret < 0) || (mhid->opState == OP_STATE_IDLE))
		goto mhid_cleanup;

	
	if (mhid->hid == NULL) {
		hdev = hid_allocate_device();
		if (IS_ERR(hdev)) {
			ret = PTR_ERR(hdev);
			goto mhid_cleanup;
		}

		mhid->hid = hdev;
		hdev->driver = &mhl3_mt3_hid_driver;
		hdev->driver_data = mhid;
		id.vendor = mhid->hdesc->wHIDVendorID;
		id.product = mhid->hdesc->wHIDProductID;
		MHL3_HID_DBG_ERR("Allocated HID\n");
	}
	hdev = mhid->hid;

	hdev->ll_driver			= &mhl3_hid_ll_driver;
#if (LINUX_KERNEL_VER < 315)
	hdev->hid_get_raw_report	= mhl3_hid_get_raw_report;
	hdev->hid_output_raw_report	= mhl3_hid_output_raw_report;
#endif

	hdev->dev.parent		= NULL;
	hdev->bus			= BUS_VIRTUAL;

	hdev->version			= mhid->hdesc->wBcdHID;
	hdev->vendor			= mhid->hdesc->wHIDVendorID;
	hdev->product			= mhid->hdesc->wHIDProductID;

	snprintf(hdev->name, sizeof(hdev->name), "MHL3 HID %04hX:%04hX",
		hdev->vendor, hdev->product);

	
	ret = mhl3_mt_add(mhid, &id);
	if (ret) {
		MHL3_HID_DBG_INFO("NOT Multitouch, trying generic HID\n");
		hdev->driver = &mhl3_hid_driver;
		ret = mhl3_hid_report_desc_parse(mhid);
	}
	if ((ret < 0) || (mhid->opState == OP_STATE_IDLE))
		goto mhid_cleanup;

	if ((mhid->flags & MHL3_HID_CONNECTED) == 0) {
		if (!hidinput_connect(hdev, 0)) {
			MHL3_HID_DBG_INFO(
				"%s hidinput_connect succeeded\n", __func__);
			hdev->claimed |= HID_CLAIMED_INPUT;
			mhid->flags |= MHL3_HID_CONNECTED;

		} else {
			MHL3_HID_DBG_ERR(
				"%s hidinput_connect FAILED\n", __func__);
			goto mhid_cleanup;
		}
	}

	mhl3_hid_find_max_report(hdev, HID_INPUT_REPORT, &bufsize);
	mhl3_hid_find_max_report(hdev, HID_OUTPUT_REPORT, &bufsize);
	mhl3_hid_find_max_report(hdev, HID_FEATURE_REPORT, &bufsize);
	if (bufsize > mhid->bufsize) {
		kfree(mhid->in_report_buf);
		mhid->in_report_buf = NULL;
		ret = mhl3_hid_alloc_buffers(mhid, bufsize);
		if (ret)
			goto mhid_cleanup;
	}

	mhid->opState = OP_STATE_CONNECTED;

	MHL3_HID_DBG_ERR("WORK QUEUE function SUCCESS\n");
	mhid->flags &= ~HID_FLAGS_WQ_ACTIVE;
	return;

mhid_cleanup:

	mhl3_send_ack(mhid, HID_ACK_NODEV);

	mhid->flags |= HID_FLAGS_WQ_CANCEL;
	MHL3_HID_DBG_ERR("WORK QUEUE function FAIL - mhid: %p\n", mhid);
	mhl3_disconnect_and_destroy_hid_device(mhid);

	if (down_interruptible(&mdev->isr_lock)) {
		MHL3_HID_DBG_ERR("Could not acquire isr_lock\n");
		return;
	}

	kfree(mhid->hdesc);
	kfree(mhid->in_report_buf);
	kfree(mhid);
	mdev->mhl_hid[mhid->id] = 0;

	up(&mdev->isr_lock);
	MHL3_HID_DBG_ERR("WORK QUEUE function exit\n");
}

static int mhid_add(struct mhl_dev_context *mdev, int dev_id)
{
	struct mhl3_hid_data *mhid;
	int status;

	MHL3_HID_DBG_ERR("Adding device %d\n", dev_id);

	if (dev_id >= MAX_HID_MESSAGE_CHANNELS)
		return -EINVAL;

	mhid = kzalloc(sizeof(struct mhl3_hid_data), GFP_KERNEL);
	if (!mhid)
		return -ENOMEM;

	sema_init(&mhid->data_wait_lock, 1);

	mhid->mhl3_work.mhid = mhid;
	mhid->id = dev_id;
	mhid->mdev = mdev;

	INIT_WORK((struct work_struct *)&mhid->mhl3_work, mhid_add_work);
	status = queue_work(
		mdev->hid_work_queue,
		(struct work_struct *)&mhid->mhl3_work);
	return 0;
}

static int mhid_remove(struct mhl_dev_context *mdev, int dev_id)
{
	struct mhl3_hid_data *mhid;

	if (dev_id >= MAX_HID_MESSAGE_CHANNELS)
		return -EINVAL;

	mhid = mdev->mhl_hid[dev_id];
	if (mhid == NULL)
		return 0;
	if ((mhid->flags & HID_FLAGS_WQ_ACTIVE) == 0) {
		mhl3_disconnect_and_destroy_hid_device(mhid);
		kfree(mhid->hdesc);
		kfree(mhid->in_report_buf);
		kfree(mhid);
		mdev->mhl_hid[dev_id] = 0;
	} else {
		mhid->flags |= HID_FLAGS_WQ_CANCEL;

		
		if (down_trylock(&mhid->data_wait_lock))
			MHL3_HID_DBG_ERR("Waiting for data when HID removed\n");
		up(&mhid->data_wait_lock);
		}
	return 0;
}

void mhl3_hid_remove_all(struct mhl_dev_context *mdev)
{
	int dev_id;

	for (dev_id = 0; dev_id < MAX_HID_MESSAGE_CHANNELS; dev_id++) {
		if (mdev->mhl_hid[dev_id])
			mhid_remove(mdev, dev_id);
	}
}

static void hid_message_processor(struct mhl_dev_context *mdev,
	uint8_t hb0, uint8_t hb1, uint8_t *pmsg, int length)
{
	struct mhl3_hid_data *mhid;
	uint8_t	msg_id;
	bool	want_ack;
	bool	matched_expected_message;
	int	opState, dev_id, hid_msg_count, header_len, ret;
	int	msg_channel;

	dev_id = (hb0 >> 4) & 0x0F;
	msg_channel = (hb0 & 0x01);
	want_ack = ((hb1 & 0x80) != 0);
	hid_msg_count = hb1 & 0x7F;
	msg_id = pmsg[0];
	matched_expected_message = false;

	if (msg_id != 1) {
		MHL3_HID_DBG_WARN(
			"Received message: msg_id: %02X, deviceID: %02X\n",
			msg_id, dev_id);
	}

	mhid = mdev->mhl_hid[dev_id];
	opState = OP_STATE_IDLE;
	if (mhid == 0) {
		MHL3_HID_DBG_WARN("Message received with mhid == 0\n");
		if (msg_id != MHL3_DSCRPT_UPDATE) {
			MHL3_HID_DBG_ERR(
				"Message NOT MHL3_DSCRPT_UPDATE: %02X\n",
				msg_id);
			mhl3_int_send_ack(mdev, HID_ACK_TIMEOUT, hb0);
			return;
		}
	} else {
		if (want_ack) {
			if (hid_msg_count !=
				(mhid->msg_count[msg_channel] + 1)) {
				send_ack_packet(
					mdev, hb0,
					mhid->msg_count[msg_channel]);
				return;
			} else {
				mhid->msg_count[msg_channel] += 1;
			}
		}
		opState = mhid->opState;
	}


	if ((msg_id == MHL3_DSCRPT_UPDATE)  && (mhid != 0)) {
		opState = OP_STATE_IDLE;
		mhid_remove(mdev, dev_id);
	}

	switch (opState) {
	case OP_STATE_IDLE:
		if (msg_id == MHL3_DSCRPT_UPDATE) {
			MHL3_HID_DBG_ERR("call mhid_add from OP_STATE_IDLE\n");
			ret = mhid_add(mdev, dev_id);
			if (ret < 0)
				mhl3_int_send_ack(mdev, ret, hb0);
		} else {
			mhl3_int_send_ack(mdev, HID_ACK_PROTV, hb0);
		}
		return;

	case OP_STATE_WAIT_MHID_DSCRPT:
		MHL3_HID_DBG_INFO("OP_STATE_WAIT_MHID_DSCRPT\n");
		if ((msg_id == MHL3_MHID_DSCRPT) ||
			(msg_id == MHL3_DSCRPT_UPDATE)) {
			matched_expected_message = true;
		}
		break;

	case OP_STATE_WAIT_REPORT_DSCRPT:
		MHL3_HID_DBG_INFO("OP_STATE_WAIT_REPORT_DSCRPT\n");
		if ((msg_id == MHL3_REPORT_DSCRPT) ||
			(msg_id == MHL3_DSCRPT_UPDATE))
			matched_expected_message = true;
		break;
	case OP_STATE_WAIT_REPORT:
		MHL3_HID_DBG_INFO("OP_STATE_WAIT_REPORT\n");
		if ((msg_id == MHL3_REPORT) || (msg_id == MHL3_DSCRPT_UPDATE))
			matched_expected_message = true;
		break;

	case OP_STATE_CONNECTED:
		switch (msg_id) {
		case MHL3_REPORT:
			if (msg_channel == HID_ACHID_INT) {
				header_len =
					sizeof(struct mhl_hid_report_msg) - 1;
				ret = hid_input_report(
					mhid->hid, HID_INPUT_REPORT,
					pmsg + header_len,
					length - header_len, 1);
				return;
			}
			matched_expected_message = true;
			break;
		case MHL3_DSCRPT_UPDATE:
			MHL3_HID_DBG_ERR(
				"call mhid_add from OP_STATE_CONNECTED\n");
			ret = mhid_add(mdev, dev_id);
			if (ret < 0)
				mhl3_int_send_ack(mdev, ret, hb0);
			return;
			break;
		default:
			if (msg_id != MHL3_HID_ACK)
				matched_expected_message = true;
			break;
		}
		break;
	}

	if (!matched_expected_message && (msg_id != MHL3_HID_ACK)) {
		mhl3_int_send_ack(mdev, HID_ACK_PROTV, hb0);
		return;
	} else if (matched_expected_message && (msg_id == MHL3_DSCRPT_UPDATE)) {

		mhid->opState = OP_STATE_IDLE;
	}

	
	if (down_trylock(&mhid->data_wait_lock)) {
		memcpy(mhid->in_data, pmsg, length);
		mhid->in_data_length = length;
	} else {
		
		MHL3_HID_DBG_ERR("Sending received msg into the ether!!!\n");
	}
	up(&mhid->data_wait_lock);
}

static void validate_ack(struct mhl_dev_context *mdev,
			 uint8_t *pmsg, int length)
{
	int dev_id;
	struct mhl3_hid_data *mhid;

	if (length != HID_ACK_PACKET_LEN)
		return;

	dev_id = (pmsg[1] & HID_HB0_DEV_ID_MSK) >> 4;
	mhid = mdev->mhl_hid[dev_id];
	if (mhid == 0)
		return;

}
void build_received_hid_message(struct mhl_dev_context *mdev,
				uint8_t *pmsg, int length)
{
	int			i, fragment_count = 0;
	uint16_t		*pchksum;
	uint16_t		accum, msg_chksum;
	struct mhl3_hid_global_data *emsc = &mdev->mhl_ghid;

	if (length == 0)
		return;
	switch (emsc->hid_receive_state) {

	case EMSC_RCV_MSG_START:

		if ((pmsg[0] & HID_FRAG_HB0_TYPE) == HID_FRAG_HB0_TYPE_ACK) {
			validate_ack(mdev, pmsg, length);
			return;
		}

		fragment_count = pmsg[0];
		if (length >= (HID_FRAG_HEADER_LEN +
			      HID_MSG_HEADER_LEN +
			      HID_MSG_CHKSUM_LEN + 1)) {
			emsc->hb0 = pmsg[1];
			emsc->hb1 = pmsg[2];
			length -= 3;
			memcpy(&emsc->in_buf[0], &pmsg[3], length);
		}
		emsc->msg_length = length;
		emsc->hid_receive_state = EMSC_RCV_MSG_NEXT;
		break;

	case EMSC_RCV_MSG_NEXT:
		fragment_count = pmsg[0];
		if (length >= (HID_FRAG_HEADER_LEN + 1)) {
			length--;
			memcpy(&emsc->in_buf[emsc->msg_length],
			       &pmsg[1], length);
			emsc->msg_length += length;
		}
		break;
	default:
		
		fragment_count = 1;
		break;
	}

	
	if (fragment_count == 0) {
		emsc->hid_receive_state = EMSC_RCV_MSG_START;
		accum = 0;
		pchksum = (uint16_t *)&emsc->in_buf[0];
		emsc->msg_length -= HID_MSG_CHKSUM_LEN;
		for (i = 0; i < (emsc->msg_length / 2); i++)
			accum += pchksum[i];
		accum += ((uint16_t)emsc->hb0 | (((uint16_t)emsc->hb1) << 8));
		if (emsc->msg_length & 0x01)
			accum += ((uint16_t)emsc->in_buf[emsc->msg_length - 1]);

		
		accum += emsc->msg_length + HID_MSG_HEADER_LEN;

		msg_chksum = *((uint16_t *)&emsc->in_buf[emsc->msg_length]);
		if (accum != msg_chksum) {
			MHL3_HID_DBG_ERR(
				"HID MSG CKSM fail, ignoring message: "
				"Act: %04X Exp: %04X\n",
				accum, msg_chksum);

			
			if (emsc->hb1 & EMSC_HID_HB1_ACK) {
				send_ack_packet(
					mdev, emsc->hb0,
					(emsc->hb1 & EMSC_HID_HB1_MSG_CNT_FLD)
					- 1);
			}
			return;
		}

		hid_message_processor(mdev,
			emsc->hb0, emsc->hb1, emsc->in_buf,
			emsc->msg_length);
	}
}

#ifdef SI_CONFIG_PM_SLEEP	
static int mhl3_hid_suspend(struct device *dev)
{
	return 0;
}

static int mhl3_hid_resume(struct device *dev)
{
	return 0;
}
#endif

