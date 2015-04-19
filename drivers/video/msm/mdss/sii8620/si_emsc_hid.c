/*
 * MHL3 HID Tunneling implementation
 *
 * Copyright (c) 2013 Lee Mulcahy <william.mulcahy@siliconimage.com>
 * Copyright (c) 2013 Silicon Image, Inc
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

void dump_array(char *ptitle, uint8_t * pdata, int count)
{
	int i, buf_offset;
	int bufsize = 128;	
	char *buf;
	{
		if (debug_level < 0)
			return;
	}

	if (count > 1) {
		
		bufsize += count * 3;
		
		bufsize += ((count / 16) + 1) * 8;
	}

	buf = kmalloc(bufsize, GFP_KERNEL);
	if (!buf)
		return;

	buf_offset = 0;
	if (ptitle)
		buf_offset =
		    scnprintf(&buf[0], bufsize, "%s (%d bytes):", ptitle,
			      count);
	for (i = 0; i < count; i++) {
		if ((i & 0x0F) == 0)
			buf_offset +=
			    scnprintf(&buf[buf_offset], bufsize - buf_offset,
				      "\n%04X: ", i);
		buf_offset +=
		    scnprintf(&buf[buf_offset], bufsize - buf_offset, "%02X ",
			      pdata[i]);
	}
	buf_offset += scnprintf(&buf[buf_offset], bufsize - buf_offset, "\n");
	MHL_TX_DBG_INFO("%s\n", buf);
	kfree(buf);
}

static int send_hid_msg(struct mhl3_hid_data *mhid, int outlen, bool want_ack)
{
	struct mhl_dev_context *context = mhid->context;
	uint8_t hb0, hb1;
	int ret;

	hb0 = mhid->id << 4;
	hb1 = mhid->ctrl_msg_cnt | (want_ack ? EMSC_HID_HB1_ACK : 0x00);
	mhid->ctrl_msg_cnt =
	    (mhid->ctrl_msg_cnt + 1) & EMSC_HID_HB1_MSG_CNT_FLD;
	ret =
	    si_mhl_tx_emsc_add_hid_message(context, hb0, hb1, mhid->out_data,
					   outlen);
	if (ret == 0)
		si_mhl_tx_push_block_transactions(context);

	return ret;
}

static int send_hid_wait(struct mhl3_hid_data *mhid,
			 int outlen, uint8_t *pin, int inlen, bool want_ack)
{
	struct mhl_dev_context *context = mhid->context;
	int count, ret;

	
	if (!mhid->hid_work_active)
		return -ENODEV;

	if (down_timeout(&context->mhl_emsc.data_wait_lock, 1 * HZ)) {
		MHL_TX_DBG_ERR("Could not acquire data_wait lock !!!\n");
		return -ENODEV;
	}
	MHL_TX_DBG_ERR("Acquired data_wait_lock\n");

	if (down_interruptible(&context->isr_lock)) {
		MHL_TX_DBG_ERR
		    ("Could not acquire isr_lock for HID work queue\n");
		ret = -ENODEV;
		goto done_release;
	}
	MHL_TX_DBG_ERR("Acquired isr_lock\n");
	ret = send_hid_msg(mhid, outlen, want_ack);
	up(&context->isr_lock);
	MHL_TX_DBG_ERR("Released isr_lock, send_hid_msg returned: %d\n", ret);
	if (ret == 0) {
		MHL_TX_DBG_ERR
		    ("send_hid_msg succeeded: mhid->hid_work_active: %d\n",
		     mhid->hid_work_active);

		
		if (!mhid->hid_work_active)
			return -ENODEV;
		
		
		if (down_timeout(&context->mhl_emsc.data_wait_lock, 15 * HZ)) {
			MHL_TX_DBG_ERR
			    ("Timed out waiting for data_wait semaphore!!!\n");
			ret = -EBUSY;

			goto done_release;
		}

		
		if ((mhid->in_data[0] == MHL3_HID_ACK)
		    && (mhid->in_data[1] == HID_ACK_NODEV)) {
			ret = -ENODEV;
			goto done_release;
		}

		
		count =
		    (mhid->in_data_length >
		     inlen) ? inlen : mhid->in_data_length;
		memcpy(pin, mhid->in_data, count);
		ret = count;
	}

done_release:
	up(&context->mhl_emsc.data_wait_lock);
	return ret;
}

static int mhl3_int_send_ack(struct mhl_dev_context *context,
			     int ret, uint8_t hb0, uint hb1)
{
	uint8_t out_data[2];

	out_data[0] = MHL3_HID_ACK;
	out_data[1] = (ret < 0) ? HID_ACK_NODEV : ret;

	ret = si_mhl_tx_emsc_add_hid_message(context, hb0, hb1, out_data, 2);
	if (ret < 0)
		MHL_TX_DBG_ERR("MHID: Failed to send a HID_ACK to device.\n");
	return ret;
}

static int mhl3_send_ack(struct mhl_dev_context *context, uint8_t ret)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;

	mhid->out_data[0] = MHL3_HID_ACK;
	mhid->out_data[1] = (ret < 0) ? HID_ACK_NODEV : ret;

	if (down_interruptible(&context->isr_lock)) {
		MHL_TX_DBG_ERR
		    ("Could not acquire isr_lock for HID work queue\n");
		return -ENODEV;
	}
	ret = send_hid_msg(mhid, 2, false);
	up(&context->isr_lock);
	if (ret < 0)
		hid_err(mhid->hid, "Failed to send a HID_ACK to device.\n");
	return ret;
}

static int mhl3_hid_get_report(struct mhl_dev_context *context, u8 report_type,
			       u8 report_id, unsigned char *pin, int inlen)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	int ret = 0;

	
	mhid->out_data[0] = MHL3_GET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	ret = send_hid_wait(mhid, 3, pin, inlen, false);
	if (ret < 0)
		hid_err(mhid->hid, "failed to retrieve report from device.\n");

	return ret;
}

static int mhl3_hid_set_report(struct mhl_dev_context *context, u8 report_type,
			       u8 report_id, unsigned char *pout, size_t outlen)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	int ret = 0;

	mhid->out_data[0] = MHL3_SET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	memcpy(&mhid->out_data[3], pout, outlen);


	if (down_interruptible(&context->isr_lock)) {
		MHL_TX_DBG_ERR
		    ("Could not acquire isr_lock for HID work queue\n");
		return -ENODEV;
	}
	ret = send_hid_msg(mhid, outlen + 3, false);
	up(&context->isr_lock);
	if (ret) {
		hid_err(mhid->hid, "failed to set a report to device.\n");
		return ret;
	}

	return outlen + 3;	
}

static int mhl3_hid_set_power(struct mhl_dev_context *context, int power_state)
{
	
	int ret = 0;


	return ret;
}

static void mhl3_hid_free_buffers(struct mhl3_hid_data *mhid)
{
	kfree(mhid->hdesc);
	mhid->hdesc = NULL;
	kfree(mhid->in_report_buf);
	mhid->in_report_buf = NULL;
}

static int mhl3_hid_alloc_buffers(struct mhl3_hid_data *mhid,
				  size_t report_size)
{
	mhid->in_report_buf = kzalloc(report_size, GFP_KERNEL);
	if (mhid->in_report_buf == NULL) {
		mhl3_hid_free_buffers(mhid);
		return -ENOMEM;
	}
	mhid->bufsize = report_size;

	return 0;
}

static int mhl3_hid_get_raw_report(struct hid_device *hid,
				   unsigned char report_number, __u8 *buf,
				   size_t count, unsigned char report_type)
{
	struct mhl_dev_context *context = hid->driver_data;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	ret = mhl3_hid_get_report(context,
				  report_type ==
				  HID_FEATURE_REPORT ? 0x03 : 0x01,
				  report_number, buf, count);

	return ret;
}

static int mhl3_hid_output_raw_report(struct hid_device *hid, __u8 * buf,
				      size_t count, unsigned char report_type)
{
	struct mhl_dev_context *context = hid->driver_data;
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	if (report_id) {
		buf++;
		count--;
	}

	ret = mhl3_hid_set_report(context,
				  report_type ==
				  HID_FEATURE_REPORT ? 0x03 : 0x02, report_id,
				  buf, count);

	if (report_id && ret >= 0)
		ret++;	

	return ret;
}

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

static void mhl3_hid_init_report(struct hid_report *report, u8 * buffer,
				 size_t bufsize)
{
	struct hid_device *hid = report->device;
	struct mhl_dev_context *context = hid->driver_data;
	unsigned int size, ret_size;

	size = mhl3_hid_get_report_length(report);
	if (mhl3_hid_get_report(context,
				report->type ==
				HID_FEATURE_REPORT ? 0x03 : 0x01, report->id,
				buffer, size))
		return;

	ret_size = buffer[0] | (buffer[1] << 8);

	if (ret_size != size) {
		hid_err(hid, "error in %s size:%d / ret_size:%d\n",
			__func__, size, ret_size);
		return;
	}

	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}

static void mhl3_hid_init_reports(struct hid_device *hid)
{
	struct mhl_dev_context *context = hid->driver_data;
	struct mhl3_hid_data *mhid = context->mhl_hid;
	struct hid_report *report;

	list_for_each_entry(report,
			    &hid->report_enum[HID_INPUT_REPORT].report_list,
			    list)
	    mhl3_hid_init_report(report, mhid->in_report_buf, mhid->bufsize);

	list_for_each_entry(report,
			    &hid->report_enum[HID_FEATURE_REPORT].report_list,
			    list)
	    mhl3_hid_init_report(report, mhid->in_report_buf, mhid->bufsize);
}

static int mhl3_hid_open(struct hid_device *hid)
{
	struct mhl_dev_context *context = hid->driver_data;
	struct mhl3_hid_data *mhid = context->mhl_hid;
	int ret = 0;

	mutex_lock(&mhl3_hid_open_mutex);
	if (!hid->open++) {
		ret = mhl3_hid_set_power(context, MHL3_HID_PWR_ON);
		if (ret) {
			hid->open--;
			goto done;
		}
		set_bit(MHL3_HID_STARTED, &mhid->flags);
	}
done:
	mutex_unlock(&mhl3_hid_open_mutex);
	return ret;
}

static void mhl3_hid_close(struct hid_device *hid)
{
	struct mhl_dev_context *context = hid->driver_data;
	struct mhl3_hid_data *mhid = context->mhl_hid;

	mutex_lock(&mhl3_hid_open_mutex);
	if (!--hid->open) {
		clear_bit(MHL3_HID_STARTED, &mhid->flags);

		
		mhl3_hid_set_power(context, MHL3_HID_PWR_SLEEP);
	}
	mutex_unlock(&mhl3_hid_open_mutex);
}

static int mhl3_hid_power(struct hid_device *hid, int lvl)
{
	int ret = 0;

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

static int mhl3_hid_fetch_hid_descriptor(struct mhl_dev_context *context)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	struct mhl3_hid_desc *hdesc;
	uint8_t *pdesc_raw;
	int ret = -ENODEV;
	int desc_len, raw_offset;

	
	desc_len = sizeof(struct mhl3_hid_desc) +
	    sizeof(mhid->desc_product_name) +
	    sizeof(mhid->desc_mfg_name) + sizeof(mhid->desc_serial_number);
	pdesc_raw = kmalloc(desc_len, GFP_KERNEL);
	if (!pdesc_raw) {
		MHL_TX_DBG_ERR("Couldn't allocate raw descriptor memory\n");
		return -ENOMEM;
	}
	MHL_TX_DBG_ERR("Fetching the HID descriptor\n");

	
	mhid->out_data[0] = MHL3_GET_MHID_DSCRPT;
	mhid->out_data[1] = 0;	
	mhid->out_data[2] = 0;	
	ret = send_hid_wait(mhid, 3, pdesc_raw, desc_len, false);
	if (ret < 0)
		goto done;

	hdesc = kmalloc(desc_len, GFP_KERNEL);
	if (!hdesc) {
		MHL_TX_DBG_ERR("Couldn't allocate hdesc descriptor memory\n");
		return -ENOMEM;
	}

	MHL_TX_DBG_INFO(
		"send_hid_wait() ret: %d, sizeof( struct mhl3_hid_desc): %ld\n",
		ret, sizeof(struct mhl3_hid_desc));
	dump_array("HID descriptor", pdesc_raw, ret);

	
	memcpy(hdesc, pdesc_raw, sizeof(struct mhl3_hid_desc));
	raw_offset = sizeof(struct mhl3_hid_desc);

	
	if (hdesc->bMHL3HIDmessageID != 0x05) {
		MHL_TX_DBG_ERR("Invalid MHID_DSCRPT data\n");
		ret = -EINVAL;
		goto done_err;
	}

	if (ret < (sizeof(struct mhl3_hid_desc) +
		   hdesc->bProductNameSize +
		   hdesc->bManufacturerNameSize + hdesc->bSerialNumberSize)) {
		
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
	goto done;

done_err:
	kfree(hdesc);
done:
	kfree(pdesc_raw);
	return ret;
}


static struct hid_driver mhl3_hid_driver;

static void mhl3_hid_add_work(struct work_struct *workdata)
{
	struct mhl3_hid_data *mhid =
	    ((struct hid_add_work_struct *)workdata)->mhid;
	struct mhl_dev_context *context = mhid->context;
	struct hid_device *hid;
	uint8_t *report_desc_raw = NULL;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;
	unsigned int report_len;
	int ret;

	MHL_TX_DBG_INFO("%s WORK QUEUE function executing\n", __func__);

	ret = mhl3_hid_fetch_hid_descriptor(context);
	if (ret < 0)
		goto err;

	
	if (mhid->hid == NULL) {
		hid = hid_allocate_device();
		if (IS_ERR(hid)) {
			ret = PTR_ERR(hid);
			goto err;
		}

		mhid->hid = hid;
		hid->driver = &mhl3_hid_driver;
		hid->driver_data = context;
	}
	hid = mhid->hid;

	MHL_TX_DBG_ERR("mhl3_hid_add_work 1- hid->claimed = %02X\n",
		       hid->claimed);

	report_desc_raw = kmalloc(HID_MAX_DESCRIPTOR_SIZE, GFP_KERNEL);
	if (!report_desc_raw) {
		dbg_hid("couldn't allocate raw report descriptor memory\n");
		ret = -ENOMEM;
		goto err;
	}
	mhid->out_data[0] = MHL3_GET_REPORT_DSCRPT;
	ret =
	    send_hid_wait(mhid, 1, report_desc_raw, HID_MAX_DESCRIPTOR_SIZE,
			  false);
	if (ret < 0) {
		hid_err(hid, "failed to retrieve report from device: %d.\n",
			ret);
		goto err;
	}
	dump_array("report_desc_raw", report_desc_raw, ret);
	if (report_desc_raw[0] != MHL3_REPORT_DSCRPT) {
		MHL_TX_DBG_ERR("Invalid response to MHL3_GET_REPORT_DSCRPT\n");
		goto err;
	}
	report_len = report_desc_raw[2];
	report_len <<= 8;
	report_len = report_desc_raw[1];
	report_len = (report_len < (ret - 3)) ? report_len : (ret - 3);
	if (hid_parse_report(hid, &report_desc_raw[3], report_len)) {
		MHL_TX_DBG_ERR("hid_parse_report failed\n");
		goto err;
	}
	MHL_TX_DBG_ERR("mhl3_hid_add_work 2- hid->claimed = %02X\n",
		       hid->claimed);

	hid->ll_driver = &mhl3_hid_ll_driver;
	hid->hid_get_raw_report = mhl3_hid_get_raw_report;
	hid->hid_output_raw_report = mhl3_hid_output_raw_report;
	hid->dev.parent = NULL;	
	hid->bus = BUS_VIRTUAL;	
	hid->version = mhid->hdesc->wBcdHID;
	
	hid->vendor = mhid->hdesc->wHIDVendorID;
	
	hid->product = mhid->hdesc->wHIDProductID;

	snprintf(hid->name, sizeof(hid->name), "MHL3 HID %04hX:%04hX",
		 hid->vendor, hid->product);

	if (!test_bit(MHL3_HID_CONNECTED, &mhid->flags)) {
		
		if (!hidinput_connect(hid, 0)) {
			MHL_TX_DBG_INFO("%s hidinput_connect succeeded\n",
			       __func__);
			hid->claimed |= HID_CLAIMED_INPUT;
			set_bit(MHL3_HID_CONNECTED, &mhid->flags);
		} else {
			MHL_TX_DBG_ERR("%s hidinput_connect FAIL\n", __func__);
			goto err;
		}
	}
	MHL_TX_DBG_ERR("mhl3_hid_add_work 3- hid->claimed = %02X\n",
		       hid->claimed);

	if (mhid->peer_wants_ack)
		mhl3_send_ack(context, HID_ACK_SUCCESS);

	mhl3_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	mhl3_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	mhl3_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);
	if (bufsize > mhid->bufsize) {
		kfree(mhid->in_report_buf);
		mhid->in_report_buf = NULL;
		ret = mhl3_hid_alloc_buffers(mhid, bufsize);
		if (ret)
			goto err;
	}

	if (!(hid->quirks & HID_QUIRK_NO_INIT_REPORTS))
		mhl3_hid_init_reports(hid);
	MHL_TX_DBG_ERR("mhl3_hid_add_work 4- hid->claimed = %02X\n",
		       hid->claimed);

	kfree(report_desc_raw);
	MHL_TX_DBG_INFO("WORK QUEUE function SUCCESS\n");
	mhid->hid_work_active = false;
	return;

err:
	MHL_TX_DBG_ERR("WORK QUEUE function FAIL\n");
	if (mhid->hid) {
		if (mhid->hid->claimed & HID_CLAIMED_INPUT)
			hidinput_disconnect(mhid->hid);
		hid_destroy_device(mhid->hid);
	}

	if (mhid->peer_wants_ack)
		mhl3_send_ack(context, HID_ACK_NODEV);

	if (down_interruptible(&context->isr_lock)) {
		MHL_TX_DBG_ERR("Could not acquire isr_lock for MHID destroy\n");
		return;
	}

	kfree(report_desc_raw);
	mhid->hid_work_active = false;
	mhl3_hid_free_buffers(mhid);
	kfree(mhid);
	context->mhl_hid = 0;

	up(&context->isr_lock);
	return;
}


static int mhl3_hid_update(struct mhl3_hid_data *mhid)
{

	mhid->hid_work_active = true;
	INIT_WORK((struct work_struct *)&mhid->mhl3_work, mhl3_hid_add_work);
	schedule_work((struct work_struct *)&mhid->mhl3_work);
	return 0;
}


static int mhl3_hid_add(struct mhl_dev_context *context, int device_id)
{
	struct mhl3_hid_data *mhid;

	
	if (context->mhl_hid)
		return mhl3_hid_update(context->mhl_hid);

	mhid = kzalloc(sizeof(struct mhl3_hid_data), GFP_KERNEL);
	if (!mhid)
		return -ENOMEM;

	context->mhl_hid = mhid;
	mhid->mhl3_work.mhid = mhid;
	mhid->id = device_id;
	mhid->context = context;

	return mhl3_hid_update(mhid);
}

int mhl3_hid_remove(struct mhl_dev_context *context, int device_id)
{
	struct mhl3_hid_data *mhid;
	struct hid_device *hid;
	int dummy;

	mhid = context->mhl_hid;
	if (mhid == NULL)
		return 0;

	hid = mhid->hid;
	mhid->hid_work_active = false;
	
	dummy = down_trylock(&context->mhl_emsc.data_wait_lock);
	up(&context->mhl_emsc.data_wait_lock);

	if (hid) {
		MHL_TX_DBG_INFO("%s Disconnecting HID device %d\n", __func__,
		       device_id);
		hidinput_disconnect(hid);
		hid_destroy_device(hid);
	}
	
	if (mhid->bufsize)
		mhl3_hid_free_buffers(mhid);

	kfree(mhid);
	context->mhl_hid = NULL;

	return 0;
}

void mhl3_hid_remove_all(struct mhl_dev_context *context)
{
	mhl3_hid_remove(context, 0);
}

void mhl3_hid_message_processor(struct mhl_dev_context *context,
				uint8_t hb0, uint8_t hb1, uint8_t *pmsg,
				int length)
{
	struct mhl3_hid_data *mhid;
	uint8_t msg_id;
	bool want_ack;
	bool is_interrupt_channel;
	int mhl3_device_id, hid_message_count, header_len, ret;

	mhl3_device_id = (hb0 >> 4) & 0x0F;

	is_interrupt_channel = ((hb0 & 0x01) != 0);
	want_ack = ((hb1 & 0x80) != 0);
	hid_message_count = hb1 & 0x7F;
	msg_id = pmsg[0];

	mhid = context->mhl_hid;	

	dump_array("Received message", pmsg, length);

	if ((mhid == 0) && (msg_id != MHL3_DSCRPT_UPDATE)) {
		mhl3_int_send_ack(context, -ENODEV, hb0, hb1);
		return;
	}
	switch (msg_id) {
	case MHL3_REPORT:

		if (is_interrupt_channel) {
			MHL_TX_DBG_INFO("Sending %s report to HID-core\n",
			       mhid->hid->report_enum[HID_INPUT_REPORT].
			       numbered ? "numbered" : "");
			
			header_len =
			    (mhid->hid->report_enum[HID_INPUT_REPORT].
			     numbered) ? 5 : 4;
			ret =
			    hid_input_report(mhid->hid, HID_INPUT_REPORT,
					     pmsg + header_len,
					     length - header_len, 1);
			if ((ret < 0) || (want_ack))
				mhl3_int_send_ack(context, ret, hb0, hb1);
			break;
		}
		
	case MHL3_HID_ACK:
	case MHL3_GET_REPORT_DSCRPT:
	case MHL3_REPORT_DSCRPT:
	case MHL3_GET_MHID_DSCRPT:
	case MHL3_MHID_DSCRPT:
	case MHL3_GET_REPORT:
	case MHL3_SET_REPORT:
		memcpy(mhid->in_data, pmsg, length);
		mhid->in_data_length = length;

		if (down_trylock(&context->mhl_emsc.data_wait_lock)) {
			mhid->peer_wants_ack = want_ack;
		} else {
			
			MHL_TX_DBG_INFO("Sending rcvd msg into the ether!\n");
		}
		up(&context->mhl_emsc.data_wait_lock);
		break;
	case MHL3_DSCRPT_UPDATE:
		ret = mhl3_hid_add(context, mhl3_device_id);
		if (ret < 0)
			mhl3_int_send_ack(context, ret, hb0, hb1);
		break;
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

