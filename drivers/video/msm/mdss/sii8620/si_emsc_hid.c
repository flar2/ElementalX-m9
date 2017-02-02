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

/* debug option */
void dump_array(char *ptitle, uint8_t * pdata, int count)
{
	int i, buf_offset;
	int bufsize = 128;	/* Give a little fudge room. */
	char *buf;
	{
		if (debug_level < 0)
			return;
	}

	if (count > 1) {
		/* 3 chars per byte displayed */
		bufsize += count * 3;
		/* plus per display row overhead */
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

/*
 * Build a HID tunneling message and send it.
 * Returns non-zero if an error occurred, such as the device was
 * disconnected.
 *
 * This function should be called wrapped in an isr_lock semaphore pair UNLESS
 * it is being called from the Titan interrupt handler.
 */
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

/*
 * Send an MHL3 HID Command and wait for a response.
 * called only from a WORK QUEUE function; Do NOT call from
 * the Titan interrupt context.
 * Returns negative if an error occurred, such as the device was
 * disconnected, otherwise returns the number of bytes
 * received.
 */
static int send_hid_wait(struct mhl3_hid_data *mhid,
			 int outlen, uint8_t *pin, int inlen, bool want_ack)
{
	struct mhl_dev_context *context = mhid->context;
	int count, ret;

	/* If work is not active, we're being canceled. */
	if (!mhid->hid_work_active)
		return -ENODEV;

	/* Take a hold of the wait lock.  It will be released
	 * when the response is received. This down call will be
	 * released by an up call in the mhl3_hid_message_processor()
	 * function when the response message has been received. Not
	 * conventional, but until I think of a better way,
	 * this is it. */
	if (down_timeout(&context->mhl_emsc.data_wait_lock, 1 * HZ)) {
		MHL_TX_DBG_ERR("Could not acquire data_wait lock !!!\n");
		return -ENODEV;
	}
	MHL_TX_DBG_ERR("Acquired data_wait_lock\n");

	/* Send the message when the Titan ISR is not active so
	 * that we don't deadlock with eMsc block buffer allocation. */
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

		/* If work is not active, we're being canceled. */
		if (!mhid->hid_work_active)
			return -ENODEV;
		/* Wait until a buffer is ready (when the driver unblocks). */
		/* TODO: Reduce this timeout value */
		if (down_timeout(&context->mhl_emsc.data_wait_lock, 15 * HZ)) {
			MHL_TX_DBG_ERR
			    ("Timed out waiting for data_wait semaphore!!!\n");
			ret = -EBUSY;

			/* As odd as it may seem, we must release the semaphore
			 * that we were waiting for because something
			 * apparently has gone wrong with the communications
			 * protocol and we need to start over.
			 */
			goto done_release;
		}

		/* Intercept HID_ACK{NO_DEV} messages. */
		if ((mhid->in_data[0] == MHL3_HID_ACK)
		    && (mhid->in_data[1] == HID_ACK_NODEV)) {
			ret = -ENODEV;
			goto done_release;
		}

		/* Message has been placed in our input buffer. */
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

/*
 * Send a HID_ACK message with the passed error code from the interrupt
 * context without using the mhid device structure (in case it's not there).
 */
static int mhl3_int_send_ack(struct mhl_dev_context *context,
			     int ret, uint8_t hb0, uint hb1)
{
	uint8_t out_data[2];

	out_data[0] = MHL3_HID_ACK;
	out_data[1] = (ret < 0) ? HID_ACK_NODEV : ret;

	ret = si_mhl_tx_emsc_add_hid_message(context, hb0, hb1, out_data, 2);
	/*
	if ( ret == 0 ) {
		si_mhl_tx_push_block_transactions(context);
	}
	*/
	if (ret < 0)
		MHL_TX_DBG_ERR("MHID: Failed to send a HID_ACK to device.\n");
	return ret;
}

/*
 * Send a HID_ACK message with the passed error code.
 */
static int mhl3_send_ack(struct mhl_dev_context *context, uint8_t ret)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;

	mhid->out_data[0] = MHL3_HID_ACK;
	mhid->out_data[1] = (ret < 0) ? HID_ACK_NODEV : ret;

	/* Send the message when the Titan ISR is not active so
	 * that we don't deadlock with eMsc block buffer allocation. */
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

/*
 * Called from mhl3_hid_get_raw_report() and mhl3_hid_init_report()
 * to get report data from the device.
 *
 * It can not be called from the Titan driver interrupt context, because
 * it blocks until it gets a response FROM the Titan interrupt driver.
 * until a reply is given.
 */
static int mhl3_hid_get_report(struct mhl_dev_context *context, u8 report_type,
			       u8 report_id, unsigned char *pin, int inlen)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	int ret = 0;

	/* Build MHL3_GET_REPORT message and add it to the out queue. */
	mhid->out_data[0] = MHL3_GET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	ret = send_hid_wait(mhid, 3, pin, inlen, false);
	if (ret < 0)
		hid_err(mhid->hid, "failed to retrieve report from device.\n");

	return ret;
}

/*
 * Called from mhl3_hid_output_raw_report() to send report data to
 * the device.
 */
static int mhl3_hid_set_report(struct mhl_dev_context *context, u8 report_type,
			       u8 report_id, unsigned char *pout, size_t outlen)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	int ret = 0;

	mhid->out_data[0] = MHL3_SET_REPORT;
	mhid->out_data[1] = report_type;
	mhid->out_data[2] = report_id;
	memcpy(&mhid->out_data[3], pout, outlen);

	/* TODO: Need to make sure that this function is never called from
	 *       the Titan ISR (through linkage from a HID function called
	 *       from a HID report response or some such).  Just needs some
	 *       research....
	 */

	/* Send the message when the Titan ISR is not active so
	 * that we don't deadlock with eMsc block buffer allocation. */
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

	return outlen + 3;	/* TODO: Lee - Should the 3 be added? */
}

/*
 * TODO: Lee - Doesn't do anything yet
 */
static int mhl3_hid_set_power(struct mhl_dev_context *context, int power_state)
{
	/* struct mhl3_hid_data *mhid = context->mhl_hid; */
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

/*
 * Called from the HID driver to obtain raw report data from the
 * device.
 *
 * It is not called from an interrupt context, so it is OK to block
 * until a reply is given.
 */
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

/*
 * Called from the HID driver to send report data to the device.
 */
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
		ret++;	/* add report_id to the number of transfered bytes */

	return ret;
}

static int mhl3_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
	    report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void mhl3_hid_find_max_report(struct hid_device *hid, unsigned int type,
				     unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = mhl3_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

/*
 *
 */
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

	/* hid->driver_lock is held as we are in probe function,
	 * we just need to setup the input fields, so using
	 * hid_report_raw_event is safe. */
	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}

/*
 * Initialize all reports.  This gets the current value of all
 * input/feature reports for the device so that the HID-core can keep
 * them in internal structures.  The structure is updated as further
 * device reports occur.
 */
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

/*
 * TODO: Lee - Doesn't do anything yet except manage the open count
 */
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

/*
 * TODO: Lee - Doesn't do anything yet except manage the open count
 */
static void mhl3_hid_close(struct hid_device *hid)
{
	struct mhl_dev_context *context = hid->driver_data;
	struct mhl3_hid_data *mhid = context->mhl_hid;

	/* protecting hid->open to make sure we don't restart
	 * data acquisition due to a resumption we no longer
	 * care about
	 */
	mutex_lock(&mhl3_hid_open_mutex);
	if (!--hid->open) {
		clear_bit(MHL3_HID_STARTED, &mhid->flags);

		/* Save some power */
		mhl3_hid_set_power(context, MHL3_HID_PWR_SLEEP);
	}
	mutex_unlock(&mhl3_hid_open_mutex);
}

/*
 * TODO: Lee - Doesn't do anything yet
 */
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

/*
* Use the MHL3 GET_MHID_DSCRPT message to request the HID Device
* Descriptor from the device on the control channel. The device
* should reply with an MHL3_MHID_DSCRPT message
*
* This function MUST be called from a work queue function.
*
*/
static int mhl3_hid_fetch_hid_descriptor(struct mhl_dev_context *context)
{
	struct mhl3_hid_data *mhid = context->mhl_hid;
	struct mhl3_hid_desc *hdesc;
	uint8_t *pdesc_raw;
	int ret = -ENODEV;
	int desc_len, raw_offset;

	/* The actual length of the data will likely not be this much. */
	desc_len = sizeof(struct mhl3_hid_desc) +
	    sizeof(mhid->desc_product_name) +
	    sizeof(mhid->desc_mfg_name) + sizeof(mhid->desc_serial_number);
	pdesc_raw = kmalloc(desc_len, GFP_KERNEL);
	if (!pdesc_raw) {
		MHL_TX_DBG_ERR("Couldn't allocate raw descriptor memory\n");
		return -ENOMEM;
	}
	MHL_TX_DBG_ERR("Fetching the HID descriptor\n");

	/* Build GET_MHID_DSCRPT message and add it to the out queue. */
	mhid->out_data[0] = MHL3_GET_MHID_DSCRPT;
	mhid->out_data[1] = 0;	/* Language ID low byte */
	mhid->out_data[2] = 0;	/* Language ID high byte        */
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

	/* Get the fixed length part and verify.        */
	memcpy(hdesc, pdesc_raw, sizeof(struct mhl3_hid_desc));
	raw_offset = sizeof(struct mhl3_hid_desc);

	/* Do some simple checks.       */
	if (hdesc->bMHL3HIDmessageID != 0x05) {
		MHL_TX_DBG_ERR("Invalid MHID_DSCRPT data\n");
		ret = -EINVAL;
		goto done_err;
	}

	if (ret < (sizeof(struct mhl3_hid_desc) +
		   hdesc->bProductNameSize +
		   hdesc->bManufacturerNameSize + hdesc->bSerialNumberSize)) {
		/* TODO: Could kmalloc these... */
		memcpy(mhid->desc_product_name,
		       &pdesc_raw[raw_offset], hdesc->bProductNameSize);
		raw_offset += hdesc->bProductNameSize;
		memcpy(mhid->desc_mfg_name,
		       &pdesc_raw[raw_offset], hdesc->bManufacturerNameSize);
		raw_offset += hdesc->bManufacturerNameSize;
		memcpy(mhid->desc_serial_number,
		       &pdesc_raw[raw_offset], hdesc->bSerialNumberSize);
	}

	/* If this was an existing mhid being updated, free the previous
	 * descriptor and reassign with the new one.
	 */
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

/* TODO: Lee - Eliminated ACPI stuff; i2c_hid_acpi_pdata. etc. */

/*
 * The users of the hid_device structure don't always check that
 * the hid_device structure has a valid hid_driver before trying to access
 * its members. Creating an empty structure at least avoids a
 * kernel panic.
 */
static struct hid_driver mhl3_hid_driver;

/*
 * The second part of the mhl3_hid_add() function, implemented as
 * a work queue.
 * A failure here deletes the mhid device.
 */
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

	/* Get a HID device if this is not an update of the existing HID. */
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

	/* Get report descriptors from the device and parse them.
	 * The Report Descriptor describes the report ID and the type(s)
	 * of data it contains. During operation, when the device returns
	 * a report with a specific ReportID, the HID core will know how
	 * to parse it. */
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
	hid->dev.parent = NULL;	/*TODO: Lee - Don't know if this is right */
	hid->bus = BUS_VIRTUAL;	/*TODO: Lee - Don't know if this is right */
	hid->version = mhid->hdesc->wBcdHID;
	/* le16_to_cpu(mhid->hdesc.wHIDVendorID); */
	hid->vendor = mhid->hdesc->wHIDVendorID;
	/* le16_to_cpu(mhid->hdesc.wHIDProductID); */
	hid->product = mhid->hdesc->wHIDProductID;

	snprintf(hid->name, sizeof(hid->name), "MHL3 HID %04hX:%04hX",
		 hid->vendor, hid->product);

	/*TODO: Don't know why this didn't work:
	 * if ( hid->claimed == 0){
	 */
	if (!test_bit(MHL3_HID_CONNECTED, &mhid->flags)) {
		/*TODO: Lee - What if this call fails? */
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

	/* Allocate some report buffers and read the initial state of
	 * the INPUT and FEATURE reports.
	 */
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

	/* Don't destroy the mhid while an interrupt is in progress on the
	 * off chance that the message we were waiting for came in after the
	 * timeout.*/
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

/*
 * Obtain the HID Descriptor and Report descriptors for the specified
 * device.  It is called when the MHL transport receives the
 * MHL3_DSCRPT_UPDATE message.
 *
 * Called from a Titan interrupt handler. All events in the MHL driver
 * are handled in interrupt handlers, so the real work is performed on
 * a work queue so the function can wait for responses from the driver.
 *
 * The device_id parameter is for when we support multiple HID devices.
 */

static int mhl3_hid_update(struct mhl3_hid_data *mhid)
{

	mhid->hid_work_active = true;
	INIT_WORK((struct work_struct *)&mhid->mhl3_work, mhl3_hid_add_work);
	schedule_work((struct work_struct *)&mhid->mhl3_work);
	return 0;
}

/*
 * Create and initialize the device control structures and get added to
 * the system HID device list.  This is the equivalent to the probe
 * function of normal HID-type drivers, but it is called when the
 * MHL transport receives the eMSC Block transfer HID Tunneling request
 * message MHL3_DSCRPT_UPDATE
 *
 * Called from a Titan interrupt handler. All events in the MHL driver
 * are handled in interrupt handlers, so the real work is performed on
 * a work queue so the function can wait for responses from the driver.
 */

static int mhl3_hid_add(struct mhl_dev_context *context, int device_id)
{
	struct mhl3_hid_data *mhid;

	/* TODO: Implement multiple mhid */
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

/*
 * This is the reverse of the mhl3_hid_add() function.
 * The device_id parameter is for when we support multiple HID devices.
 */
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
	/* Release the data wait to let the work queue finish */
	dummy = down_trylock(&context->mhl_emsc.data_wait_lock);
	up(&context->mhl_emsc.data_wait_lock);

	if (hid) {
		MHL_TX_DBG_INFO("%s Disconnecting HID device %d\n", __func__,
		       device_id);
		hidinput_disconnect(hid);
		hid_destroy_device(hid);
	}
	/* TODO: Do we need to do an isr_lock here? */
	if (mhid->bufsize)
		mhl3_hid_free_buffers(mhid);

	kfree(mhid);
	context->mhl_hid = NULL;

	return 0;
}

/*
 * For when we support multiple HID devices.
 */
void mhl3_hid_remove_all(struct mhl_dev_context *context)
{
	mhl3_hid_remove(context, 0);
}

/*
 * We have received a completed MHL3 HID Tunneling message.  Parse it
 * to decide where it should go next.
 * Called from the Titan interrupt handler.
 */
void mhl3_hid_message_processor(struct mhl_dev_context *context,
				uint8_t hb0, uint8_t hb1, uint8_t *pmsg,
				int length)
{
	struct mhl3_hid_data *mhid;
	uint8_t msg_id;
	bool want_ack;
	bool is_interrupt_channel;
	int mhl3_device_id, hid_message_count, header_len, ret;

	/* In the future use this to select an appropriate mhid.
	 * For now we only allow one remote HID device.
	 */
	mhl3_device_id = (hb0 >> 4) & 0x0F;

	is_interrupt_channel = ((hb0 & 0x01) != 0);
	want_ack = ((hb1 & 0x80) != 0);
	hid_message_count = hb1 & 0x7F;
	msg_id = pmsg[0];

	mhid = context->mhl_hid;	/* TODO: Convert to multiple device */

	dump_array("Received message", pmsg, length);

	/* Make sure we didn't lose the device, possibly by taking too
	 * long to return this message. MHL3_DSCRPT_UPDATE is the only
	 * message that can come without an allocated mhid. */
	if ((mhid == 0) && (msg_id != MHL3_DSCRPT_UPDATE)) {
		mhl3_int_send_ack(context, -ENODEV, hb0, hb1);
		return;
	}
	switch (msg_id) {
	case MHL3_REPORT:

		/* If sent by the interrupt channel, the message is NOT from a
		 * MHL3_GET_REPORT request by the host, and we send it directly
		 * to the HID core.  Otherwise, we return it via the WORK QUEUE.
		 */
		if (is_interrupt_channel) {
			MHL_TX_DBG_INFO("Sending %s report to HID-core\n",
			       mhid->hid->report_enum[HID_INPUT_REPORT].
			       numbered ? "numbered" : "");
			/* Send the report directly to the HID core.    */
			header_len =
			    (mhid->hid->report_enum[HID_INPUT_REPORT].
			     numbered) ? 5 : 4;
			/* Skip MSG_ID, REPORT_TYPE, REPORT_ID (if present),
			 * and LENGTH (2 bytes)
			 */
			ret =
			    hid_input_report(mhid->hid, HID_INPUT_REPORT,
					     pmsg + header_len,
					     length - header_len, 1);
			if ((ret < 0) || (want_ack))
				mhl3_int_send_ack(context, ret, hb0, hb1);
			break;
		}
		/* Fall through to return message to WORK QUEUE. */
	case MHL3_HID_ACK:
	case MHL3_GET_REPORT_DSCRPT:
	case MHL3_REPORT_DSCRPT:
	case MHL3_GET_MHID_DSCRPT:
	case MHL3_MHID_DSCRPT:
	case MHL3_GET_REPORT:
	case MHL3_SET_REPORT:
		memcpy(mhid->in_data, pmsg, length);
		mhid->in_data_length = length;

		/* If someone was waiting for this data, let them know it's
		 * here.
		 */
		if (down_trylock(&context->mhl_emsc.data_wait_lock)) {
			mhid->peer_wants_ack = want_ack;
		} else {
			/*TODO: If not on a WORK QUEUE, where does it go?*/
			MHL_TX_DBG_INFO("Sending rcvd msg into the ether!\n");
		}
		/* Either the try was successful, in which case no one was
		 * waiting for the message, or it wasn't, meaning someone WAS
		 * waiting. In either case we need to release the semaphore.
		 */
		up(&context->mhl_emsc.data_wait_lock);
		break;
	case MHL3_DSCRPT_UPDATE:
		ret = mhl3_hid_add(context, mhl3_device_id);
		if (ret < 0)
			mhl3_int_send_ack(context, ret, hb0, hb1);
		break;
	}
}

#ifdef SI_CONFIG_PM_SLEEP	/* this was originally CONFIG_PM_SLEEP,
				   but we don't want the compiler warnings
				 */
/*
 * TODO: Lee - This is used during system suspend and hibernation as well
 * as normal runtime PM.  Needs work.
 */
static int mhl3_hid_suspend(struct device *dev)
{
	return 0;
}

/*
 * TODO: Lee - This is used during system suspend and hibernation as well
 * as normal runtime PM.  Needs work.
 */
static int mhl3_hid_resume(struct device *dev)
{
	return 0;
}
#endif

