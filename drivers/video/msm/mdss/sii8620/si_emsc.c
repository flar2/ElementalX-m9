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

#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#include "si_mdt_inputdev.h"
#include "mhl_linux_tx.h"
#include "platform.h"
#ifdef KERNEL_2_6_38_AND_LATER
#include <linux/input/mt.h>
#endif
#ifdef DEBUG
#include <linux/kernel.h>
#endif

#include "mhl_supp.h"
#include "si_mhl_callback_api.h" 
#include "si_8620_drv.h"

void mhl_tx_hid_host_role_request(struct mhl_dev_context *context, int request)
{
	
	context->mhl_emsc.is_host = false;
	context->mhl_emsc.is_device = false;
	context->mhl_emsc.want_host = false;

	si_mhl_tx_send_msc_msg(context, MHL_MSC_MSG_RHID, request);
}

void mhl_tx_hid_host_negotiation(struct mhl_dev_context *context)
{
	uint8_t rhidk_status = MHL_RHID_NO_ERR;

	if (context->msc_msg_sub_command == MHL_MSC_MSG_RHID) {
		if (context->msc_msg_data == MHL_RHID_REQUEST_HOST) {
			if (context->mhl_emsc.is_host)
				rhidk_status = MHL_RHID_DENY;
		} else if (context->msc_msg_data == MHL_RHID_RELINQUISH_HOST) {
			context->mhl_emsc.want_host = true;
		} else {
			rhidk_status = MHL_RHID_INVALID;
		}

		MHL_TX_DBG_INFO("RHID: Received HID Host role %s result: %d\n",
				(context->msc_msg_data ==
				 MHL_RHID_REQUEST_HOST) ? "REQUEST" :
				"RELINQUISH", rhidk_status);

		
		si_mhl_tx_send_msc_msg(context, MHL_MSC_MSG_RHIDK,
				       rhidk_status);

	} else if (context->msc_msg_sub_command == MHL_MSC_MSG_RHIDK) {
		if (context->msc_msg_data == MHL_RHID_NO_ERR) {
			if (context->msc_msg_last_data ==
				MHL_RHID_REQUEST_HOST) {
				context->mhl_emsc.is_host = true;
				context->mhl_emsc.is_device = false;
				MHL_TX_DBG_INFO
				    ("RHIDK: HID HOST role granted\n");
			} else {
				context->mhl_emsc.is_host = false;
				context->mhl_emsc.is_device = true;
				MHL_TX_DBG_INFO
				    ("RHIDK: HID DEVICE role granted\n");
			}
		} else if (context->msc_msg_last_data ==
			MHL_RHID_REQUEST_HOST) {
			context->mhl_emsc.is_host = false;
			context->mhl_emsc.is_device = true;
			MHL_TX_DBG_INFO("RHIDK: HID HOST role DENIED\n");
		}
	}
}


#define HID_BURST_ID_LEN		(2 + 1)	
#define STD_TRANSPORT_HEADER_LEN	(2)	
#define HID_MSG_HEADER_LEN		(2)	
#define HID_FRAG_HEADER_LEN		(1)
#define HID_MSG_CHKSUM_LEN		(2)	
#define BLOCK_CMD_LEN_MAX		(256)
#define BLOCK_CMD_PAYLOAD_LEN_MAX	(256 - STD_TRANSPORT_HEADER_LEN)

int si_mhl_tx_emsc_add_hid_message(struct mhl_dev_context *context,
				   uint8_t hb0, uint8_t hb1, uint8_t *msg_body,
				   int msg_body_length)
{
	int i, cmd_size, fragment_count, msg_index, payload_index;
	uint8_t *payload;
	uint8_t payload_size;
	uint16_t accum;
	uint16_t *pchksum;
	bool first_fragment;

	dump_array("Submitted message", msg_body, msg_body_length);

	
	cmd_size =
	    STD_TRANSPORT_HEADER_LEN +
	    HID_BURST_ID_LEN +
	    HID_FRAG_HEADER_LEN +
	    HID_MSG_HEADER_LEN + HID_MSG_CHKSUM_LEN + msg_body_length;

	fragment_count = 0;
	
	while (cmd_size > 0) {
		if (cmd_size > BLOCK_CMD_LEN_MAX) {
			fragment_count++;
			cmd_size -= BLOCK_CMD_LEN_MAX;
			if (cmd_size > 0) {
				cmd_size += STD_TRANSPORT_HEADER_LEN +
				    HID_BURST_ID_LEN +
				    HID_FRAG_HEADER_LEN + HID_MSG_CHKSUM_LEN;
			}
		} else {
			cmd_size = 0;
		}
	}

	
	cmd_size =
	    HID_BURST_ID_LEN +
	    HID_FRAG_HEADER_LEN +
	    HID_MSG_HEADER_LEN + HID_MSG_CHKSUM_LEN + msg_body_length;

	
	first_fragment = true;
	msg_index = 0;
	payload_index = 0;
	accum = 0;

	payload_index = 0;
	while (fragment_count >= 0) {

		payload_size = (cmd_size > BLOCK_CMD_PAYLOAD_LEN_MAX) ?
		    BLOCK_CMD_PAYLOAD_LEN_MAX : cmd_size;

		payload =
		    (uint8_t *) si_mhl_tx_get_sub_payload_buffer(context,
								 payload_size);
		if (payload == NULL) {
			MHL_TX_DBG_ERR
			    ("%ssi_mhl_tx_get_sub_payload_buffer failed%s\n",
			     ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		} else {
			payload[payload_index++] =
			    (uint8_t) (burst_id_HID_PAYLOAD >> 8);
			payload[payload_index++] =
			    (uint8_t) (burst_id_HID_PAYLOAD);
			payload[payload_index++] =
			    payload_size - HID_BURST_ID_LEN;
			payload[payload_index++] = fragment_count;
			payload_size -=
			    (HID_BURST_ID_LEN + HID_FRAG_HEADER_LEN);
			if (fragment_count == 0)
				payload_size -= HID_MSG_CHKSUM_LEN;

			if (first_fragment) {
				first_fragment = false;
				payload[payload_index++] = hb0;
				payload[payload_index++] = hb1;
				payload_size -= 2;

				
				accum = ((((uint16_t) hb1) << 8) | hb0);
				pchksum = (uint16_t *) &msg_body[0];
				for (i = 0; i < (msg_body_length / 2); i++)
					accum += pchksum[i];
				if (msg_body_length & 0x01)
					accum +=
					    ((uint16_t)
					     msg_body[msg_body_length - 1]);
				accum = ~(accum - 1);
			}
			memcpy(&payload[payload_index], &msg_body[msg_index],
			       payload_size);
			msg_index += payload_size;
			payload_index += payload_size;

			if (fragment_count == 0) {
				payload[payload_index++] = (uint8_t) accum;
				payload[payload_index++] =
				    (uint8_t) (accum >> 8);
			}
			dump_array("Message fragment", &payload[0],
				   payload_index);
		}
		fragment_count--;
	}
	return 0;
}

static void build_received_hid_message(struct mhl_dev_context *context,
				       uint8_t *pmsg, int length)
{
	int i, fragment_count = 0;
	uint16_t *pchksum;
	uint16_t accum;
	struct mhl3_emsc_data *emsc = &context->mhl_emsc;

	switch (emsc->hid_receive_state) {

	case 0:		

		if (length < 4)	
			return;

		fragment_count = pmsg[0];
		emsc->last_fragment_count = fragment_count;
		emsc->hb0 = pmsg[1];
		emsc->hb1 = pmsg[2];
		length -= 3;
		memcpy(&emsc->in_buffer[0], &pmsg[3], length);
		emsc->msg_length = length;
		emsc->hid_receive_state = 1;
		break;

	case 1:
		if (length < 2)	
			return;

		fragment_count = pmsg[0];
		length--;
		if ((emsc->last_fragment_count - 1) != fragment_count)
			return;	

		emsc->last_fragment_count = fragment_count;
		memcpy(&emsc->in_buffer[emsc->msg_length], &pmsg[1], length);
		emsc->msg_length += length;
		break;
	default:
		
		fragment_count = 1;
		break;
	}

	
	if (fragment_count == 0) {
		dump_array("Received eMSC HID message", &emsc->in_buffer[0],
			   emsc->msg_length);
		accum = 0;
		pchksum = (uint16_t *) &emsc->in_buffer[0];
		emsc->msg_length -= 2;	
		for (i = 0; i < (emsc->msg_length / 2); i++)
			accum += pchksum[i];

		accum += ((uint16_t) emsc->hb0 | (((uint16_t) emsc->hb1) << 8));
		if (emsc->msg_length & 0x01) {
			accum +=
			    ((uint16_t) emsc->in_buffer[emsc->msg_length - 1]);
		}
		
		accum += *((uint16_t *) &emsc->in_buffer[emsc->msg_length]);
		if (accum != 0) {
			
			MHL_TX_DBG_ERR("Zero-sum checksum fail: %04X\n", accum);
		}
		emsc->hid_receive_state = 0;	

		
		mhl3_hid_message_processor(context,
					   emsc->hb0, emsc->hb1,
					   emsc->in_buffer, emsc->msg_length);
	}
}

void si_mhl_tx_emsc_received(struct mhl_dev_context *context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)(&context->drv_context);
	int ret, length, index;
	uint16_t burst_id;
	uint8_t *prbuf, *pmsg;

	ret = si_mhl_tx_drv_peek_block_input_buffer(context, &prbuf, &length);
	
	if (ret == 0) {
		index = 0;
		do {
			struct MHL_burst_id_t *p_burst_id;
			pmsg = &prbuf[index];
			p_burst_id = (struct MHL_burst_id_t *) pmsg;
			burst_id = (((uint16_t) pmsg[0]) << 8) | pmsg[1];
			
			index += sizeof(*p_burst_id);

			switch (burst_id) {
			case burst_id_HID_PAYLOAD:
				build_received_hid_message(context, &pmsg[3],
							   pmsg[2]);
				
				index += (1 + pmsg[2]);
				break;

			case burst_id_BLK_RCV_BUFFER_INFO:

				{
					unsigned long incoming_peer_buffer_size;
					incoming_peer_buffer_size = pmsg[3];
					incoming_peer_buffer_size <<= 8;
					incoming_peer_buffer_size |= pmsg[2];

					hw_context->block_protocol.
					    peer_blk_rx_buffer_avail +=
					    incoming_peer_buffer_size - 256;
				}

				index += 2;
				break;

			case burst_id_BITS_PER_PIXEL_FMT:
				index += (4 + (2 * pmsg[5]));
				
				break;
			case burst_id_HEV_DTDA:{
				struct MHL3_hev_dtd_a_data_t *p_dtda =
				    (struct MHL3_hev_dtd_a_data_t *) p_burst_id;
				index +=
				    sizeof(*p_dtda) -
				    sizeof(*p_burst_id);
				}
				break;
			case burst_id_HEV_DTDB:{
				struct MHL3_hev_dtd_b_data_t *p_dtdb =
				    (struct MHL3_hev_dtd_b_data_t *) p_burst_id;
				index +=
				    sizeof(*p_dtdb) -
				    sizeof(*p_burst_id);
				}
				break;
			case burst_id_HEV_VIC:{
				struct MHL3_hev_vic_data_t *p_burst;
				p_burst =
				    (struct MHL3_hev_vic_data_t *) p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id) +
				    sizeof(p_burst->
				    video_descriptors[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			case burst_id_3D_VIC:{
				struct MHL3_hev_vic_data_t *p_burst =
				    (struct MHL3_hev_vic_data_t *) p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->video_descriptors) +
				    sizeof(p_burst->
					   video_descriptors[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			case burst_id_3D_DTD:{
				struct MHL2_video_format_data_t *p_burst =
				    (struct MHL2_video_format_data_t *)
				    p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->video_descriptors)
				    +
				    sizeof(p_burst->
					   video_descriptors[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			case burst_id_VC_ASSIGN:{
				struct SI_PACK_THIS_STRUCT
				    tdm_alloc_burst * p_burst;
				p_burst =
				    (struct SI_PACK_THIS_STRUCT
				     tdm_alloc_burst*)p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->reserved)
				    - sizeof(p_burst->vc_info)
				    +
				    sizeof(p_burst->vc_info[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			case burst_id_VC_CONFIRM:{
				struct SI_PACK_THIS_STRUCT
				    tdm_alloc_burst * p_burst;
				p_burst =
				    (struct SI_PACK_THIS_STRUCT
				     tdm_alloc_burst*)p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->reserved)
				    - sizeof(p_burst->vc_info) +
				    sizeof(p_burst->vc_info[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			case burst_id_AUD_DELAY:{
				struct MHL3_audio_delay_burst_t
				    *p_burst =
				    (struct MHL3_audio_delay_burst_t *)
				    p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->reserved);
				}
				break;
			case burst_id_ADT_BURSTID:{
				struct MHL3_adt_data_t *p_burst =
				    (struct MHL3_adt_data_t *)
				    p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id);
				}
				break;
			case burst_id_BIST_SETUP:{
				struct SI_PACK_THIS_STRUCT
				    bist_setup_burst * p_bist_setup;
				p_bist_setup =
				    (struct SI_PACK_THIS_STRUCT
				     bist_setup_burst*)p_burst_id;
				index +=
				    sizeof(*p_bist_setup) -
				    sizeof(*p_burst_id);
				}
				break;
			case burst_id_BIST_RETURN_STAT:{
				struct SI_PACK_THIS_STRUCT
				    bist_return_stat_burst
				    *p_bist_return;
				p_bist_return =
				    (struct SI_PACK_THIS_STRUCT
				     bist_return_stat_burst*)
				    p_burst_id;
				index +=
				    sizeof(*p_bist_return) -
				    sizeof(*p_burst_id);
				}
				break;
			case burst_id_EMSC_SUPPORT:{
				struct MHL3_emsc_support_data_t *p_burst =
				    (struct MHL3_emsc_support_data_t *)
				    p_burst_id;
				index +=
				    sizeof(*p_burst) -
				    sizeof(*p_burst_id)
				    - sizeof(p_burst->payload)
				    +
				    sizeof(p_burst->payload.
					   burst_ids[0]) *
				    p_burst->num_entries_this_burst;
				}
				break;
			default:
				if ((MHL_TEST_ADOPTER_ID == burst_id) ||
				    (burst_id >= adopter_id_RANGE_START)) {
					struct EMSC_BLK_ADOPT_ID_PAYLD_HDR
					    *p_adopter_id;
					p_adopter_id =
					    (struct EMSC_BLK_ADOPT_ID_PAYLD_HDR
					     *)pmsg;
					MHL_TX_DBG_ERR
					    ("Unsupported ADOPTER_ID: %04X\n",
					     burst_id);
					index += p_adopter_id->remaining_length;
				} else {
					MHL_TX_DBG_ERR
					    ("Unsupported BURST ID: %04X\n",
					     burst_id);
					index = length;
				}
				break;
			}

			length -= index;
		} while (length > 0);
		si_mhl_tx_drv_free_block_input_buffer(context);
	}
}
