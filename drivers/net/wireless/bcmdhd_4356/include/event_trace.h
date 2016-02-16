/*
 * Trace log blocks sent over HBUS
 *
 * Copyright (C) 2015, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id$
 */


#ifndef	_WL_DIAG_H
#define	_WL_DIAG_H

#define DIAG_MAJOR_VERSION      1	
#define DIAG_MINOR_VERSION      0	
#define DIAG_MICRO_VERSION      0	

#define DIAG_VERSION		\
	((DIAG_MICRO_VERSION&0xF) | (DIAG_MINOR_VERSION&0xF)<<4 | \
	(DIAG_MAJOR_VERSION&0xF)<<8)
					
					
					

#define TRACE_FW_AUTH_STARTED			0x8000
#define TRACE_FW_ASSOC_STARTED			0x8001
#define TRACE_FW_RE_ASSOC_STARTED		0x8002
#define TRACE_G_SCAN_STARTED			0x8003
#define TRACE_ROAM_SCAN_STARTED			0x8004
#define TRACE_ROAM_SCAN_COMPLETE		0x8005
#define TRACE_FW_EAPOL_FRAME_TRANSMIT_START	0x8006
#define TRACE_FW_EAPOL_FRAME_TRANSMIT_STOP	0x8007
#define TRACE_BLOCK_ACK_NEGOTIATION_COMPLETE	0x8008	
#define TRACE_BT_COEX_BT_SCO_START		0x8009
#define TRACE_BT_COEX_BT_SCO_STOP		0x800a
#define TRACE_BT_COEX_BT_SCAN_START		0x800b
#define TRACE_BT_COEX_BT_SCAN_STOP		0x800c
#define TRACE_BT_COEX_BT_HID_START		0x800d
#define TRACE_BT_COEX_BT_HID_STOP		0x800e
#define TRACE_ROAM_AUTH_STARTED			0x800f
#define TRACE_NAN_CLUSTER_STARTED               0x9000
#define TRACE_NAN_CLUSTER_JOINED                0x9001
#define TRACE_NAN_CLUSTER_MERGED                0x9002
#define TRACE_NAN_ROLE_CHANGED                  0x9003
#define TRACE_NAN_SCAN_COMPLETE                 0x9004
#define TRACE_NAN_STATUS_CHNG                   0x9005

#define TRACE_TAG_VENDOR_SPECIFIC		0 
#define TRACE_TAG_BSSID				1 
#define TRACE_TAG_ADDR				2 
#define TRACE_TAG_SSID				3 
#define TRACE_TAG_STATUS			4 
#define TRACE_TAG_CHANNEL_SPEC			5 
						  
#define TRACE_TAG_WAKE_LOCK_EVENT		6 
#define TRACE_TAG_ADDR1				7 
#define TRACE_TAG_ADDR2				8 
#define TRACE_TAG_ADDR3				9 
#define TRACE_TAG_ADDR4				10 
#define TRACE_TAG_TSF				11 
#define TRACE_TAG_IE				12 
						   
						   
#define TRACE_TAG_INTERFACE			13 
#define TRACE_TAG_REASON_CODE			14 
						   
#define TRACE_TAG_RATE_MBPS			15 

typedef union {
	struct {
		uint16 event:	16;
		uint16 version:	16;
	};
	uint32 t;
} wl_event_log_id_ver_t;

#ifndef LINUX_POSTMOGRIFY_REMOVAL

#define ETHER_ADDR_PACK_LOW(addr)  (((addr)->octet[3])<<24 | ((addr)->octet[2])<<16 | \
	((addr)->octet[1])<<8 | ((addr)->octet[0]))
#define ETHER_ADDR_PACK_HI(addr)   (((addr)->octet[5])<<8 | ((addr)->octet[4]))
#define SSID_PACK(addr)   (((uint8)(addr)[0])<<24 | ((uint8)(addr)[1])<<16 | \
	((uint8)(addr)[2])<<8 | ((uint8)(addr)[3]))


typedef union {
	struct {
		uint16 status:	16;
		uint16 paraset:	16;
	};
	uint32 t;
} wl_event_log_blk_ack_t;

typedef union {
	struct {
		uint8	mode:	8;
		uint8	count:	8;
		uint16    ch:	16;
	};
	uint32 t;
} wl_event_log_csa_t;

typedef union {
	struct {
		uint8  status:		1;
		uint16 notused:		15;
		uint16 frag_tx_cnt:	16;
	};
	uint32 t;
} wl_event_log_eapol_tx_t;

typedef union {
	struct {
		uint16 tag;
		uint16 length; 
	};
	uint32 t;
} wl_event_log_tlv_hdr_t;

#ifdef WL_EVENT_LOG_COMPILE
extern event_log_top_t *event_log_top;

#define _WL_EVENT_LOG(tag, event, ...) \
	do {					\
		event_log_top->timestamp = OSL_SYSUPTIME(); \
		wl_event_log_id_ver_t entry = {{event, DIAG_VERSION}}; \
		EVENT_LOG(tag, "WL event", entry.t , ## __VA_ARGS__); \
	} while (0)
#define WL_EVENT_LOG(args)	_WL_EVENT_LOG args
#else
#define WL_EVENT_LOG(args)
#endif    
#endif  
#endif	
