/*
 * DHD debugability header file
 *
 * Copyright (C) 1999-2015, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id$
 */

#ifndef _dhd_debug_h_
#define _dhd_debug_h_
enum {
	DEBUG_RING_ID_INVALID	= 0,
	FW_VERBOSE_RING_ID,
	FW_EVENT_RING_ID,
	DHD_EVENT_RING_ID,
	NAN_EVENT_RING_ID,
	
	DEBUG_RING_ID_MAX
};

enum {
	
	DBG_MEMORY_DUMP_SUPPORTED = (1 << (0)), 
	DBG_PER_PACKET_TX_RX_STATUS_SUPPORTED = (1 << (1)), 
	DBG_CONNECT_EVENT_SUPPORTED = (1 << (2)), 
	DBG_POWER_EVENT_SUPOORTED = (1 << (3)), 
	DBG_WAKE_LOCK_SUPPORTED = (1 << (4)), 
	DBG_VERBOSE_LOG_SUPPORTED = (1 << (5)), 
	DBG_HEALTH_CHECK_SUPPORTED = (1 << (6)), 
	DBG_NAN_EVENT_SUPPORTED = (1 << (7)), 
};

enum {
	
	DBG_RING_ENTRY_FLAGS_HAS_BINARY = (1 << (0)),
	
	DBG_RING_ENTRY_FLAGS_HAS_TIMESTAMP = (1 << (1))
};

#define DBGRING_NAME_MAX		32
#define FW_VERBOSE_RING_NAME		"fw_verbose"
#define FW_VERBOSE_RING_SIZE		(64 * 1024)
#define FW_EVENT_RING_NAME		"fw_event"
#define FW_EVENT_RING_SIZE		(64 * 1024)
#define DHD_EVENT_RING_NAME		"dhd_event"
#define DHD_EVENT_RING_SIZE		(64 * 1024)
#define NAN_EVENT_RING_NAME		"nan_event"
#define NAN_EVENT_RING_SIZE		(64 * 1024)

#define DBG_RING_STATUS_SIZE (sizeof(dhd_dbg_ring_status_t))

#define VALID_RING(id)	\
	(id > DEBUG_RING_ID_INVALID && id < DEBUG_RING_ID_MAX)

enum {
	
	WIFI_EVENT_ASSOCIATION_REQUESTED	= 0,
	WIFI_EVENT_AUTH_COMPLETE,
	WIFI_EVENT_ASSOC_COMPLETE,
	
	WIFI_EVENT_FW_AUTH_STARTED,
	
	WIFI_EVENT_FW_ASSOC_STARTED,
	
	WIFI_EVENT_FW_RE_ASSOC_STARTED,
	WIFI_EVENT_DRIVER_SCAN_REQUESTED,
	WIFI_EVENT_DRIVER_SCAN_RESULT_FOUND,
	WIFI_EVENT_DRIVER_SCAN_COMPLETE,
	WIFI_EVENT_G_SCAN_STARTED,
	WIFI_EVENT_G_SCAN_COMPLETE,
	WIFI_EVENT_DISASSOCIATION_REQUESTED,
	WIFI_EVENT_RE_ASSOCIATION_REQUESTED,
	WIFI_EVENT_ROAM_REQUESTED,
	
	WIFI_EVENT_BEACON_RECEIVED,
	
	WIFI_EVENT_ROAM_SCAN_STARTED,
	
	WIFI_EVENT_ROAM_SCAN_COMPLETE,
	
	WIFI_EVENT_ROAM_SEARCH_STARTED,
	
	WIFI_EVENT_ROAM_SEARCH_STOPPED,
	WIFI_EVENT_UNUSED_0,
	
	WIFI_EVENT_CHANNEL_SWITCH_ANOUNCEMENT,
	
	WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_START,
	
	WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_STOP,
	
	WIFI_EVENT_DRIVER_EAPOL_FRAME_TRANSMIT_REQUESTED,
	WIFI_EVENT_FW_EAPOL_FRAME_RECEIVED,
	WIFI_EVENT_UNUSED_1,
	
	
	WIFI_EVENT_DRIVER_EAPOL_FRAME_RECEIVED,
	
	WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE,
	WIFI_EVENT_BT_COEX_BT_SCO_START,
	WIFI_EVENT_BT_COEX_BT_SCO_STOP,
	
	WIFI_EVENT_BT_COEX_BT_SCAN_START,
	WIFI_EVENT_BT_COEX_BT_SCAN_STOP,
	WIFI_EVENT_BT_COEX_BT_HID_START,
	WIFI_EVENT_BT_COEX_BT_HID_STOP,
	
	WIFI_EVENT_ROAM_AUTH_STARTED,
	
	WIFI_EVENT_ROAM_AUTH_COMPLETE,
	
	WIFI_EVENT_ROAM_ASSOC_STARTED,
	
	WIFI_EVENT_ROAM_ASSOC_COMPLETE
};

enum {
	WIFI_TAG_VENDOR_SPECIFIC = 0, 
	WIFI_TAG_BSSID, 
	WIFI_TAG_ADDR, 
	WIFI_TAG_SSID, 
	WIFI_TAG_STATUS, 
	WIFI_TAG_CHANNEL_SPEC, 
	WIFI_TAG_WAKE_LOCK_EVENT, 
	WIFI_TAG_ADDR1, 
	WIFI_TAG_ADDR2, 
	WIFI_TAG_ADDR3, 
	WIFI_TAG_ADDR4, 
	WIFI_TAG_TSF, 
	WIFI_TAG_IE,
	WIFI_TAG_INTERFACE, 
	WIFI_TAG_REASON_CODE, 
	WIFI_TAG_RATE_MBPS, 
};

typedef enum {
	NAN_EVENT_INVALID = 0,
	NAN_EVENT_CLUSTER_STARTED = 1,
	NAN_EVENT_CLUSTER_JOINED = 2,
	NAN_EVENT_CLUSTER_MERGED = 3,
	NAN_EVENT_ROLE_CHANGED = 4,
	NAN_EVENT_SCAN_COMPLETE = 5,
	NAN_EVENT_STATUS_CHNG  = 6,
	
	NAN_EVENT_MAX
} nan_event_id_t;

typedef struct {
    uint16 tag;
    uint16 len; 
    uint8 *value;
} tlv_log;

typedef struct per_packet_status_entry {
    uint8 flags;
    uint8 tid; 
    uint16 MCS; 
    uint8 rssi;
    uint8 num_retries; 
    uint16 last_transmit_rate; 
	 
    uint16 link_layer_transmit_sequence;
    uint64 firmware_entry_timestamp;
    uint64 start_contention_timestamp;
	uint64 transmit_success_timestamp;
    uint8 *data;
} per_packet_status_entry_t;

typedef struct log_conn_event {
    uint16 event;
    tlv_log *tlvs;
} log_conn_event_t;


typedef struct wake_lock_event {
    uint32 status; 
    uint32 reason; 
    char *name; 
} wake_lock_event_t;

typedef struct wifi_power_event {
    uint16 event;
    tlv_log *tlvs;
} wifi_power_event_t;

#define NAN_EVENT_VERSION 1
typedef struct log_nan_event {
    uint8 version;
    uint8 pad;
    uint16 event;
    tlv_log *tlvs;
} log_nan_event_t;

enum {
	DBG_RING_ENTRY_EVENT_TYPE = 1,
	DBG_RING_ENTRY_PKT_TYPE,
	DBG_RING_ENTRY_WAKE_LOCK_EVENT_TYPE,
	DBG_RING_ENTRY_POWER_EVENT_TYPE,
	DBG_RING_ENTRY_DATA_TYPE,
	DBG_RING_ENTRY_NAN_EVENT_TYPE
};

typedef struct dhd_dbg_ring_entry {
	uint16 len; 
	uint8 flags;
	uint8 type; 
	uint64 timestamp; 
} dhd_dbg_ring_entry_t;

#define DBG_RING_ENTRY_SIZE (sizeof(dhd_dbg_ring_entry_t))
#define ENTRY_LENGTH(hdr) (hdr->len + DBG_RING_ENTRY_SIZE)
typedef struct dhd_dbg_ring_status {
	uint8 name[DBGRING_NAME_MAX];
	uint32 flags;
	int ring_id; 
	
	uint32 ring_buffer_byte_size;
	uint32 verbose_level;
	/* number of bytes that was written to the buffer by driver */
	uint32 written_bytes;
	
	uint32 read_bytes;
	
	uint32 written_records;
} dhd_dbg_ring_status_t;

struct log_level_table {
	int log_level;
	uint16 tag;
	char *desc;
};

typedef void (*dbg_pullreq_t)(void *os_priv, const int ring_id);

typedef void (*dbg_urgent_noti_t) (dhd_pub_t *dhdp, const void *data, const uint32 len);

#if defined(DEBUGABILITY_SUPPORT)
#define DBG_EVENT_LOG(dhd, connect_state) \
{											\
	do {									\
		uint16 state = connect_state;			\
		dhd_os_push_push_ring_data(dhd, DHD_EVENT_RING_ID, &state, sizeof(state)); \
	} while (0);						\
}
#else
#define DBG_EVENT_LOG(dhd, connect_state)
#endif 

extern void dhd_dbg_trace_evnt_handler(dhd_pub_t *dhdp, void *event_data,
		void *raw_event_ptr, uint datalen);
extern int dhd_dbg_attach(dhd_pub_t *dhdp, dbg_pullreq_t os_pullreq,
	dbg_urgent_noti_t os_urgent_notifier, void *os_priv);
extern void dhd_dbg_detach(dhd_pub_t *dhdp);
extern int dhd_dbg_start(dhd_pub_t *dhdp, bool start);
extern int dhd_dbg_set_configuration(dhd_pub_t *dhdp, int ring_id,
		int log_level, int flags, uint32 threshold);
extern int dhd_dbg_get_ring_status(dhd_pub_t *dhdp, int ring_id,
		dhd_dbg_ring_status_t *dbg_ring_status);
extern int dhd_dbg_ring_push(dhd_pub_t *dhdp, int ring_id, dhd_dbg_ring_entry_t *hdr, void *data);
extern int dhd_dbg_ring_pull(dhd_pub_t *dhdp, int ring_id, void *data, uint32 buf_len);
extern int dhd_dbg_find_ring_id(dhd_pub_t *dhdp, char *ring_name);
extern void *dhd_dbg_get_priv(dhd_pub_t *dhdp);
extern int dhd_dbg_send_urgent_evt(dhd_pub_t *dhdp, const void *data, const uint32 len);

extern int dhd_os_dbg_attach(dhd_pub_t *dhdp);
extern void dhd_os_dbg_detach(dhd_pub_t *dhdp);
extern int dhd_os_dbg_register_callback(int ring_id,
	void (*dbg_ring_sub_cb)(void *ctx, const int ring_id, const void *data,
		const uint32 len, const dhd_dbg_ring_status_t dbg_ring_status));
extern int dhd_os_dbg_register_urgent_notifier(dhd_pub_t *dhdp,
	void (*urgent_noti)(void *ctx, const void *data, const uint32 len, const uint32 fw_len));

extern int dhd_os_start_logging(dhd_pub_t *dhdp, char *ring_name, int log_level,
		int flags, int time_intval, int threshold);
extern int dhd_os_reset_logging(dhd_pub_t *dhdp);
extern int dhd_os_suppress_logging(dhd_pub_t *dhdp, bool suppress);

extern int dhd_os_get_ring_status(dhd_pub_t *dhdp, int ring_id,
		dhd_dbg_ring_status_t *dbg_ring_status);
extern int dhd_os_trigger_get_ring_data(dhd_pub_t *dhdp, char *ring_name);
extern int dhd_os_push_push_ring_data(dhd_pub_t *dhdp, int ring_id, void *data, int32 data_len);
extern int dhd_os_dbg_get_feature(dhd_pub_t *dhdp, int32 *features);

#endif 
