/*
 * EVENT_LOG system definitions
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
 * $Id: event_log.h 241182 2011-02-17 21:50:03Z $
 */

#ifndef _EVENT_LOG_H_
#define _EVENT_LOG_H_

#include <typedefs.h>

#define NUM_EVENT_LOG_SETS 4
#define EVENT_LOG_SET_BUS	0
#define EVENT_LOG_SET_WL	1
#define EVENT_LOG_SET_PSM	2
#define EVENT_LOG_SET_DBG	3

#define EVENT_LOG_TAG_NULL	0	
#define EVENT_LOG_TAG_TS	1	
#define EVENT_LOG_TAG_BUS_OOB	2
#define EVENT_LOG_TAG_BUS_STATE	3
#define EVENT_LOG_TAG_BUS_PROTO	4
#define EVENT_LOG_TAG_BUS_CTL	5
#define EVENT_LOG_TAG_BUS_EVENT	6
#define EVENT_LOG_TAG_BUS_PKT	7
#define EVENT_LOG_TAG_BUS_FRAME	8
#define EVENT_LOG_TAG_BUS_DESC	9
#define EVENT_LOG_TAG_BUS_SETUP	10
#define EVENT_LOG_TAG_BUS_MISC	11
#define EVENT_LOG_TAG_SRSCAN		22
#define EVENT_LOG_TAG_PWRSTATS_INFO	23
#define EVENT_LOG_TAG_UCODE_WATCHDOG 26
#define EVENT_LOG_TAG_UCODE_FIFO 27
#define EVENT_LOG_TAG_SCAN_TRACE_LOW	28
#define EVENT_LOG_TAG_SCAN_TRACE_HIGH	29
#define EVENT_LOG_TAG_SCAN_ERROR	30
#define EVENT_LOG_TAG_SCAN_WARN	31
#define EVENT_LOG_TAG_MPF_ERR	32
#define EVENT_LOG_TAG_MPF_WARN	33
#define EVENT_LOG_TAG_MPF_INFO	34
#define EVENT_LOG_TAG_MPF_DEBUG	35
#define EVENT_LOG_TAG_EVENT_INFO	36
#define EVENT_LOG_TAG_EVENT_ERR	37
#define EVENT_LOG_TAG_PWRSTATS_ERROR	38
#define EVENT_LOG_TAG_EXCESS_PM_ERROR	39
#define EVENT_LOG_TAG_IOCTL_LOG			40
#define EVENT_LOG_TAG_PFN_ERR	41
#define EVENT_LOG_TAG_PFN_WARN	42
#define EVENT_LOG_TAG_PFN_INFO	43
#define EVENT_LOG_TAG_PFN_DEBUG	44
#define EVENT_LOG_TAG_BEACON_LOG	45
#define EVENT_LOG_TAG_WNM_BSSTRANS_INFO 46
#define EVENT_LOG_TAG_TRACE_CHANSW 47
#define EVENT_LOG_TAG_PCI_ERROR	48
#define EVENT_LOG_TAG_PCI_TRACE	49
#define EVENT_LOG_TAG_PCI_WARN	50
#define EVENT_LOG_TAG_PCI_INFO	51
#define EVENT_LOG_TAG_PCI_DBG	52
#define EVENT_LOG_TAG_PCI_DATA  53
#define EVENT_LOG_TAG_PCI_RING	54
#define EVENT_LOG_TAG_WL_ERROR		56
#define EVENT_LOG_TAG_PHY_ERROR		57
#define EVENT_LOG_TAG_OTP_ERROR		58
#define EVENT_LOG_TAG_NOTIF_ERROR	59
#define EVENT_LOG_TAG_MPOOL_ERROR	60
#define EVENT_LOG_TAG_OBJR_ERROR	61
#define EVENT_LOG_TAG_DMA_ERROR		62
#define EVENT_LOG_TAG_PMU_ERROR		63
#define EVENT_LOG_TAG_BSROM_ERROR	64
#define EVENT_LOG_TAG_SI_ERROR		65
#define EVENT_LOG_TAG_ROM_PRINTF	66
#define EVENT_LOG_TAG_RATE_CNT		67
#define EVENT_LOG_TAG_CTL_MGT_CNT	68
#define EVENT_LOG_TAG_AMPDU_DUMP	69
#define EVENT_LOG_TAG_MEM_ALLOC_SUCC	70
#define EVENT_LOG_TAG_MEM_ALLOC_FAIL	71
#define EVENT_LOG_TAG_MEM_FREE		72
#define EVENT_LOG_TAG_WL_ASSOC_LOG	73
#define EVENT_LOG_TAG_WL_PS_LOG		74
#define EVENT_LOG_TAG_WL_ROAM_LOG	75
#define EVENT_LOG_TAG_WL_MPC_LOG	76
#define EVENT_LOG_TAG_WL_WSEC_LOG	77
#define EVENT_LOG_TAG_WL_WSEC_DUMP	78
#define EVENT_LOG_TAG_WL_MCNX_LOG	79
#define EVENT_LOG_TAG_HEALTH_CHECK_ERROR 80
#define EVENT_LOG_TAG_HNDRTE_EVENT_ERROR 81
#define EVENT_LOG_TAG_ECOUNTERS_ERROR	82
#define EVENT_LOG_TAG_WL_COUNTERS	83
#define EVENT_LOG_TAG_ECOUNTERS_IPCSTATS	84
#define EVENT_LOG_TAG_WL_P2P_LOG            85
#define EVENT_LOG_TAG_SDIO_ERROR		86
#define EVENT_LOG_TAG_SDIO_TRACE		87
#define EVENT_LOG_TAG_SDIO_DBG          88
#define EVENT_LOG_TAG_SDIO_PRHDRS		89
#define EVENT_LOG_TAG_SDIO_PRPKT		90
#define EVENT_LOG_TAG_SDIO_INFORM		91
#define EVENT_LOG_TAG_MIMO_PS_ERROR	92
#define EVENT_LOG_TAG_MIMO_PS_TRACE	93
#define EVENT_LOG_TAG_MIMO_PS_INFO	94
#define EVENT_LOG_TAG_BTCX_STATS	95
#define EVENT_LOG_TAG_LEAKY_AP_STATS	96
#define EVENT_LOG_TAG_AWDL_TRACE_ELECTION	97
#define EVENT_LOG_TAG_MIMO_PS_STATS	98
#define EVENT_LOG_TAG_PWRSTATS_PHY	99
#define EVENT_LOG_TAG_PWRSTATS_SCAN	100
#define EVENT_LOG_TAG_PWRSTATS_AWDL	101
#define EVENT_LOG_TAG_PWRSTATS_WAKE_V2	102
#define EVENT_LOG_TAG_LQM		103
#define EVENT_LOG_TAG_TRACE_WL_INFO	104
#define EVENT_LOG_TAG_TRACE_BTCOEX_INFO	105
#define EVENT_LOG_TAG_ECOUNTERS_TIME_DATA	106
#define EVENT_LOG_TAG_NAN_ERROR		107
#define EVENT_LOG_TAG_NAN_INFO		108
#define EVENT_LOG_TAG_NAN_DBG		109
#define EVENT_LOG_TAG_STF_ARBITRATOR_ERROR	110
#define EVENT_LOG_TAG_STF_ARBITRATOR_TRACE	111
#define EVENT_LOG_TAG_STF_ARBITRATOR_WARN	112
#define EVENT_LOG_TAG_MAX		112

#define EVENT_LOG_TAG_FLAG_NONE		0
#define EVENT_LOG_TAG_FLAG_LOG		0x80
#define EVENT_LOG_TAG_FLAG_PRINT	0x40
#define EVENT_LOG_TAG_FLAG_MASK		0x3f

#define LOGSTRS_MAGIC   0x4C4F4753
#define LOGSTRS_VERSION 0x1

#define EVENT_LOG_MAX_BLOCK_SIZE 1400
#define EVENT_LOG_PSM_BLOCK	0x200
#define EVENT_LOG_BUS_BLOCK	0x200
#define EVENT_LOG_DBG_BLOCK	0x100


#ifndef __ASSEMBLER__

#ifdef EVENT_LOG_DUMPER
#define _EL_BLOCK_PTR uint32
#define _EL_TYPE_PTR uint32
#define _EL_SET_PTR uint32
#define _EL_TOP_PTR uint32
#else
#define _EL_BLOCK_PTR struct event_log_block *
#define _EL_TYPE_PTR uint32 *
#define _EL_SET_PTR struct event_log_set **
#define _EL_TOP_PTR struct event_log_top *
#endif 

typedef union event_log_hdr {
	struct {
		uint8 tag;		
		uint8 count;		
		uint16 fmt_num;		
	};
	uint32 t;			
} event_log_hdr_t;

typedef struct event_log_block {
	_EL_BLOCK_PTR next_block;
	_EL_BLOCK_PTR prev_block;
	_EL_TYPE_PTR end_ptr;

	
	uint16 pktlen;			
	uint16 count;			
	uint32 timestamp;		
	uint32 event_logs;
} event_log_block_t;

typedef struct event_log_set {
	_EL_BLOCK_PTR first_block; 	
	_EL_BLOCK_PTR last_block; 	
	_EL_BLOCK_PTR logtrace_block;	
	_EL_BLOCK_PTR cur_block;   	
	_EL_TYPE_PTR cur_ptr;      	
	uint32 blockcount;		
	uint16 logtrace_count;		
	uint16 blockfill_count;		
	uint32 timestamp;		
	uint32 cyclecount;		
} event_log_set_t;

typedef struct event_log_top {
	uint32 magic;
#define EVENT_LOG_TOP_MAGIC 0x474C8669 
	uint32 version;
#define EVENT_LOG_VERSION 1
	uint32 num_sets;
	uint32 logstrs_size;		
	uint32 timestamp;		
	uint32 cyclecount;		
	_EL_SET_PTR sets; 		
} event_log_top_t;

typedef struct {
	uint32 logstrs_size;    
	uint32 rom_lognums_offset; 
	uint32 ram_lognums_offset; 
	uint32 rom_logstrs_offset; 
	uint32 ram_logstrs_offset; 
	
	uint32 version;            
	uint32 log_magic;       
} logstr_header_t;


#ifndef EVENT_LOG_DUMPER

#ifndef EVENT_LOG_COMPILE

#define EVENT_LOG(format, ...)

#else  

#define _EVENT_LOG0(tag, fmt_num) 			\
	event_log0(tag, fmt_num)
#define _EVENT_LOG1(tag, fmt_num, t1) 			\
	event_log1(tag, fmt_num, t1)
#define _EVENT_LOG2(tag, fmt_num, t1, t2) 		\
	event_log2(tag, fmt_num, t1, t2)
#define _EVENT_LOG3(tag, fmt_num, t1, t2, t3) 		\
	event_log3(tag, fmt_num, t1, t2, t3)
#define _EVENT_LOG4(tag, fmt_num, t1, t2, t3, t4) 	\
	event_log4(tag, fmt_num, t1, t2, t3, t4)

#define _EVENT_LOG5(tag, fmt_num, ...) event_logn(5, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOG6(tag, fmt_num, ...) event_logn(6, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOG7(tag, fmt_num, ...) event_logn(7, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOG8(tag, fmt_num, ...) event_logn(8, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOG9(tag, fmt_num, ...) event_logn(9, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGA(tag, fmt_num, ...) event_logn(10, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGB(tag, fmt_num, ...) event_logn(11, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGC(tag, fmt_num, ...) event_logn(12, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGD(tag, fmt_num, ...) event_logn(13, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGE(tag, fmt_num, ...) event_logn(14, tag, fmt_num, __VA_ARGS__)
#define _EVENT_LOGF(tag, fmt_num, ...) event_logn(15, tag, fmt_num, __VA_ARGS__)


#define _EVENT_LOG_VA_NUM_ARGS(F, _1, _2, _3, _4, _5, _6, _7, _8, _9,	\
			       _A, _B, _C, _D, _E, _F, N, ...) F ## N

#define _EVENT_LOG(tag, fmt, ...)					\
	static char logstr[] __attribute__ ((section(".logstrs"))) = fmt; \
	static uint32 fmtnum __attribute__ ((section(".lognums"))) = (uint32) &logstr; \
	_EVENT_LOG_VA_NUM_ARGS(_EVENT_LOG, ##__VA_ARGS__,		\
			       F, E, D, C, B, A, 9, 8,			\
			       7, 6, 5, 4, 3, 2, 1, 0)			\
	(tag, (int) &fmtnum , ## __VA_ARGS__);				\


#define EVENT_LOG_FAST(tag, fmt, ...)					\
	if (event_log_tag_sets != NULL) {				\
		uint8 tag_flag = *(event_log_tag_sets + tag);		\
		if (tag_flag != 0) {					\
			_EVENT_LOG(tag, fmt , ## __VA_ARGS__);		\
		}							\
	}

#define EVENT_LOG_COMPACT(tag, fmt, ...)				\
	if (1) {							\
		_EVENT_LOG(tag, fmt , ## __VA_ARGS__);			\
	}

#define EVENT_LOG(tag, fmt, ...) EVENT_LOG_COMPACT(tag, fmt , ## __VA_ARGS__)

#define EVENT_LOG_IS_LOG_ON(tag) (*(event_log_tag_sets + (tag)) & EVENT_LOG_TAG_FLAG_LOG)

#define EVENT_DUMP	event_log_buffer

extern uint8 *event_log_tag_sets;

#include <siutils.h>

extern int event_log_init(si_t *sih);
extern int event_log_set_init(si_t *sih, int set_num, int size);
extern int event_log_set_expand(si_t *sih, int set_num, int size);
extern int event_log_set_shrink(si_t *sih, int set_num, int size);
extern int event_log_tag_start(int tag, int set_num, int flags);
extern int event_log_tag_stop(int tag);
extern int event_log_get(int set_num, int buflen, void *buf);
extern uint8 * event_log_next_logtrace(int set_num);

extern void event_log0(int tag, int fmtNum);
extern void event_log1(int tag, int fmtNum, uint32 t1);
extern void event_log2(int tag, int fmtNum, uint32 t1, uint32 t2);
extern void event_log3(int tag, int fmtNum, uint32 t1, uint32 t2, uint32 t3);
extern void event_log4(int tag, int fmtNum, uint32 t1, uint32 t2, uint32 t3, uint32 t4);
extern void event_logn(int num_args, int tag, int fmtNum, ...);

extern void event_log_time_sync(void);
extern void event_log_buffer(int tag, uint8 *buf, int size);

#endif 

#endif 

#endif 

#endif 
