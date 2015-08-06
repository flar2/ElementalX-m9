/*
 * Header file describing the internal (inter-module) DHD interfaces.
 *
 * Provides type definitions and function prototypes used to link the
 * DHD OS, bus, and protocol modules.
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
 * $Id: dhd.h 556045 2015-05-12 08:30:49Z $
 */


#ifndef _dhd_h_
#define _dhd_h_

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/ethtool.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif 
struct task_struct;
struct sched_param;
int setScheduler(struct task_struct *p, int policy, struct sched_param *param);
int get_scheduler_policy(struct task_struct *p);
#define MAX_EVENT	16

#define ALL_INTERFACES	0xff

#include <wlioctl.h>
#include <wlfc_proto.h>

#if defined(BCMWDF)
#include <wdf.h>
#include <WdfMiniport.h>
#endif 

#if defined(WL11U) && !defined(MFP)
#define MFP 
#endif 

#if defined(KEEP_ALIVE)
#define KEEP_ALIVE_PERIOD 55000
#define NULL_PKT_STR	"null_pkt"
#endif 
struct dhd_bus;
struct dhd_prot;
struct dhd_info;
struct dhd_ioctl;

enum dhd_bus_state {
	DHD_BUS_DOWN,		
	DHD_BUS_LOAD,		
	DHD_BUS_DATA,		
	DHD_BUS_SUSPEND,	
};

#if defined(NDISVER)
#if (NDISVER >= 0x0600)
#define STA_MASK			0x0001
#define HOSTAPD_MASK		0x0002
#define WFD_MASK			0x0004
#define SOFTAP_FW_MASK	0x0008
#define P2P_GO_ENABLED		0x0010
#define P2P_GC_ENABLED		0x0020
#define CONCURENT_MASK		0x00F0
#endif 
#endif 

#define DHD_IF_ROLE_STA(role)	(role == WLC_E_IF_ROLE_STA ||\
				role == WLC_E_IF_ROLE_P2P_CLIENT)

#define DHD_MAX_IFS	16
#define DHD_DEL_IF	-0xE
#define DHD_BAD_IF	-0xF

enum dhd_op_flags {
	DHD_FLAG_STA_MODE				= (1 << (0)), 
	DHD_FLAG_HOSTAP_MODE				= (1 << (1)), 
	DHD_FLAG_P2P_MODE				= (1 << (2)), 
	
	DHD_FLAG_CONCURR_SINGLE_CHAN_MODE = (DHD_FLAG_STA_MODE | DHD_FLAG_P2P_MODE),
	DHD_FLAG_CONCURR_MULTI_CHAN_MODE		= (1 << (4)), 
	
	DHD_FLAG_P2P_GC_MODE				= (1 << (5)),
	DHD_FLAG_P2P_GO_MODE				= (1 << (6)),
	DHD_FLAG_MBSS_MODE				= (1 << (7)), 
	DHD_FLAG_IBSS_MODE				= (1 << (8)),
	DHD_FLAG_MFG_MODE				= (1 << (9))
};

#ifndef MAX_CNTL_TX_TIMEOUT
#define MAX_CNTL_TX_TIMEOUT 2
#endif 
#ifndef MAX_CNTL_RX_TIMEOUT
#define MAX_CNTL_RX_TIMEOUT 1
#endif 

#ifdef BCMPCIE
#ifndef MAX_CNTL_D3ACK_TIMEOUT
#define MAX_CNTL_D3ACK_TIMEOUT 1
#endif 
#endif 

#define DHD_SCAN_ASSOC_ACTIVE_TIME	40 
#define DHD_SCAN_UNASSOC_ACTIVE_TIME 80 
#ifdef CUSTOMER_HW_ONE
#define DHD_SCAN_PASSIVE_TIME		100 
#else
#define DHD_SCAN_PASSIVE_TIME		130 
#endif

#ifndef POWERUP_MAX_RETRY
#define POWERUP_MAX_RETRY	3 
#endif
#ifndef POWERUP_WAIT_MS
#define POWERUP_WAIT_MS		2000 
#endif

enum dhd_bus_wake_state {
	WAKE_LOCK_OFF,
	WAKE_LOCK_PRIV,
	WAKE_LOCK_DPC,
	WAKE_LOCK_IOCTL,
	WAKE_LOCK_DOWNLOAD,
	WAKE_LOCK_TMOUT,
	WAKE_LOCK_WATCHDOG,
	WAKE_LOCK_LINK_DOWN_TMOUT,
	WAKE_LOCK_PNO_FIND_TMOUT,
	WAKE_LOCK_SOFTAP_SET,
	WAKE_LOCK_SOFTAP_STOP,
	WAKE_LOCK_SOFTAP_START,
	WAKE_LOCK_SOFTAP_THREAD
};

enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0,
	DHD_PREALLOC_RXBUF,
	DHD_PREALLOC_DATABUF,
	DHD_PREALLOC_OSL_BUF,
#if defined(STATIC_WL_PRIV_STRUCT)
	DHD_PREALLOC_WIPHY_ESCAN0 = 5,
#endif 
	DHD_PREALLOC_DHD_INFO = 7,
	DHD_PREALLOC_DHD_WLFC_INFO = 8,
	DHD_PREALLOC_IF_FLOW_LKUP = 9,
	DHD_PREALLOC_MEMDUMP_BUF = 10,
	DHD_PREALLOC_MEMDUMP_RAM = 11
};

#if defined(USE_STATIC_MEMDUMP)
enum dhd_dongledump_mode {
	DUMP_DISABLED = 0,
	DUMP_MEMONLY,
	DUMP_MEMFILE,
	DUMP_MEMFILE_BUGON,
#ifdef CUSTOMER_HW_ONE
	DUMP_MEMFILE_USR_TRIGGER,
	DUMP_MEMFILE_HANG
#endif 
};
#endif 

#ifndef DHD_SDALIGN
#ifdef CUSTOMER_HW_ONE
#define DHD_SDALIGN	4
#else
#define DHD_SDALIGN	32
#endif
#endif 

typedef struct reorder_info {
	void **p;
	uint8 flow_id;
	uint8 cur_idx;
	uint8 exp_idx;
	uint8 max_idx;
	uint8 pend_pkts;
} reorder_info_t;

#ifdef DHDTCPACK_SUPPRESS

enum {
	
	TCPACK_SUP_OFF,
	
	TCPACK_SUP_REPLACE,
	TCPACK_SUP_DELAYTX,
	TCPACK_SUP_HOLD,
	TCPACK_SUP_LAST_MODE
};
#endif 


#ifdef BCM_INDX_TCM 
#define DMA_INDX_ENAB(dma_indxsup)	0
#elif defined BCM_INDX_DMA  
#define DMA_INDX_ENAB(dma_indxsup)	1
#else	
#define DMA_INDX_ENAB(dma_indxsup)	dma_indxsup
#endif	

#if defined(WLTDLS) && defined(PCIE_FULL_DONGLE)
struct tdls_peer_node {
	uint8 addr[ETHER_ADDR_LEN];
	struct tdls_peer_node *next;
};
typedef struct tdls_peer_node tdls_peer_node_t;
typedef struct {
	tdls_peer_node_t *node;
	uint8 tdls_peer_count;
} tdls_peer_tbl_t;
#endif 

typedef struct dhd_pub {
	
	osl_t *osh;		
	struct dhd_bus *bus;	
	struct dhd_prot *prot;	
	struct dhd_info  *info; 



	
	bool up;		
	bool txoff;		
	bool dongle_reset;  
	enum dhd_bus_state busstate;
	uint hdrlen;		
	uint maxctl;		
	uint rxsz;		
	uint8 wme_dp;	

	
	bool iswl;		
	ulong drv_version;	
	struct ether_addr mac;	
	dngl_stats_t dstats;	

	
	ulong tx_packets;	
	ulong tx_dropped;	
	ulong tx_multicast;	
	ulong tx_errors;	
	ulong tx_ctlpkts;	
	ulong tx_ctlerrs;	
	ulong rx_packets;	
	ulong rx_multicast;	
	ulong rx_errors;	
	ulong rx_ctlpkts;	
	ulong rx_ctlerrs;	
	ulong rx_dropped;	
	ulong rx_flushed;  
	ulong wd_dpc_sched;   

	ulong rx_readahead_cnt;	
	ulong tx_realloc;	
	ulong fc_packets;       

	
	int bcmerror;
	uint tickcnt;

	
	int dongle_error;

	uint8 country_code[WLC_CNTRY_BUF_SZ];

	
	int suspend_disable_flag; 
	int in_suspend;			
#ifdef PNO_SUPPORT
	int pno_enable;			
	int pno_suspend;		
#endif 
	int suspend_bcn_li_dtim;         
#ifdef PKT_FILTER_SUPPORT
	int early_suspended;	
	int dhcp_in_progress;	
#endif

	
	char * pktfilter[100];
	int pktfilter_count;

	wl_country_t dhd_cspec;		
	char eventmask[WL_EVENTING_MASK_LEN];
	int	op_mode;				

/* Set this to 1 to use a seperate interface (p2p0) for p2p operations.
 *  For ICS MR1 releases it should be disable to be compatable with ICS MR1 Framework
 *  see target dhd-cdc-sdmmc-panda-cfg80211-icsmr1-gpl-debug in Makefile
 */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
	struct mutex 	wl_start_stop_lock; 
	struct mutex 	wl_softap_lock;		 
#endif 

#ifdef WLBTAMP
	uint16	maxdatablks;
#endif 
#ifdef PROP_TXSTATUS
	bool	wlfc_enabled;
	int	wlfc_mode;
	void*	wlfc_state;
	uint8	proptxstatus_mode;
	bool	proptxstatus_txoff;
	bool	proptxstatus_module_ignore;
	bool	proptxstatus_credit_ignore;
	bool	proptxstatus_txstatus_ignore;

	bool	wlfc_rxpkt_chk;
	
	bool (*skip_fc)(void);
	
	void (*plat_init)(void *dhd);
	void (*plat_deinit)(void *dhd);
#endif 
#ifdef PNO_SUPPORT
	void *pno_state;
#endif
	bool	dongle_isolation;
	bool	dongle_trap_occured;	
	int   hang_was_sent;
	int   rxcnt_timeout;		
	int   txcnt_timeout;		
#ifdef BCMPCIE
	int   d3ackcnt_timeout;		
#endif 
	bool hang_report;		
#ifdef WLMEDIA_HTSF
	uint8 htsfdlystat_sz; 
#endif
#ifdef WLTDLS
	bool tdls_enable;
#endif
	struct reorder_info *reorder_bufs[WLHOST_REORDERDATA_MAXFLOWS];
	char  fw_capabilities[WLC_IOCTL_SMLEN];
	#define MAXSKBPEND 1024
	void *skbbuf[MAXSKBPEND];
	uint32 store_idx;
	uint32 sent_idx;
#ifdef DHDTCPACK_SUPPRESS
	uint8 tcpack_sup_mode;		
	void *tcpack_sup_module;	
	uint32 tcpack_sup_ratio;
	uint32 tcpack_sup_delay;
#endif 
#if defined(ARP_OFFLOAD_SUPPORT)
	uint32 arp_version;
#endif
#if defined(BCMSUP_4WAY_HANDSHAKE) && defined(WLAN_AKM_SUITE_FT_8021X)
	bool fw_4way_handshake;		
#endif
#if defined(CUSTOM_PLATFORM_NV_TEGRA)
#ifdef PKT_FILTER_SUPPORT
	uint pkt_filter_mode;
	uint pkt_filter_ports_count;
	uint16 pkt_filter_ports[WL_PKT_FILTER_PORTS_MAX];
#endif 
#endif 
#ifdef CUSTOM_SET_CPUCORE
	struct task_struct * current_dpc;
	struct task_struct * current_rxf;
	int chan_isvht80;
#endif 

	void    *sta_pool;          
	void    *staid_allocator;   

	void    *flowid_allocator;  
	void	*flow_ring_table;   
	void	*if_flow_lkup;      
	void    *flowid_lock;       
	uint32  num_flow_rings;

	uint32 d2h_sync_mode;		

	uint8  flow_prio_map[NUMPRIO];
	uint8	flow_prio_map_type;
	char enable_log[MAX_EVENT];
	bool dma_d2h_ring_upd_support;
	bool dma_h2d_ring_upd_support;
#ifdef DHD_WMF
	bool wmf_ucast_igmp;
#ifdef DHD_IGMP_UCQUERY
	bool wmf_ucast_igmp_query;
#endif
#ifdef DHD_UCAST_UPNP
	bool wmf_ucast_upnp;
#endif
#endif 
#ifdef DHD_UNICAST_DHCP
	bool dhcp_unicast;
#endif 
#ifdef DHD_L2_FILTER
	bool block_ping;
	bool trace_ping;
#endif
#if defined(WLTDLS) && defined(PCIE_FULL_DONGLE)
	tdls_peer_tbl_t peer_tbl;
#endif 
#if defined(USE_STATIC_MEMDUMP)
	uint8 *soc_ram;
	uint32 soc_ram_length;
	uint32 memdump_enabled;
#endif 
	bool tx_in_progress;
#ifdef CUSTOMER_HW_ONE
	bool os_stopped;
	uint32 txdesc_no_res;
	bool allow_p2p_event;
#endif
	unsigned int irq_cpu_count[NR_CPUS+1];
} dhd_pub_t;

#if defined(BCMWDF)
typedef struct {
	dhd_pub_t *dhd_pub;
} dhd_workitem_context_t;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(dhd_workitem_context_t, dhd_get_dhd_workitem_context)
#endif 

	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)

	#define DHD_PM_RESUME_WAIT_INIT(a) DECLARE_WAIT_QUEUE_HEAD(a);
	#define _DHD_PM_RESUME_WAIT(a, b) do {\
			int retry = 0; \
			SMP_RD_BARRIER_DEPENDS(); \
			while (dhd_mmc_suspend && retry++ != b) { \
				SMP_RD_BARRIER_DEPENDS(); \
				wait_event_interruptible_timeout(a, !dhd_mmc_suspend, 1); \
			} \
		} 	while (0)
	#define DHD_PM_RESUME_WAIT(a) 		_DHD_PM_RESUME_WAIT(a, 200)
	#define DHD_PM_RESUME_WAIT_FOREVER(a) 	_DHD_PM_RESUME_WAIT(a, ~0)
	#ifdef CUSTOMER_HW4
		#define DHD_PM_RESUME_RETURN_ERROR(a)   do { \
				if (dhd_mmc_suspend) { \
					printf("%s[%d]: mmc is still in suspend state!!!\n", \
							__FUNCTION__, __LINE__); \
					return a; \
				} \
			} while (0)
	#else
		#define DHD_PM_RESUME_RETURN_ERROR(a)	do { \
			if (dhd_mmc_suspend) return a; } while (0)
	#endif 
	#define DHD_PM_RESUME_RETURN		do { if (dhd_mmc_suspend) return; } while (0)

	#define DHD_SPINWAIT_SLEEP_INIT(a) DECLARE_WAIT_QUEUE_HEAD(a);
	#define SPINWAIT_SLEEP(a, exp, us) do { \
		uint countdown = (us) + 9999; \
		while ((exp) && (countdown >= 10000)) { \
			wait_event_interruptible_timeout(a, FALSE, 1); \
			countdown -= 10000; \
		} \
	} while (0)

	#else

	#define DHD_PM_RESUME_WAIT_INIT(a)
	#define DHD_PM_RESUME_WAIT(a)
	#define DHD_PM_RESUME_WAIT_FOREVER(a)
	#define DHD_PM_RESUME_RETURN_ERROR(a)
	#define DHD_PM_RESUME_RETURN

	#define DHD_SPINWAIT_SLEEP_INIT(a)
	#define SPINWAIT_SLEEP(a, exp, us)  do { \
		uint countdown = (us) + 9; \
		while ((exp) && (countdown >= 10)) { \
			OSL_DELAY(10);  \
			countdown -= 10;  \
		} \
	} while (0)

	#endif 

#ifndef OSL_SLEEP
#define OSL_SLEEP(ms)		OSL_DELAY(ms*1000)
#endif 

#define DHD_IF_VIF	0x01	

#ifdef PNO_SUPPORT
int dhd_pno_clean(dhd_pub_t *dhd);
#endif 
extern int dhd_os_wake_lock(dhd_pub_t *pub);
extern int dhd_os_wake_unlock(dhd_pub_t *pub);
extern int dhd_os_wake_lock_timeout(dhd_pub_t *pub);
extern int dhd_os_wake_lock_rx_timeout_enable(dhd_pub_t *pub, int val);
extern int dhd_os_wake_lock_ctrl_timeout_enable(dhd_pub_t *pub, int val);
extern int dhd_os_wake_lock_ctrl_timeout_cancel(dhd_pub_t *pub);
extern int dhd_os_wd_wake_lock(dhd_pub_t *pub);
extern int dhd_os_wd_wake_unlock(dhd_pub_t *pub);
#ifdef BCMPCIE_OOB_HOST_WAKE
extern int dhd_os_oob_irq_wake_lock_timeout(dhd_pub_t *pub, int val);
extern int dhd_os_oob_irq_wake_unlock(dhd_pub_t *pub);
#endif 
extern int dhd_os_wake_lock_waive(dhd_pub_t *pub);
extern int dhd_os_wake_lock_restore(dhd_pub_t *pub);

inline static void MUTEX_LOCK_SOFTAP_SET_INIT(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
	mutex_init(&dhdp->wl_softap_lock);
#endif 
}

inline static void MUTEX_LOCK_SOFTAP_SET(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
	mutex_lock(&dhdp->wl_softap_lock);
#endif 
}

inline static void MUTEX_UNLOCK_SOFTAP_SET(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
	mutex_unlock(&dhdp->wl_softap_lock);
#endif 
}

#define DHD_OS_WAKE_LOCK(pub)			dhd_os_wake_lock(pub)
#define DHD_OS_WAKE_UNLOCK(pub)		dhd_os_wake_unlock(pub)
#define DHD_OS_WAKE_LOCK_TIMEOUT(pub)		dhd_os_wake_lock_timeout(pub)
#define DHD_OS_WAKE_LOCK_RX_TIMEOUT_ENABLE(pub, val) \
	dhd_os_wake_lock_rx_timeout_enable(pub, val)
#define DHD_OS_WAKE_LOCK_CTRL_TIMEOUT_ENABLE(pub, val) \
	dhd_os_wake_lock_ctrl_timeout_enable(pub, val)
#define DHD_OS_WAKE_LOCK_CTRL_TIMEOUT_CANCEL(pub) \
	dhd_os_wake_lock_ctrl_timeout_cancel(pub)
#define DHD_OS_WAKE_LOCK_WAIVE(pub)             dhd_os_wake_lock_waive(pub)
#define DHD_OS_WAKE_LOCK_RESTORE(pub)           dhd_os_wake_lock_restore(pub)

#define DHD_OS_WD_WAKE_LOCK(pub)		dhd_os_wd_wake_lock(pub)
#define DHD_OS_WD_WAKE_UNLOCK(pub)		dhd_os_wd_wake_unlock(pub)
#ifdef BCMPCIE_OOB_HOST_WAKE
#define OOB_WAKE_LOCK_TIMEOUT 500
#define DHD_OS_OOB_IRQ_WAKE_LOCK_TIMEOUT(pub, val) dhd_os_oob_irq_wake_lock_timeout(pub, val)
#define DHD_OS_OOB_IRQ_WAKE_UNLOCK(pub)		dhd_os_oob_irq_wake_unlock(pub)
#endif 
#define DHD_PACKET_TIMEOUT_MS	500
#define DHD_EVENT_TIMEOUT_MS	1500


void dhd_net_if_lock(struct net_device *dev);
void dhd_net_if_unlock(struct net_device *dev);

#if defined(MULTIPLE_SUPPLICANT)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1 && defined(BCMSDIO)
extern struct mutex _dhd_sdio_mutex_lock_;
#endif
#endif 

typedef enum dhd_attach_states
{
	DHD_ATTACH_STATE_INIT = 0x0,
	DHD_ATTACH_STATE_NET_ALLOC = 0x1,
	DHD_ATTACH_STATE_DHD_ALLOC = 0x2,
	DHD_ATTACH_STATE_ADD_IF = 0x4,
	DHD_ATTACH_STATE_PROT_ATTACH = 0x8,
	DHD_ATTACH_STATE_WL_ATTACH = 0x10,
	DHD_ATTACH_STATE_THREADS_CREATED = 0x20,
	DHD_ATTACH_STATE_WAKELOCKS_INIT = 0x40,
	DHD_ATTACH_STATE_CFG80211 = 0x80,
	DHD_ATTACH_STATE_EARLYSUSPEND_DONE = 0x100,
	DHD_ATTACH_STATE_DONE = 0x200
} dhd_attach_states_t;

#define DHD_PID_KT_INVALID 	-1
#define DHD_PID_KT_TL_INVALID	-2


extern dhd_pub_t *dhd_attach(osl_t *osh, struct dhd_bus *bus, uint bus_hdrlen);
#if defined(WLP2P) && defined(WL_CFG80211)
extern int dhd_attach_p2p(dhd_pub_t *);
extern int dhd_detach_p2p(dhd_pub_t *);
#endif 
extern int dhd_register_if(dhd_pub_t *dhdp, int idx, bool need_rtnl_lock);

extern void dhd_detach(dhd_pub_t *dhdp);
extern void dhd_free(dhd_pub_t *dhdp);
extern void dhd_clear(dhd_pub_t *dhdp);

extern void dhd_txflowcontrol(dhd_pub_t *dhdp, int ifidx, bool on);

extern void dhd_store_conn_status(uint32 event, uint32 status, uint32 reason);

extern bool dhd_prec_enq(dhd_pub_t *dhdp, struct pktq *q, void *pkt, int prec);

extern void dhd_rx_frame(dhd_pub_t *dhdp, int ifidx, void *rxp, int numpkt, uint8 chan);

extern char *dhd_ifname(dhd_pub_t *dhdp, int idx);

extern void dhd_sched_dpc(dhd_pub_t *dhdp);

extern void dhd_txcomplete(dhd_pub_t *dhdp, void *txp, bool success);

extern int dhd_os_proto_block(dhd_pub_t * pub);
extern int dhd_os_proto_unblock(dhd_pub_t * pub);
extern int dhd_os_ioctl_resp_wait(dhd_pub_t * pub, uint * condition, bool * pending);
extern int dhd_os_ioctl_resp_wake(dhd_pub_t * pub);
extern unsigned int dhd_os_get_ioctl_resp_timeout(void);
extern void dhd_os_set_ioctl_resp_timeout(unsigned int timeout_msec);

extern int dhd_os_get_image_block(char * buf, int len, void * image);
extern void * dhd_os_open_image(char * filename);
extern void dhd_os_close_image(void * image);
extern void dhd_os_wd_timer(void *bus, uint wdtick);
extern void dhd_os_sdlock(dhd_pub_t * pub);
extern void dhd_os_sdunlock(dhd_pub_t * pub);
extern void dhd_os_sdlock_txq(dhd_pub_t * pub);
extern void dhd_os_sdunlock_txq(dhd_pub_t * pub);
extern void dhd_os_sdlock_rxq(dhd_pub_t * pub);
extern void dhd_os_sdunlock_rxq(dhd_pub_t * pub);
extern void dhd_os_sdlock_sndup_rxq(dhd_pub_t * pub);
#ifdef DHDTCPACK_SUPPRESS
extern unsigned long dhd_os_tcpacklock(dhd_pub_t *pub);
extern void dhd_os_tcpackunlock(dhd_pub_t *pub, unsigned long flags);
#endif 

extern int dhd_customer_oob_irq_map(void *adapter, unsigned long *irq_flags_ptr);
extern int dhd_customer_gpio_wlan_ctrl(void *adapter, int onoff);
extern int dhd_custom_get_mac_address(void *adapter, unsigned char *buf);
extern void get_customized_country_code(void *adapter, char *country_iso_code, wl_country_t *cspec);
extern void dhd_os_sdunlock_sndup_rxq(dhd_pub_t * pub);
extern void dhd_os_sdlock_eventq(dhd_pub_t * pub);
extern void dhd_os_sdunlock_eventq(dhd_pub_t * pub);
extern bool dhd_os_check_hang(dhd_pub_t *dhdp, int ifidx, int ret);
extern int dhd_os_send_hang_message(dhd_pub_t *dhdp);
extern void dhd_set_version_info(dhd_pub_t *pub, char *fw);
extern bool dhd_os_check_if_up(dhd_pub_t *pub);
extern int dhd_os_check_wakelock(dhd_pub_t *pub);
extern int dhd_os_check_wakelock_all(dhd_pub_t *pub);
extern int dhd_get_instance(dhd_pub_t *pub);
#ifdef CUSTOM_SET_CPUCORE
extern void dhd_set_cpucore(dhd_pub_t *dhd, int set);
#endif 

#if defined(KEEP_ALIVE)
extern int dhd_keep_alive_onoff(dhd_pub_t *dhd);
#endif 

#if defined(USE_STATIC_MEMDUMP)
void dhd_schedule_memdump(dhd_pub_t *dhdp, uint8 *buf, uint32 size);
#endif 

#ifdef SUPPORT_AP_POWERSAVE
extern int dhd_set_ap_powersave(dhd_pub_t *dhdp, int ifidx, int enable);
#endif


#ifdef PKT_FILTER_SUPPORT
#define DHD_UNICAST_FILTER_NUM		0
#define DHD_BROADCAST_FILTER_NUM	1
#define DHD_MULTICAST4_FILTER_NUM	2
#define DHD_MULTICAST6_FILTER_NUM	3
#define DHD_MDNS_FILTER_NUM		4
#define DHD_ARP_FILTER_NUM		5

#if defined(CUSTOM_PLATFORM_NV_TEGRA)
#define PKT_FILTER_PORTS_CLEAR		0
#define PKT_FILTER_PORTS_ADD		1
#define PKT_FILTER_PORTS_DEL		2
#define PKT_FILTER_PORTS_LOOPBACK	3
#define PKT_FILTER_PORTS_MAX		PKT_FILTER_PORTS_LOOPBACK
#endif 

extern int dhd_os_enable_packet_filter(dhd_pub_t *dhdp, int val);
extern void dhd_enable_packet_filter(int value, dhd_pub_t *dhd);
extern int net_os_enable_packet_filter(struct net_device *dev, int val);
extern int net_os_rxfilter_add_remove(struct net_device *dev, int val, int num);
#if defined(CUSTOM_PLATFORM_NV_TEGRA)
extern void dhd_set_packet_filter_mode(struct net_device *dev, char *command);
extern int dhd_set_packet_filter_ports(struct net_device *dev, char *command);
#endif 
#endif 

extern int dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd);
extern bool dhd_support_sta_mode(dhd_pub_t *dhd);

#ifdef DHD_DEBUG
extern int write_to_file(dhd_pub_t *dhd, uint8 *buf, int size);
#ifdef CUSTOMER_HW_ONE
extern int user_triger_write_to_file(dhd_pub_t *dhd, uint8 *buf, int size);
#endif
#endif 

typedef struct {
	uint32 limit;		
	uint32 increment;	
	uint32 elapsed;		
	uint32 tick;		
} dhd_timeout_t;

#ifdef SHOW_LOGTRACE
typedef struct {
	int  num_fmts;
	char **fmts;
	char *raw_fmts;
} dhd_event_log_t;
#endif 

extern void dhd_timeout_start(dhd_timeout_t *tmo, uint usec);
extern int dhd_timeout_expired(dhd_timeout_t *tmo);

extern int dhd_ifname2idx(struct dhd_info *dhd, char *name);
extern int dhd_ifidx2hostidx(struct dhd_info *dhd, int ifidx);
extern int dhd_net2idx(struct dhd_info *dhd, struct net_device *net);
extern struct net_device * dhd_idx2net(void *pub, int ifidx);
extern int net_os_send_hang_message(struct net_device *dev);
extern int wl_host_event(dhd_pub_t *dhd_pub, int *idx, void *pktdata,
                         wl_event_msg_t *, void **data_ptr,  void *);
extern void wl_event_to_host_order(wl_event_msg_t * evt);

extern int dhd_wl_ioctl(dhd_pub_t *dhd_pub, int ifindex, wl_ioctl_t *ioc, void *buf, int len);
extern int dhd_wl_ioctl_cmd(dhd_pub_t *dhd_pub, int cmd, void *arg, int len, uint8 set,
                            int ifindex);
extern void dhd_common_init(osl_t *osh);

extern int dhd_do_driver_init(struct net_device *net);
extern int dhd_event_ifadd(struct dhd_info *dhd, struct wl_event_data_if *ifevent,
	char *name, uint8 *mac);
extern int dhd_event_ifdel(struct dhd_info *dhd, struct wl_event_data_if *ifevent,
	char *name, uint8 *mac);
extern struct net_device* dhd_allocate_if(dhd_pub_t *dhdpub, int ifidx, char *name,
	uint8 *mac, uint8 bssidx, bool need_rtnl_lock);
extern int dhd_remove_if(dhd_pub_t *dhdpub, int ifidx, bool need_rtnl_lock);
extern void dhd_vif_add(struct dhd_info *dhd, int ifidx, char * name);
extern void dhd_vif_del(struct dhd_info *dhd, int ifidx);
extern void dhd_event(struct dhd_info *dhd, char *evpkt, int evlen, int ifidx);
extern void dhd_vif_sendup(struct dhd_info *dhd, int ifidx, uchar *cp, int len);

extern int dhd_sendpkt(dhd_pub_t *dhdp, int ifidx, void *pkt);

extern void dhd_sendup_event_common(dhd_pub_t *dhdp, wl_event_msg_t *event, void *data);
extern void dhd_sendup_event(dhd_pub_t *dhdp, wl_event_msg_t *event, void *data);
#ifdef LOG_INTO_TCPDUMP
extern void dhd_sendup_log(dhd_pub_t *dhdp, void *data, int len);
#endif 
extern int dhd_bus_devreset(dhd_pub_t *dhdp, uint8 flag);
extern uint dhd_bus_status(dhd_pub_t *dhdp);
extern int  dhd_bus_start(dhd_pub_t *dhdp);
extern int dhd_bus_suspend(dhd_pub_t *dhdpub);
extern int dhd_bus_resume(dhd_pub_t *dhdpub, int stage);
extern int dhd_bus_membytes(dhd_pub_t *dhdp, bool set, uint32 address, uint8 *data, uint size);
extern void dhd_print_buf(void *pbuf, int len, int bytes_per_line);
extern bool dhd_is_associated(dhd_pub_t *dhd, void *bss_buf, int *retval);
#if defined(BCMSDIO) || defined(BCMPCIE)
extern uint dhd_bus_chip_id(dhd_pub_t *dhdp);
extern uint dhd_bus_chiprev_id(dhd_pub_t *dhdp);
extern uint dhd_bus_chippkg_id(dhd_pub_t *dhdp);
#endif 

#if defined(KEEP_ALIVE)
extern int dhd_keep_alive_onoff(dhd_pub_t *dhd);
#endif 

extern void *dhd_os_spin_lock_init(osl_t *osh);
extern void dhd_os_spin_lock_deinit(osl_t *osh, void *lock);
extern unsigned long dhd_os_spin_lock(void *lock);
void dhd_os_spin_unlock(void *lock, unsigned long flags);

struct dhd_sta;
extern struct dhd_sta *dhd_findadd_sta(void *pub, int ifidx, void *ea);
extern void dhd_del_sta(void *pub, int ifidx, void *ea);
extern int dhd_get_ap_isolate(dhd_pub_t *dhdp, uint32 idx);
extern int dhd_set_ap_isolate(dhd_pub_t *dhdp, uint32 idx, int val);
extern int dhd_bssidx2idx(dhd_pub_t *dhdp, uint32 bssidx);
extern int dhd_os_d3ack_wait(dhd_pub_t * pub, uint * condition, bool * pending);
extern int dhd_os_d3ack_wake(dhd_pub_t * pub);

extern bool dhd_is_concurrent_mode(dhd_pub_t *dhd);
extern int dhd_iovar(dhd_pub_t *pub, int ifidx, char *name, char *cmd_buf, uint cmd_len, int set);
typedef enum cust_gpio_modes {
	WLAN_RESET_ON,
	WLAN_RESET_OFF,
	WLAN_POWER_ON,
	WLAN_POWER_OFF
} cust_gpio_modes_t;

extern int wl_iw_iscan_set_scan_broadcast_prep(struct net_device *dev, uint flag);
extern int wl_iw_send_priv_event(struct net_device *dev, char *flag);

extern uint dhd_watchdog_ms;

#if defined(DHD_DEBUG)
extern uint dhd_console_ms;
extern uint wl_msg_level;
#endif 

extern uint dhd_slpauto;

extern uint dhd_intr;

extern uint dhd_poll;

extern uint dhd_arp_mode;

extern uint dhd_arp_enable;

extern uint dhd_pkt_filter_enable;

extern uint dhd_pkt_filter_init;

extern uint dhd_master_mode;

extern uint dhd_roam_disable;

extern uint dhd_radio_up;

extern int dhd_idletime;
#ifdef DHD_USE_IDLECOUNT
#define DHD_IDLETIME_TICKS 5
#else
#define DHD_IDLETIME_TICKS 1
#endif 

extern uint dhd_sdiod_drive_strength;

extern uint dhd_force_tx_queueing;
#define DEFAULT_KEEP_ALIVE_VALUE 	55000 
#ifndef CUSTOM_KEEP_ALIVE_SETTING
#define CUSTOM_KEEP_ALIVE_SETTING 	DEFAULT_KEEP_ALIVE_VALUE
#endif 

#define NULL_PKT_STR	"null_pkt"

#define DEFAULT_GLOM_VALUE 	-1
#ifndef CUSTOM_GLOM_SETTING
#define CUSTOM_GLOM_SETTING 	DEFAULT_GLOM_VALUE
#endif
#define WL_AUTO_ROAM_TRIGGER -75
#define DEFAULT_ROAM_TRIGGER_VALUE -75 
#define DEFAULT_ROAM_TRIGGER_SETTING 	-1
#ifndef CUSTOM_ROAM_TRIGGER_SETTING
#define CUSTOM_ROAM_TRIGGER_SETTING 	DEFAULT_ROAM_TRIGGER_VALUE
#endif

#define DEFAULT_ROAM_DELTA_VALUE  10 
#define DEFAULT_ROAM_DELTA_SETTING 	-1
#ifndef CUSTOM_ROAM_DELTA_SETTING
#define CUSTOM_ROAM_DELTA_SETTING 	DEFAULT_ROAM_DELTA_VALUE
#endif

#define DEFAULT_PNO_EVENT_LOCK_xTIME 	2 	
#ifndef CUSTOM_PNO_EVENT_LOCK_xTIME
#define CUSTOM_PNO_EVENT_LOCK_xTIME	 DEFAULT_PNO_EVENT_LOCK_xTIME
#endif
#ifdef CUSTOMER_HW_ONE
#define DEFAULT_DHP_DPC_PRIO  0
#else
#define DEFAULT_DHP_DPC_PRIO  1
#endif
#ifndef CUSTOM_DPC_PRIO_SETTING
#define CUSTOM_DPC_PRIO_SETTING 	DEFAULT_DHP_DPC_PRIO
#endif

#ifndef CUSTOM_LISTEN_INTERVAL
#define CUSTOM_LISTEN_INTERVAL 		LISTEN_INTERVAL
#endif 

#define DEFAULT_SUSPEND_BCN_LI_DTIM		3
#ifndef CUSTOM_SUSPEND_BCN_LI_DTIM
#define CUSTOM_SUSPEND_BCN_LI_DTIM		DEFAULT_SUSPEND_BCN_LI_DTIM
#endif

#ifndef CUSTOM_RXF_PRIO_SETTING
#ifdef CUSTOMER_HW_ONE
#define CUSTOM_RXF_PRIO_SETTING 	(DEFAULT_DHP_DPC_PRIO + 1)
#else
#define CUSTOM_RXF_PRIO_SETTING		MAX((CUSTOM_DPC_PRIO_SETTING - 1), 1)
#endif
#endif 

#define DEFAULT_WIFI_TURNOFF_DELAY		0
#ifndef WIFI_TURNOFF_DELAY
#define WIFI_TURNOFF_DELAY		DEFAULT_WIFI_TURNOFF_DELAY
#endif 

#define DEFAULT_WIFI_TURNON_DELAY		200
#ifndef WIFI_TURNON_DELAY
#define WIFI_TURNON_DELAY		DEFAULT_WIFI_TURNON_DELAY
#endif 

#ifdef CUSTOMER_HW_ONE
#define DEFAULT_DHD_WATCHDOG_INTERVAL_MS	125 
#else
#define DEFAULT_DHD_WATCHDOG_INTERVAL_MS	10 
#endif 
#ifndef CUSTOM_DHD_WATCHDOG_MS
#define CUSTOM_DHD_WATCHDOG_MS			DEFAULT_DHD_WATCHDOG_INTERVAL_MS
#endif 

#ifdef WLTDLS
#ifndef CUSTOM_TDLS_IDLE_MODE_SETTING
#define CUSTOM_TDLS_IDLE_MODE_SETTING  60000 
#endif
#ifndef CUSTOM_TDLS_RSSI_THRESHOLD_HIGH
#define CUSTOM_TDLS_RSSI_THRESHOLD_HIGH -70 
#endif
#ifndef CUSTOM_TDLS_RSSI_THRESHOLD_LOW
#define CUSTOM_TDLS_RSSI_THRESHOLD_LOW -80 
#endif
#endif 

#define DEFAULT_BCN_TIMEOUT		8
#ifndef CUSTOM_BCN_TIMEOUT
#define CUSTOM_BCN_TIMEOUT		DEFAULT_BCN_TIMEOUT
#endif

#define MAX_DTIM_SKIP_BEACON_INTERVAL	100 
#ifndef MAX_DTIM_ALLOWED_INTERVAL
#define MAX_DTIM_ALLOWED_INTERVAL 600 
#endif
#define NO_DTIM_SKIP 1
#ifdef SDTEST
extern uint dhd_pktgen;

extern uint dhd_pktgen_len;
#define MAX_PKTGEN_LEN 1800
#endif


#define MOD_PARAM_PATHLEN	2048
#define MOD_PARAM_INFOLEN	512

#ifdef SOFTAP
extern char fw_path2[MOD_PARAM_PATHLEN];
#endif

extern uint dhd_download_fw_on_driverload;


extern void dhd_wait_for_event(dhd_pub_t *dhd, bool *lockvar);
extern void dhd_wait_event_wakeup(dhd_pub_t*dhd);

#define IFLOCK_INIT(lock)       *lock = 0
#define IFLOCK(lock)    while (InterlockedCompareExchange((lock), 1, 0))	\
	NdisStallExecution(1);
#define IFUNLOCK(lock)  InterlockedExchange((lock), 0)
#define IFLOCK_FREE(lock)
#define FW_SUPPORTED(dhd, capa) ((strstr(dhd->fw_capabilities, #capa) != NULL))
#ifdef ARP_OFFLOAD_SUPPORT
#define MAX_IPV4_ENTRIES	8
void dhd_arp_offload_set(dhd_pub_t * dhd, int arp_mode);
void dhd_arp_offload_enable(dhd_pub_t * dhd, int arp_enable);

void dhd_aoe_hostip_clr(dhd_pub_t *dhd, int idx);
void dhd_aoe_arp_clr(dhd_pub_t *dhd, int idx);
int dhd_arp_get_arp_hostip_table(dhd_pub_t *dhd, void *buf, int buflen, int idx);
void dhd_arp_offload_add_ip(dhd_pub_t *dhd, uint32 ipaddr, int idx);
#endif 
#ifdef WLTDLS
int dhd_tdls_enable(struct net_device *dev, bool tdls_on, bool auto_on, struct ether_addr *mac);
#ifdef PCIE_FULL_DONGLE
void dhd_tdls_update_peer_info(struct net_device *dev, bool connect_disconnect, uint8 *addr);
#endif 
#endif 
int dhd_ndo_enable(dhd_pub_t * dhd, int ndo_enable);
int dhd_ndo_add_ip(dhd_pub_t *dhd, char* ipaddr, int idx);
int dhd_ndo_remove_ip(dhd_pub_t *dhd, int idx);
int dhd_ioctl_process(dhd_pub_t *pub, int ifidx, struct dhd_ioctl *ioc, void *data_buf);

void dhd_bus_update_fw_nv_path(struct dhd_bus *bus, char *pfw_path, char *pnv_path);
void dhd_set_bus_state(void *bus, uint32 state);

typedef int (*f_droppkt_t)(dhd_pub_t *dhdp, int prec, void* p, bool bPktInQ);
extern bool dhd_prec_drop_pkts(dhd_pub_t *dhdp, struct pktq *pq, int prec, f_droppkt_t fn);

#ifdef PROP_TXSTATUS
int dhd_os_wlfc_block(dhd_pub_t *pub);
int dhd_os_wlfc_unblock(dhd_pub_t *pub);
extern const uint8 prio2fifo[];
#endif 

uint8* dhd_os_prealloc(dhd_pub_t *dhdpub, int section, uint size, bool kmalloc_if_fail);
void dhd_os_prefree(dhd_pub_t *dhdpub, void *addr, uint size);

int dhd_process_cid_mac(dhd_pub_t *dhdp, bool prepost);

#if defined(CONFIG_DHD_USE_STATIC_BUF)
#define DHD_OS_PREALLOC(dhdpub, section, size) dhd_os_prealloc(dhdpub, section, size, FALSE)
#define DHD_OS_PREFREE(dhdpub, addr, size) dhd_os_prefree(dhdpub, addr, size)
#else
#define DHD_OS_PREALLOC(dhdpub, section, size) MALLOC(dhdpub->osh, size)
#define DHD_OS_PREFREE(dhdpub, addr, size) MFREE(dhdpub->osh, addr, size)
#endif 


#define dhd_add_flowid(pub, ifidx, ac_prio, ea, flowid)  do {} while (0)
#define dhd_del_flowid(pub, ifidx, flowid)               do {} while (0)

extern unsigned long dhd_os_general_spin_lock(dhd_pub_t *pub);
extern void dhd_os_general_spin_unlock(dhd_pub_t *pub, unsigned long flags);


#define DHD_PERIM_LOCK(dhdp)              do {} while (0)
#define DHD_PERIM_UNLOCK(dhdp)            do {} while (0)

#define DHD_GENERAL_LOCK(dhdp, flags) \
	(flags) = dhd_os_general_spin_lock(dhdp)
#define DHD_GENERAL_UNLOCK(dhdp, flags) \
	dhd_os_general_spin_unlock((dhdp), (flags))

#define DHD_FLOWRING_LOCK(lock, flags)     (flags) = dhd_os_spin_lock(lock)
#define DHD_FLOWRING_UNLOCK(lock, flags)   dhd_os_spin_unlock((lock), (flags))

#define DHD_FLOWID_LOCK(lock, flags)       (flags) = dhd_os_spin_lock(lock)
#define DHD_FLOWID_UNLOCK(lock, flags)     dhd_os_spin_unlock((lock), (flags))



typedef struct wl_io_pport {
	dhd_pub_t *dhd_pub;
	uint ifidx;
} wl_io_pport_t;

extern void *dhd_pub_wlinfo(dhd_pub_t *dhd_pub);
#ifdef CONFIG_MACH_UNIVERSAL5433
extern int check_rev(void);
#endif

#ifdef CUSTOMER_HW_ONE
struct dd_pkt_filter_s {
	int add;
	int id;
	int offset;
	char mask[256];
	char pattern[256];
};

extern void dhd_htc_wake_lock_timeout(dhd_pub_t *pub, int sec);
extern bool dhd_check_ap_mode_set(dhd_pub_t *dhd);
extern int wl_android_black_list_match(char *ea);
extern int dhd_os_set_packet_filter(dhd_pub_t *dhdp, int val);

extern void wl_android_enable_pktfilter(struct net_device *dev, int multicastlock);
void wl_android_traffic_monitor(struct net_device *dev);
void wlan_unlock_multi_core(struct net_device *dev);
void wlan_unlock_perf(void);
int dhdhtc_set_power_control(int power_mode, unsigned int reason);
void wlan_lock_perf(void);
void wlan_lock_multi_core(struct net_device *dev);

#ifdef PNO_SUPPORT
#define MAX_PFN_NUMBER	2
#define PFN_SCAN_FREQ	300 
#endif

enum pkt_filter_id {
	ALLOW_UNICAST = 100,
	ALLOW_ARP,
	ALLOW_DHCP,
	ALLOW_IPV4_MULTICAST,
	ALLOW_IPV6_MULTICAST
};
int dhd_set_pktfilter(dhd_pub_t * dhd, int add, int id, int offset, char *mask, char *pattern);

enum dhdhtc_pwr_ctrl {
	DHDHTC_POWER_CTRL_ANDROID_NORMAL = 0,
	DHDHTC_POWER_CTRL_BROWSER_LOAD_PAGE,
	DHDHTC_POWER_CTRL_USER_CONFIG,
	DHDHTC_POWER_CTRL_WIFI_PHONE,
	DHDHTC_POWER_CTRL_FOTA_DOWNLOADING,
	DHDHTC_POWER_CTRL_KDDI_APK,
	DHDHTC_POWER_CTRL_MAX_NUM
};
extern int dhdhtc_update_wifi_power_mode(int is_screen_off);
extern unsigned int dhdhtc_get_cur_pwr_ctrl(void);
extern int dhdhtc_update_dtim_listen_interval(int is_screen_off);
extern char firmware_path[MOD_PARAM_PATHLEN];

extern int dhd_get_txrx_stats(struct net_device *net, unsigned long *rx_packets,
	unsigned long *tx_packets);
extern bool dhd_APUP;
#define MAX_TXQ_FULL_EVENT 300
extern int block_ap_event;
extern int dhdcdc_power_active_while_plugin;
extern char wl_abdroid_gatewaybuf[8+1]; 
#endif 
#endif 
