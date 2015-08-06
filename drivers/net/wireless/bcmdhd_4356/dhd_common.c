/*
 * Broadcom Dongle Host Driver (DHD), common DHD core.
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
 * $Id: dhd_common.c 542237 2015-03-19 06:49:27Z $
 */
#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <wlioctl.h>
#include <dhd.h>
#include <dhd_ip.h>
#include <proto/bcmevent.h>

#ifdef SHOW_LOGTRACE
#include <event_log.h>
#endif 

#ifdef BCMPCIE
#include <dhd_flowring.h>
#endif

#include <dhd_bus.h>
#include <dhd_proto.h>
#include <dhd_dbg.h>
#include <msgtrace.h>

#ifdef WL_CFG80211
#include <wl_cfg80211.h>
#endif
#ifdef WLBTAMP
#include <proto/bt_amp_hci.h>
#include <dhd_bta.h>
#endif
#ifdef PNO_SUPPORT
#include <dhd_pno.h>
#endif

#define htod32(i) (i)
#define htod16(i) (i)
#define dtoh32(i) (i)
#define dtoh16(i) (i)
#define htodchanspec(i) (i)
#define dtohchanspec(i) (i)

#ifdef PROP_TXSTATUS
#include <wlfc_proto.h>
#include <dhd_wlfc.h>
#endif

#ifdef DHD_WMF
#include <dhd_linux.h>
#include <dhd_wmf_linux.h>
#endif 


#ifdef WLMEDIA_HTSF
extern void htsf_update(struct dhd_info *dhd, void *data);
#endif
int dhd_msg_level = DHD_ERROR_VAL;


#include <wl_iw.h>

#ifdef SOFTAP
char fw_path2[MOD_PARAM_PATHLEN];
extern bool softap_enabled;
#endif

#ifdef CUSTOMER_HW_ONE
#if defined(KEEP_ALIVE)
int dhd_keep_alive_onoff(dhd_pub_t *dhd);
#endif 

#define WIFI_DEFAULT 1
#define WIFI_EMEA 2
char nvram_EMEA_path[MOD_PARAM_PATHLEN] = "/etc/calibration_EMEA";
extern int get_wifi_setting(void);
extern bool hasDLNA;
extern char ip_str[32];
extern int is_screen_off;
#endif 

uint32 dhd_conn_event;
uint32 dhd_conn_status;
uint32 dhd_conn_reason;

#if defined(SHOW_EVENTS) && defined(SHOW_LOGTRACE)
static int check_event_log_sequence_number(uint32 seq_no);
#endif 
extern int dhd_iscan_request(void * dhdp, uint16 action);
extern void dhd_ind_scan_confirm(void *h, bool status);
extern int dhd_iscan_in_progress(void *h);
void dhd_iscan_lock(void);
void dhd_iscan_unlock(void);
extern int dhd_change_mtu(dhd_pub_t *dhd, int new_mtu, int ifidx);
#if !defined(AP) && defined(WLP2P)
extern int dhd_get_concurrent_capabilites(dhd_pub_t *dhd);
#endif
bool ap_cfg_running = FALSE;
bool ap_fw_loaded = FALSE;

#ifdef DHD_DEBUG
#ifndef SRCBASE
#define SRCBASE        "drivers/net/wireless/bcmdhd"
#endif
#define DHD_COMPILED "\nCompiled in " SRCBASE
#endif 

#if defined(DHD_DEBUG)
const char dhd_version[] = "Dongle Host Driver, version " EPI_VERSION_STR
	DHD_COMPILED " on " __DATE__ " at " __TIME__;
#else
const char dhd_version[] = "\nDongle Host Driver, version " EPI_VERSION_STR "\nCompiled from ";
#endif 

void dhd_set_timer(void *bus, uint wdtick);



enum {
	IOV_VERSION = 1,
	IOV_MSGLEVEL,
	IOV_BCMERRORSTR,
	IOV_BCMERROR,
	IOV_WDTICK,
	IOV_DUMP,
	IOV_CLEARCOUNTS,
	IOV_LOGDUMP,
	IOV_LOGCAL,
	IOV_LOGSTAMP,
	IOV_GPIOOB,
	IOV_IOCTLTIMEOUT,
#ifdef WLBTAMP
	IOV_HCI_CMD,		
	IOV_HCI_ACL_DATA,	
#endif
#if defined(DHD_DEBUG)
	IOV_CONS,
	IOV_DCONSOLE_POLL,
#endif 
#ifdef PROP_TXSTATUS
	IOV_PROPTXSTATUS_ENABLE,
	IOV_PROPTXSTATUS_MODE,
	IOV_PROPTXSTATUS_OPT,
#ifdef QMONITOR
	IOV_QMON_TIME_THRES,
	IOV_QMON_TIME_PERCENT,
#endif 
	IOV_PROPTXSTATUS_MODULE_IGNORE,
	IOV_PROPTXSTATUS_CREDIT_IGNORE,
	IOV_PROPTXSTATUS_TXSTATUS_IGNORE,
	IOV_PROPTXSTATUS_RXPKT_CHK,
#endif 
	IOV_BUS_TYPE,
#ifdef WLMEDIA_HTSF
	IOV_WLPKTDLYSTAT_SZ,
#endif
	IOV_CHANGEMTU,
	IOV_HOSTREORDER_FLOWS,
#ifdef DHDTCPACK_SUPPRESS
	IOV_TCPACK_SUPPRESS,
#endif 
#ifdef DHD_WMF
	IOV_WMF_BSS_ENAB,
	IOV_WMF_UCAST_IGMP,
	IOV_WMF_MCAST_DATA_SENDUP,
#ifdef WL_IGMP_UCQUERY
	IOV_WMF_UCAST_IGMP_QUERY,
#endif 
#ifdef DHD_UCAST_UPNP
	IOV_WMF_UCAST_UPNP,
#endif 
#endif 
	IOV_AP_ISOLATE,
#ifdef DHD_UNICAST_DHCP
	IOV_DHCP_UNICAST,
#endif 
#ifdef DHD_L2_FILTER
	IOV_BLOCK_PING,
	IOV_TRACE_PING,
#endif
	IOV_LAST
};

const bcm_iovar_t dhd_iovars[] = {
	{"version",	IOV_VERSION,	0,	IOVT_BUFFER,	sizeof(dhd_version) },
#ifdef DHD_DEBUG
	{"msglevel",	IOV_MSGLEVEL,	0,	IOVT_UINT32,	0 },
#endif 
	{"bcmerrorstr", IOV_BCMERRORSTR, 0, IOVT_BUFFER,	BCME_STRLEN },
	{"bcmerror",	IOV_BCMERROR,	0,	IOVT_INT8,	0 },
	{"wdtick",	IOV_WDTICK, 0,	IOVT_UINT32,	0 },
	{"dump",	IOV_DUMP,	0,	IOVT_BUFFER,	DHD_IOCTL_MAXLEN },
#ifdef DHD_DEBUG
	{"cons",	IOV_CONS,	0,	IOVT_BUFFER,	0 },
	{"dconpoll",	IOV_DCONSOLE_POLL, 0,	IOVT_UINT32,	0 },
#endif
	{"clearcounts", IOV_CLEARCOUNTS, 0, IOVT_VOID,	0 },
	{"gpioob",	IOV_GPIOOB,	0,	IOVT_UINT32,	0 },
	{"ioctl_timeout",	IOV_IOCTLTIMEOUT,	0,	IOVT_UINT32,	0 },
#ifdef WLBTAMP
	{"HCI_cmd",	IOV_HCI_CMD,	0,	IOVT_BUFFER,	0},
	{"HCI_ACL_data", IOV_HCI_ACL_DATA, 0,	IOVT_BUFFER,	0},
#endif
#ifdef PROP_TXSTATUS
	{"proptx",	IOV_PROPTXSTATUS_ENABLE,	0,	IOVT_BOOL,	0 },
	{"ptxmode",	IOV_PROPTXSTATUS_MODE,	0,	IOVT_UINT32,	0 },
	{"proptx_opt", IOV_PROPTXSTATUS_OPT,	0,	IOVT_UINT32,	0 },
#ifdef QMONITOR
	{"qtime_thres",	IOV_QMON_TIME_THRES,	0,	IOVT_UINT32,	0 },
	{"qtime_percent", IOV_QMON_TIME_PERCENT, 0,	IOVT_UINT32,	0 },
#endif 
	{"pmodule_ignore", IOV_PROPTXSTATUS_MODULE_IGNORE, 0, IOVT_BOOL, 0 },
	{"pcredit_ignore", IOV_PROPTXSTATUS_CREDIT_IGNORE, 0, IOVT_BOOL, 0 },
	{"ptxstatus_ignore", IOV_PROPTXSTATUS_TXSTATUS_IGNORE, 0, IOVT_BOOL, 0 },
	{"rxpkt_chk", IOV_PROPTXSTATUS_RXPKT_CHK, 0, IOVT_BOOL, 0 },
#endif 
	{"bustype", IOV_BUS_TYPE, 0, IOVT_UINT32, 0},
#ifdef WLMEDIA_HTSF
	{"pktdlystatsz", IOV_WLPKTDLYSTAT_SZ, 0, IOVT_UINT8, 0 },
#endif
	{"changemtu", IOV_CHANGEMTU, 0, IOVT_UINT32, 0 },
	{"host_reorder_flows", IOV_HOSTREORDER_FLOWS, 0, IOVT_BUFFER,
	(WLHOST_REORDERDATA_MAXFLOWS + 1) },
#ifdef DHDTCPACK_SUPPRESS
	{"tcpack_suppress",	IOV_TCPACK_SUPPRESS,	0,	IOVT_UINT8,	0 },
#endif 
#ifdef DHD_WMF
	{"wmf_bss_enable", IOV_WMF_BSS_ENAB,	0,	IOVT_BOOL,	0 },
	{"wmf_ucast_igmp", IOV_WMF_UCAST_IGMP,	0,	IOVT_BOOL,	0 },
	{"wmf_mcast_data_sendup", IOV_WMF_MCAST_DATA_SENDUP,	0,	IOVT_BOOL,	0 },
#ifdef WL_IGMP_UCQUERY
	{"wmf_ucast_igmp_query", IOV_WMF_UCAST_IGMP_QUERY, (0), IOVT_BOOL, 0 },
#endif 
#ifdef DHD_UCAST_UPNP
	{"wmf_ucast_upnp", IOV_WMF_UCAST_UPNP, (0), IOVT_BOOL, 0 },
#endif 
#endif 
#ifdef DHD_UNICAST_DHCP
	{"dhcp_unicast", IOV_DHCP_UNICAST, (0), IOVT_BOOL, 0 },
#endif 
	{"ap_isolate", IOV_AP_ISOLATE, (0), IOVT_BOOL, 0},
#ifdef DHD_L2_FILTER
	{"block_ping", IOV_BLOCK_PING, (0), IOVT_BOOL, 0},
	{"trace_ping", IOV_TRACE_PING, (0), IOVT_BOOL, 0},
#endif
	{NULL, 0, 0, 0, 0 }
};

#define DHD_IOVAR_BUF_SIZE	128


static int
dhd_dump(dhd_pub_t *dhdp, char *buf, int buflen)
{
	char eabuf[ETHER_ADDR_STR_LEN];

	struct bcmstrbuf b;
	struct bcmstrbuf *strbuf = &b;
	int i;

	bcm_binit(strbuf, buf, buflen);

	
	bcm_bprintf(strbuf, "%s\n", dhd_version);
	bcm_bprintf(strbuf, "\n");
	bcm_bprintf(strbuf, "pub.up %d pub.txoff %d pub.busstate %d\n",
	            dhdp->up, dhdp->txoff, dhdp->busstate);
	bcm_bprintf(strbuf, "pub.hdrlen %u pub.maxctl %u pub.rxsz %u\n",
	            dhdp->hdrlen, dhdp->maxctl, dhdp->rxsz);
	bcm_bprintf(strbuf, "pub.iswl %d pub.drv_version %ld pub.mac %s\n",
	            dhdp->iswl, dhdp->drv_version, bcm_ether_ntoa(&dhdp->mac, eabuf));
	bcm_bprintf(strbuf, "pub.bcmerror %d tickcnt %u\n", dhdp->bcmerror, dhdp->tickcnt);

	bcm_bprintf(strbuf, "dongle stats:\n");
	bcm_bprintf(strbuf, "tx_packets %lu tx_bytes %lu tx_errors %lu tx_dropped %lu\n",
	            dhdp->dstats.tx_packets, dhdp->dstats.tx_bytes,
	            dhdp->dstats.tx_errors, dhdp->dstats.tx_dropped);
	bcm_bprintf(strbuf, "rx_packets %lu rx_bytes %lu rx_errors %lu rx_dropped %lu\n",
	            dhdp->dstats.rx_packets, dhdp->dstats.rx_bytes,
	            dhdp->dstats.rx_errors, dhdp->dstats.rx_dropped);
	bcm_bprintf(strbuf, "multicast %lu\n", dhdp->dstats.multicast);

	bcm_bprintf(strbuf, "bus stats:\n");
	bcm_bprintf(strbuf, "tx_packets %lu  tx_dropped %lu tx_multicast %lu tx_errors %lu\n",
	            dhdp->tx_packets, dhdp->tx_dropped, dhdp->tx_multicast, dhdp->tx_errors);
	bcm_bprintf(strbuf, "tx_ctlpkts %lu tx_ctlerrs %lu\n",
	            dhdp->tx_ctlpkts, dhdp->tx_ctlerrs);
	bcm_bprintf(strbuf, "rx_packets %lu rx_multicast %lu rx_errors %lu \n",
	            dhdp->rx_packets, dhdp->rx_multicast, dhdp->rx_errors);
	bcm_bprintf(strbuf, "rx_ctlpkts %lu rx_ctlerrs %lu rx_dropped %lu\n",
	            dhdp->rx_ctlpkts, dhdp->rx_ctlerrs, dhdp->rx_dropped);
	bcm_bprintf(strbuf, "rx_readahead_cnt %lu tx_realloc %lu\n",
	            dhdp->rx_readahead_cnt, dhdp->tx_realloc);
	bcm_bprintf(strbuf, "\n");

	bcm_bprintf(strbuf, "irq stats:\n");
	for (i = 0; i < NR_CPUS; i++) {
		bcm_bprintf(strbuf, "CPU%d[%d] ", i, dhdp->irq_cpu_count[i]);
	}
	bcm_bprintf(strbuf, "\n");

	
	dhd_prot_dump(dhdp, strbuf);
	bcm_bprintf(strbuf, "\n");

	
	dhd_bus_dump(dhdp, strbuf);


	return (!strbuf->size ? BCME_BUFTOOSHORT : 0);
}

int
dhd_wl_ioctl_cmd(dhd_pub_t *dhd_pub, int cmd, void *arg, int len, uint8 set, int ifidx)
{
	wl_ioctl_t ioc;

	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;
	ioc.set = set;

	return dhd_wl_ioctl(dhd_pub, ifidx, &ioc, arg, len);
}

int
dhd_wl_ioctl(dhd_pub_t *dhd_pub, int ifidx, wl_ioctl_t *ioc, void *buf, int len)
{
	int ret = BCME_ERROR;

	if (dhd_os_proto_block(dhd_pub))
	{
#if defined(WL_WLC_SHIM)
		wl_info_t *wl = dhd_pub_wlinfo(dhd_pub);

		wl_io_pport_t io_pport;
		io_pport.dhd_pub = dhd_pub;
		io_pport.ifidx = ifidx;

		ret = wl_shim_ioctl(wl->shim, ioc, &io_pport);
		if (ret != BCME_OK) {
			DHD_ERROR(("%s: wl_shim_ioctl(%d) ERR %d\n", __FUNCTION__, ioc->cmd, ret));
		}
#else
		ret = dhd_prot_ioctl(dhd_pub, ifidx, ioc, buf, len);
#endif 

		if (ret && dhd_pub->up) {
			
			dhd_os_check_hang(dhd_pub, ifidx, ret);
		}

		if (ret == -ETIMEDOUT && !dhd_pub->up) {
			DHD_ERROR(("%s: 'resumed on timeout' error is "
				"occurred before the interface does not"
				" bring up\n", __FUNCTION__));
			dhd_pub->busstate = DHD_BUS_DOWN;
		}

		dhd_os_proto_unblock(dhd_pub);

#if defined(CUSTOMER_HW_ONE)
		if (ret < 0) {
			if (ioc->cmd == WLC_GET_VAR)
				DHD_ERROR(("%s: WLC_GET_VAR: %s, ret = %d\n",
					__FUNCTION__, (char *)ioc->buf, ret));
			else if (ioc->cmd == WLC_SET_VAR)
				DHD_ERROR(("%s: WLC_SET_VAR: %s, ret = %d\n",
					__FUNCTION__, (char *)ioc->buf, ret));
#ifdef CUSTOMER_HW_ONE
			else if ((ioc->cmd == WLC_GET_BSSID) &&
				(ret == BCME_NOTASSOCIATED))
				DHD_INFO(("%s: WLC_GET_BSSID: ret = %d, "
					"Not associated yet!\n",
					 __FUNCTION__, ret));
#endif
			else
				DHD_ERROR(("%s: WLC_IOCTL: cmd: %d, ret = %d\n",
					__FUNCTION__, ioc->cmd, ret));
		}
#endif 
	}

	return ret;
}

uint wl_get_port_num(wl_io_pport_t *io_pport)
{
	return 0;
}

static int
dhd_iovar_parse_bssidx(dhd_pub_t *dhd_pub, char *params, int *idx, char **val)
{
	char *prefix = "bsscfg:";
	uint32	bssidx;

	if (!(strncmp(params, prefix, strlen(prefix)))) {
		
		char *p = (char *)params + strlen(prefix);

		
		while (*p != '\0')
			p++;
		
		p = p + 1;
		bcopy(p, &bssidx, sizeof(uint32));
		
		bssidx = dhd_bssidx2idx(dhd_pub, bssidx);

		if (bssidx >= DHD_MAX_IFS) {
			DHD_ERROR(("%s Wrong bssidx provided\n", __FUNCTION__));
			return BCME_ERROR;
		}

		
		p += sizeof(uint32);
		*val = p;
		*idx = bssidx;
	} else {
		DHD_ERROR(("%s: bad parameter for per bss iovar\n", __FUNCTION__));
		return BCME_ERROR;
	}

	return BCME_OK;
}

static int
dhd_doiovar(dhd_pub_t *dhd_pub, const bcm_iovar_t *vi, uint32 actionid, const char *name,
            void *params, int plen, void *arg, int len, int val_size)
{
	int bcmerror = 0;
	int32 int_val = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	DHD_TRACE(("%s: actionid = %d; name %s\n", __FUNCTION__, actionid, name));

	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid))) != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	switch (actionid) {
	case IOV_GVAL(IOV_VERSION):
		
		bcm_strncpy_s((char*)arg, len, dhd_version, len);
		break;

	case IOV_GVAL(IOV_MSGLEVEL):
		int_val = (int32)dhd_msg_level;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_MSGLEVEL):
#ifdef WL_CFG80211
		
		if (int_val & DHD_WL_VAL2)
			wl_cfg80211_enable_trace(TRUE, int_val & (~DHD_WL_VAL2));
		else if (int_val & DHD_WL_VAL)
			wl_cfg80211_enable_trace(FALSE, WL_DBG_DBG);
		if (!(int_val & DHD_WL_VAL2))
#endif 
		dhd_msg_level = int_val;
		break;
	case IOV_GVAL(IOV_BCMERRORSTR):
		bcm_strncpy_s((char *)arg, len, bcmerrorstr(dhd_pub->bcmerror), BCME_STRLEN);
		((char *)arg)[BCME_STRLEN - 1] = 0x00;
		break;

	case IOV_GVAL(IOV_BCMERROR):
		int_val = (int32)dhd_pub->bcmerror;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_WDTICK):
		int_val = (int32)dhd_watchdog_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WDTICK):
		if (!dhd_pub->up) {
			bcmerror = BCME_NOTUP;
			break;
		}
		dhd_os_wd_timer(dhd_pub, (uint)int_val);
		break;

	case IOV_GVAL(IOV_DUMP):
		bcmerror = dhd_dump(dhd_pub, arg, len);
		break;

#ifdef DHD_DEBUG
	case IOV_GVAL(IOV_DCONSOLE_POLL):
		int_val = (int32)dhd_console_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DCONSOLE_POLL):
		dhd_console_ms = (uint)int_val;
		break;

	case IOV_SVAL(IOV_CONS):
		if (len > 0)
			bcmerror = dhd_bus_console_in(dhd_pub, arg, len - 1);
		break;
#endif 

	case IOV_SVAL(IOV_CLEARCOUNTS):
		dhd_pub->tx_packets = dhd_pub->rx_packets = 0;
		dhd_pub->tx_errors = dhd_pub->rx_errors = 0;
		dhd_pub->tx_ctlpkts = dhd_pub->rx_ctlpkts = 0;
		dhd_pub->tx_ctlerrs = dhd_pub->rx_ctlerrs = 0;
		dhd_pub->tx_dropped = 0;
		dhd_pub->rx_dropped = 0;
		dhd_pub->rx_readahead_cnt = 0;
		dhd_pub->tx_realloc = 0;
		dhd_pub->wd_dpc_sched = 0;
		memset(&dhd_pub->dstats, 0, sizeof(dhd_pub->dstats));
		dhd_bus_clearcounts(dhd_pub);
#ifdef PROP_TXSTATUS
		
		dhd_wlfc_clear_counts(dhd_pub);
#endif 
		break;


	case IOV_GVAL(IOV_IOCTLTIMEOUT): {
		int_val = (int32)dhd_os_get_ioctl_resp_timeout();
		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_IOCTLTIMEOUT): {
		if (int_val <= 0)
			bcmerror = BCME_BADARG;
		else
			dhd_os_set_ioctl_resp_timeout((unsigned int)int_val);
		break;
	}

#ifdef WLBTAMP
	case IOV_SVAL(IOV_HCI_CMD): {
		amp_hci_cmd_t *cmd = (amp_hci_cmd_t *)arg;

		
		if (len < HCI_CMD_PREAMBLE_SIZE)
			return BCME_BUFTOOSHORT;

		
		if (len < (int)(HCI_CMD_PREAMBLE_SIZE + cmd->plen))
			return BCME_BUFTOOSHORT;

		dhd_bta_docmd(dhd_pub, cmd, len);
		break;
	}

	case IOV_SVAL(IOV_HCI_ACL_DATA): {
		amp_hci_ACL_data_t *ACL_data = (amp_hci_ACL_data_t *)arg;

		
		if (len < HCI_ACL_DATA_PREAMBLE_SIZE)
			return BCME_BUFTOOSHORT;

		
		if (len < (int)(HCI_ACL_DATA_PREAMBLE_SIZE + ACL_data->dlen))
			return BCME_BUFTOOSHORT;

		dhd_bta_tx_hcidata(dhd_pub, ACL_data, len);
		break;
	}
#endif 

#ifdef PROP_TXSTATUS
	case IOV_GVAL(IOV_PROPTXSTATUS_ENABLE): {
		bool wlfc_enab = FALSE;
		bcmerror = dhd_wlfc_get_enable(dhd_pub, &wlfc_enab);
		if (bcmerror != BCME_OK)
			goto exit;
		int_val = wlfc_enab ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_PROPTXSTATUS_ENABLE): {
		bool wlfc_enab = FALSE;
		bcmerror = dhd_wlfc_get_enable(dhd_pub, &wlfc_enab);
		if (bcmerror != BCME_OK)
			goto exit;

		
		if (wlfc_enab == (int_val == 0 ? FALSE : TRUE))
			goto exit;

		if (int_val == TRUE)
			bcmerror = dhd_wlfc_init(dhd_pub);
		else
			bcmerror = dhd_wlfc_deinit(dhd_pub);

		break;
	}
	case IOV_GVAL(IOV_PROPTXSTATUS_MODE):
		bcmerror = dhd_wlfc_get_mode(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_MODE):
		dhd_wlfc_set_mode(dhd_pub, int_val);
		break;
#ifdef QMONITOR
	case IOV_GVAL(IOV_QMON_TIME_THRES): {
		int_val = dhd_qmon_thres(dhd_pub, FALSE, 0);
		bcopy(&int_val, arg, val_size);
		break;
	}

	case IOV_SVAL(IOV_QMON_TIME_THRES): {
		dhd_qmon_thres(dhd_pub, TRUE, int_val);
		break;
	}

	case IOV_GVAL(IOV_QMON_TIME_PERCENT): {
		int_val = dhd_qmon_getpercent(dhd_pub);
		bcopy(&int_val, arg, val_size);
		break;
	}
#endif 

	case IOV_GVAL(IOV_PROPTXSTATUS_MODULE_IGNORE):
		bcmerror = dhd_wlfc_get_module_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_MODULE_IGNORE):
		dhd_wlfc_set_module_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_CREDIT_IGNORE):
		bcmerror = dhd_wlfc_get_credit_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_CREDIT_IGNORE):
		dhd_wlfc_set_credit_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_TXSTATUS_IGNORE):
		bcmerror = dhd_wlfc_get_txstatus_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_TXSTATUS_IGNORE):
		dhd_wlfc_set_txstatus_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_RXPKT_CHK):
		bcmerror = dhd_wlfc_get_rxpkt_chk(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_RXPKT_CHK):
		dhd_wlfc_set_rxpkt_chk(dhd_pub, int_val);
		break;

#endif 

	case IOV_GVAL(IOV_BUS_TYPE):
		
#ifdef BCMDHDUSB
		int_val = BUS_TYPE_USB;
#endif
#ifdef BCMSDIO
		int_val = BUS_TYPE_SDIO;
#endif
#ifdef PCIE_FULL_DONGLE
		int_val = BUS_TYPE_PCIE;
#endif
		bcopy(&int_val, arg, val_size);
		break;


#ifdef WLMEDIA_HTSF
	case IOV_GVAL(IOV_WLPKTDLYSTAT_SZ):
		int_val = dhd_pub->htsfdlystat_sz;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WLPKTDLYSTAT_SZ):
		dhd_pub->htsfdlystat_sz = int_val & 0xff;
		DHD_ERROR(("Setting tsfdlystat_sz:%d\n", dhd_pub->htsfdlystat_sz));
		break;
#endif
	case IOV_SVAL(IOV_CHANGEMTU):
		int_val &= 0xffff;
		bcmerror = dhd_change_mtu(dhd_pub, int_val, 0);
		break;

	case IOV_GVAL(IOV_HOSTREORDER_FLOWS):
	{
		uint i = 0;
		uint8 *ptr = (uint8 *)arg;
		uint8 count = 0;

		ptr++;
		for (i = 0; i < WLHOST_REORDERDATA_MAXFLOWS; i++) {
			if (dhd_pub->reorder_bufs[i] != NULL) {
				*ptr = dhd_pub->reorder_bufs[i]->flow_id;
				ptr++;
				count++;
			}
		}
		ptr = (uint8 *)arg;
		*ptr = count;
		break;
	}
#ifdef DHDTCPACK_SUPPRESS
	case IOV_GVAL(IOV_TCPACK_SUPPRESS): {
		int_val = (uint32)dhd_pub->tcpack_sup_mode;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_TCPACK_SUPPRESS): {
		bcmerror = dhd_tcpack_suppress_set(dhd_pub, (uint8)int_val);
		break;
	}
#endif 
#ifdef DHD_WMF
	case IOV_GVAL(IOV_WMF_BSS_ENAB): {
		uint32	bssidx;
		dhd_wmf_t *wmf;
		char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: wmf_bss_enable: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		wmf = dhd_wmf_conf(dhd_pub, bssidx);
		int_val = wmf->wmf_enable ? 1 :0;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_WMF_BSS_ENAB): {
		
		uint32	bssidx;
		dhd_wmf_t *wmf;
		char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: wmf_bss_enable: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		ASSERT(val);
		bcopy(val, &int_val, sizeof(uint32));
		wmf = dhd_wmf_conf(dhd_pub, bssidx);
		if (wmf->wmf_enable == int_val)
			break;
		if (int_val) {
			
			if (dhd_wmf_instance_add(dhd_pub, bssidx) != BCME_OK) {
				DHD_ERROR(("%s: Error in creating WMF instance\n",
				__FUNCTION__));
				break;
			}
			if (dhd_wmf_start(dhd_pub, bssidx) != BCME_OK) {
				DHD_ERROR(("%s: Failed to start WMF\n", __FUNCTION__));
				break;
			}
			wmf->wmf_enable = TRUE;
		} else {
			
			wmf->wmf_enable = FALSE;
			dhd_wmf_stop(dhd_pub, bssidx);
			dhd_wmf_instance_del(dhd_pub, bssidx);
		}
		break;
	}
	case IOV_GVAL(IOV_WMF_UCAST_IGMP):
		int_val = dhd_pub->wmf_ucast_igmp ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_WMF_UCAST_IGMP):
		if (dhd_pub->wmf_ucast_igmp == int_val)
			break;

		if (int_val >= OFF && int_val <= ON)
			dhd_pub->wmf_ucast_igmp = int_val;
		else
			bcmerror = BCME_RANGE;
		break;
	case IOV_GVAL(IOV_WMF_MCAST_DATA_SENDUP):
		int_val = dhd_wmf_mcast_data_sendup(dhd_pub, 0, FALSE, FALSE);
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_WMF_MCAST_DATA_SENDUP):
		dhd_wmf_mcast_data_sendup(dhd_pub, 0, TRUE, int_val);
		break;

#ifdef WL_IGMP_UCQUERY
	case IOV_GVAL(IOV_WMF_UCAST_IGMP_QUERY):
		int_val = dhd_pub->wmf_ucast_igmp_query ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_WMF_UCAST_IGMP_QUERY):
		if (dhd_pub->wmf_ucast_igmp_query == int_val)
			break;

		if (int_val >= OFF && int_val <= ON)
			dhd_pub->wmf_ucast_igmp_query = int_val;
		else
			bcmerror = BCME_RANGE;
		break;
#endif 
#ifdef DHD_UCAST_UPNP
	case IOV_GVAL(IOV_WMF_UCAST_UPNP):
		int_val = dhd_pub->wmf_ucast_upnp ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_WMF_UCAST_UPNP):
		if (dhd_pub->wmf_ucast_upnp == int_val)
			break;

		if (int_val >= OFF && int_val <= ON)
			dhd_pub->wmf_ucast_upnp = int_val;
		else
			bcmerror = BCME_RANGE;
		break;
#endif 
#endif 


#ifdef DHD_UNICAST_DHCP
	case IOV_GVAL(IOV_DHCP_UNICAST):
		int_val = dhd_pub->dhcp_unicast;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_DHCP_UNICAST):
		if (dhd_pub->dhcp_unicast == int_val)
			break;

		if (int_val >= OFF || int_val <= ON) {
			dhd_pub->dhcp_unicast = int_val;
		} else {
			bcmerror = BCME_RANGE;
		}
		break;
#endif 
#ifdef DHD_L2_FILTER
	case IOV_GVAL(IOV_BLOCK_PING):
		int_val = dhd_pub->block_ping;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_BLOCK_PING):
		if (dhd_pub->block_ping == int_val)
			break;
		if (int_val >= OFF || int_val <= ON) {
			dhd_pub->block_ping = int_val;
		} else {
			bcmerror = BCME_RANGE;
		}
		break;
	case IOV_GVAL(IOV_TRACE_PING):
		int_val = dhd_pub->trace_ping;
		bcopy(&int_val, arg, val_size);
		break;
	case IOV_SVAL(IOV_TRACE_PING):
		if (dhd_pub->trace_ping == int_val)
			break;
		if (int_val >= OFF || int_val <= ON) {
			dhd_pub->trace_ping = int_val;
		} else {
			bcmerror = BCME_RANGE;
		}
		break;
#endif 

	case IOV_GVAL(IOV_AP_ISOLATE): {
		uint32	bssidx;
		char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: ap isoalate: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		int_val = dhd_get_ap_isolate(dhd_pub, bssidx);
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_AP_ISOLATE): {
		uint32	bssidx;
		char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: ap isolate: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		ASSERT(val);
		bcopy(val, &int_val, sizeof(uint32));
		dhd_set_ap_isolate(dhd_pub, bssidx, int_val);
		break;
	}

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}

exit:
	DHD_TRACE(("%s: actionid %d, bcmerror %d\n", __FUNCTION__, actionid, bcmerror));
	return bcmerror;
}

void
dhd_store_conn_status(uint32 event, uint32 status, uint32 reason)
{
	if (!(event == WLC_E_SET_SSID && status == WLC_E_STATUS_FAIL &&
	      dhd_conn_event == WLC_E_PRUNE)) {
		dhd_conn_event = event;
		dhd_conn_status = status;
		dhd_conn_reason = reason;
	}
}

bool
dhd_prec_enq(dhd_pub_t *dhdp, struct pktq *q, void *pkt, int prec)
{
	void *p;
	int eprec = -1;		
	bool discard_oldest;

	if (!pktq_pfull(q, prec) && !pktq_full(q)) {
		pktq_penq(q, prec, pkt);
		return TRUE;
	}

	
	if (pktq_pfull(q, prec))
		eprec = prec;
	else if (pktq_full(q)) {
		p = pktq_peek_tail(q, &eprec);
		ASSERT(p);
		if (eprec > prec || eprec < 0)
			return FALSE;
	}

	
	if (eprec >= 0) {
		
		ASSERT(!pktq_pempty(q, eprec));
		discard_oldest = AC_BITMAP_TST(dhdp->wme_dp, eprec);
		if (eprec == prec && !discard_oldest)
			return FALSE;		
		
		p = discard_oldest ? pktq_pdeq(q, eprec) : pktq_pdeq_tail(q, eprec);
#ifdef CUSTOMER_HW_ONE
		if (p == NULL) {
			DHD_ERROR(("%s: pktq_penq() failed, oldest %d.",
				__FUNCTION__, discard_oldest));
			ASSERT(p);
		}
#else
		ASSERT(p);
#endif
#ifdef DHDTCPACK_SUPPRESS
		if (dhd_tcpack_check_xmit(dhdp, p) == BCME_ERROR) {
			DHD_ERROR(("%s %d: tcpack_suppress ERROR!!! Stop using it\n",
				__FUNCTION__, __LINE__));
			dhd_tcpack_suppress_set(dhdp, TCPACK_SUP_OFF);
		}
#endif 
		PKTFREE(dhdp->osh, p, TRUE);
	}

	
	p = pktq_penq(q, prec, pkt);
#ifdef CUSTOMER_HW_ONE
	if (p == NULL) {
		DHD_ERROR(("%s: pktq_penq() failed.", __FUNCTION__));
		ASSERT(p);
	}
#else
	ASSERT(p);
#endif
	return TRUE;
}

bool
dhd_prec_drop_pkts(dhd_pub_t *dhdp, struct pktq *pq, int prec, f_droppkt_t fn)
{
	struct pktq_prec *q = NULL;
	void *p, *prev = NULL, *next = NULL, *first = NULL, *last = NULL, *prev_first = NULL;
	pkt_frag_t frag_info;

	ASSERT(dhdp && pq);
	ASSERT(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];
	p = q->head;

	if (p == NULL)
		return FALSE;

	while (p) {
		frag_info = pkt_frag_info(dhdp->osh, p);
		if (frag_info == DHD_PKT_FRAG_NONE) {
			break;
		} else if (frag_info == DHD_PKT_FRAG_FIRST) {
			if (first) {
				
				last = prev;
				break;
			} else {
				first = p;
				prev_first = prev;
			}
		} else if (frag_info == DHD_PKT_FRAG_LAST) {
			if (first) {
				last = p;
				break;
			}
		}

		prev = p;
		p = PKTLINK(p);
	}

	if ((p == NULL) || ((frag_info != DHD_PKT_FRAG_NONE) && !(first && last))) {
		
		prev = NULL;
		p = q->head;
		frag_info = 0;
	}

	if (frag_info == DHD_PKT_FRAG_NONE) {
		first = last = p;
		prev_first = prev;
	}

	p = first;
	while (p) {
		next = PKTLINK(p);
		q->len--;
		pq->len--;

		PKTSETLINK(p, NULL);

		if (fn)
			fn(dhdp, prec, p, TRUE);

		if (p == last)
			break;

		p = next;
	}

	if (prev_first == NULL) {
		if ((q->head = next) == NULL)
			q->tail = NULL;
	} else {
		PKTSETLINK(prev_first, next);
		if (!next)
			q->tail = prev_first;
	}

	return TRUE;
}

static int
dhd_iovar_op(dhd_pub_t *dhd_pub, const char *name,
	void *params, int plen, void *arg, int len, bool set)
{
	int bcmerror = 0;
	int val_size;
	const bcm_iovar_t *vi = NULL;
	uint32 actionid;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(name);
	ASSERT(len >= 0);

	
	ASSERT(set || (arg && len));

	
	ASSERT(!set || (!params && !plen));

	if ((vi = bcm_iovar_lookup(dhd_iovars, name)) == NULL) {
		bcmerror = BCME_UNSUPPORTED;
		goto exit;
	}

	DHD_CTL(("%s: %s %s, len %d plen %d\n", __FUNCTION__,
		name, (set ? "set" : "get"), len, plen));

	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);

	bcmerror = dhd_doiovar(dhd_pub, vi, actionid, name, params, plen, arg, len, val_size);

exit:
	return bcmerror;
}

int
dhd_ioctl(dhd_pub_t * dhd_pub, dhd_ioctl_t *ioc, void * buf, uint buflen)
{
	int bcmerror = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (!buf) {
		return BCME_BADARG;
	}

	switch (ioc->cmd) {
	case DHD_GET_MAGIC:
		if (buflen < sizeof(int))
			bcmerror = BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_MAGIC;
		break;

	case DHD_GET_VERSION:
		if (buflen < sizeof(int))
			bcmerror = BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_VERSION;
		break;

	case DHD_GET_VAR:
	case DHD_SET_VAR: {
		char *arg;
		uint arglen;

		
		for (arg = buf, arglen = buflen; *arg && arglen; arg++, arglen--)
			;

		if (*arg) {
			bcmerror = BCME_BUFTOOSHORT;
			break;
		}

		
		arg++, arglen--;

		
		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_iovar_op(dhd_pub, buf, arg, arglen,
			buf, buflen, IOV_GET);
		else
			bcmerror = dhd_iovar_op(dhd_pub, buf, NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		
		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf, arg,
				arglen, buf, buflen, IOV_GET);
		else
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf,
				NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		
		if (ioc->cmd == DHD_GET_VAR) {
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
				arg, arglen, buf, buflen, IOV_GET);
		} else {
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
				NULL, 0, arg, arglen, IOV_SET);
		}

		break;
	}

	default:
		bcmerror = BCME_UNSUPPORTED;
	}

	return bcmerror;
}

#ifdef SHOW_EVENTS
#ifdef SHOW_LOGTRACE

#define AVOID_BYTE 64
#define MAX_NO_OF_ARG 16

static int
check_event_log_sequence_number(uint32 seq_no)
{
	int32 diff;
	uint32 ret;
	static uint32 logtrace_seqnum_prev = 0;

	diff = ntoh32(seq_no)-logtrace_seqnum_prev;
	switch (diff)
	{
		case 0:
			ret = -1; 
			break;

		case 1:
			ret =0; 
			break;

		default:
			if ((ntoh32(seq_no) == 0) &&
				(logtrace_seqnum_prev == 0xFFFFFFFF) ) { 
					ret = 0;
			} else {

				if (diff > 0) {
					DHD_EVENT(("WLC_E_TRACE:"
						"Event lost (log) seqnum %d nblost %d\n",
						ntoh32(seq_no), (diff-1)));
				} else {
					DHD_EVENT(("WLC_E_TRACE:"
						"Event Packets coming out of order!!\n"));
				}
				ret = 0;
			}
	}

	logtrace_seqnum_prev = ntoh32(seq_no);

	return ret;
}
#endif 

static void
wl_show_host_event(dhd_pub_t *dhd_pub, wl_event_msg_t *event, void *event_data,
	void *raw_event_ptr, char *eventmask)
{
	uint i, status, reason;
	bool group = FALSE, flush_txq = FALSE, link = FALSE;
	const char *auth_str;
	const char *event_name;
	uchar *buf;
	char err_msg[256], eabuf[ETHER_ADDR_STR_LEN];
	uint event_type, flags, auth_type, datalen;

	event_type = ntoh32(event->event_type);
	flags = ntoh16(event->flags);
	status = ntoh32(event->status);
	reason = ntoh32(event->reason);
	BCM_REFERENCE(reason);
	auth_type = ntoh32(event->auth_type);
	datalen = ntoh32(event->datalen);

	
	snprintf(eabuf, sizeof(eabuf), "%02x:%02x:%02x:%02x:%02x:%02x",
	        (uchar)event->addr.octet[0]&0xff,
	        (uchar)event->addr.octet[1]&0xff,
	        (uchar)event->addr.octet[2]&0xff,
	        (uchar)event->addr.octet[3]&0xff,
	        (uchar)event->addr.octet[4]&0xff,
	        (uchar)event->addr.octet[5]&0xff);

	event_name = bcmevent_get_name(event_type);
	BCM_REFERENCE(event_name);

	if (flags & WLC_EVENT_MSG_LINK)
		link = TRUE;
	if (flags & WLC_EVENT_MSG_GROUP)
		group = TRUE;
	if (flags & WLC_EVENT_MSG_FLUSHTXQ)
		flush_txq = TRUE;

	switch (event_type) {
	case WLC_E_START:
	case WLC_E_DEAUTH:
	case WLC_E_DISASSOC:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:

		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC:
	case WLC_E_REASSOC:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, SUCCESS\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, TIMEOUT\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, FAILURE, reason %d\n",
			       event_name, eabuf, (int)reason));
		} else {
			DHD_EVENT(("MACEVENT: %s, MAC %s, unexpected status %d\n",
			       event_name, eabuf, (int)status));
		}
		break;

	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
		DHD_EVENT(("MACEVENT: %s, MAC %s, reason %d\n", event_name, eabuf, (int)reason));
		break;

	case WLC_E_AUTH:
	case WLC_E_AUTH_IND:
		if (auth_type == DOT11_OPEN_SYSTEM)
			auth_str = "Open System";
		else if (auth_type == DOT11_SHARED_KEY)
			auth_str = "Shared Key";
		else {
			snprintf(err_msg, sizeof(err_msg), "AUTH unknown: %d", (int)auth_type);
			auth_str = err_msg;
		}
		if (event_type == WLC_E_AUTH_IND) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s\n", event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, SUCCESS\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, TIMEOUT\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, FAILURE, reason %d\n",
			       event_name, eabuf, auth_str, (int)reason));
		}
		BCM_REFERENCE(auth_str);

		break;

	case WLC_E_JOIN:
	case WLC_E_ROAM:
	case WLC_E_SET_SSID:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, failed\n", event_name));
		} else if (status == WLC_E_STATUS_NO_NETWORKS) {
			DHD_EVENT(("MACEVENT: %s, no networks found\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, unexpected status %d\n",
				event_name, (int)status));
		}
		break;

	case WLC_E_BEACON_RX:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, SUCCESS\n", event_name));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, FAIL\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, status %d\n", event_name, status));
		}
		break;

	case WLC_E_LINK:
		DHD_EVENT(("MACEVENT: %s %s\n", event_name, link?"UP":"DOWN"));
		BCM_REFERENCE(link);
		break;

	case WLC_E_MIC_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s, Group %d, Flush %d\n",
		       event_name, eabuf, group, flush_txq));
		BCM_REFERENCE(group);
		BCM_REFERENCE(flush_txq);
		break;

	case WLC_E_ICV_ERROR:
	case WLC_E_UNICAST_DECODE_ERROR:
	case WLC_E_MULTICAST_DECODE_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n",
		       event_name, eabuf));
		break;

	case WLC_E_TXFAIL:
		DHD_EVENT(("MACEVENT: %s, RA %s\n", event_name, eabuf));
		break;

	case WLC_E_SCAN_COMPLETE:
	case WLC_E_ASSOC_REQ_IE:
	case WLC_E_ASSOC_RESP_IE:
	case WLC_E_PMKID_CACHE:
		DHD_EVENT(("MACEVENT: %s\n", event_name));
		break;

	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
	case WLC_E_PFN_SCAN_COMPLETE:
	case WLC_E_PFN_SCAN_NONE:
	case WLC_E_PFN_SCAN_ALLGONE:
		DHD_EVENT(("PNOEVENT: %s\n", event_name));
		break;

	case WLC_E_PSK_SUP:
	case WLC_E_PRUNE:
		DHD_EVENT(("MACEVENT: %s, status %d, reason %d\n",
		           event_name, (int)status, (int)reason));
		break;

#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME:
		DHD_TRACE(("MACEVENT: %s Bssid %s\n", event_name, eabuf));
		break;
#endif 

#ifdef SHOW_LOGTRACE
	case WLC_E_TRACE:
	{
		msgtrace_hdr_t hdr;
		uint32 nblost;
		uint8 count;
		char *s, *p;
		static uint32 seqnum_prev = 0;
		uint32 *record = NULL;
		uint32 *log_ptr =  NULL;
		uint32 writeindex = 0;
		event_log_hdr_t event_hdr;
		int no_of_fmts = 0;
		char *fmt = NULL;
		dhd_event_log_t *raw_event = (dhd_event_log_t *) raw_event_ptr;

		buf = (uchar *) event_data;
		memcpy(&hdr, buf, MSGTRACE_HDRLEN);

		if (hdr.version != MSGTRACE_VERSION) {
			DHD_EVENT(("\nMACEVENT: %s [unsupported version --> "
				"dhd version:%d dongle version:%d]\n",
				event_name, MSGTRACE_VERSION, hdr.version));
			
			datalen = 0;
			break;
		}

		if (hdr.trace_type == MSGTRACE_HDR_TYPE_MSG) {
			
			buf[MSGTRACE_HDRLEN + ntoh16(hdr.len)] = '\0';

			if (ntoh32(hdr.discarded_bytes) || ntoh32(hdr.discarded_printf)) {
				DHD_EVENT(("WLC_E_TRACE: [Discarded traces in dongle -->"
					"discarded_bytes %d discarded_printf %d]\n",
					ntoh32(hdr.discarded_bytes),
					ntoh32(hdr.discarded_printf)));
			}

			nblost = ntoh32(hdr.seqnum) - seqnum_prev - 1;
			if (nblost > 0) {
				DHD_EVENT(("WLC_E_TRACE:"
					"[Event lost (msg) --> seqnum %d nblost %d\n",
					ntoh32(hdr.seqnum), nblost));
			}
			seqnum_prev = ntoh32(hdr.seqnum);

			p = (char *)&buf[MSGTRACE_HDRLEN];
			while (*p != '\0' && (s = strstr(p, "\n")) != NULL) {
				*s = '\0';
				DHD_EVENT(("%s\n", p));
				p = s+1;
			}
			if (*p)
				DHD_EVENT(("%s", p));

			
			datalen = 0;

		} else if (hdr.trace_type == MSGTRACE_HDR_TYPE_LOG) {
			
			uint32 timestamp, w, malloc_len;

			if (check_event_log_sequence_number(hdr.seqnum)) {

				DHD_EVENT(("%s: WLC_E_TRACE:"
					"[Event duplicate (log) %d] dropping!!\n",
					__FUNCTION__, hdr.seqnum));
				return; 
			}

			p = (char *)&buf[MSGTRACE_HDRLEN];
			datalen -= MSGTRACE_HDRLEN;
			w = ntoh32((uint32)*p);
			p += 4;
			datalen -= 4;
			timestamp = ntoh32((uint32)*p);
			BCM_REFERENCE(timestamp);
			BCM_REFERENCE(w);

			DHD_EVENT(("timestamp %x%x\n", timestamp, w));

			if (raw_event->fmts) {
				malloc_len = datalen+ AVOID_BYTE;
				record = (uint32 *)MALLOC(dhd_pub->osh, malloc_len);
				if (record == NULL) {
					DHD_EVENT(("MSGTRACE_HDR_TYPE_LOG:"
						"malloc failed\n"));
					return;
				}
				log_ptr = (uint32 *) (p + datalen);
				writeindex = datalen/4;

				if (record) {
					while (datalen > 4) {
						log_ptr--;
						datalen -= 4;
						event_hdr.t = *log_ptr;
						if (log_ptr - (uint32 *) p < event_hdr.count) {
								break;
						}
						if (event_hdr.tag ==  EVENT_LOG_TAG_NULL) {
							continue;
						}
						if (event_hdr.tag == EVENT_LOG_TAG_TS) {
							datalen -= 12;
							log_ptr = log_ptr - 3;
							continue;
						}

						log_ptr[0] = event_hdr.t;
						if (event_hdr.count > MAX_NO_OF_ARG) {
							break;
						}
						log_ptr -= event_hdr.count;

						writeindex = writeindex - event_hdr.count;
						record[writeindex++] = event_hdr.t;
						for (count = 0; count < (event_hdr.count-1);
							count++) {
							record[writeindex++] = log_ptr[count];
						}
						writeindex = writeindex - event_hdr.count;
						datalen = datalen - (event_hdr.count * 4);
						no_of_fmts++;
					}
				}

				while (no_of_fmts--)
				{
					event_log_hdr_t event_hdr;
					event_hdr.t = record[writeindex];

					if ((event_hdr.fmt_num>>2) < raw_event->num_fmts) {
						fmt = raw_event->fmts[event_hdr.fmt_num>>2];
						DHD_EVENT((fmt,
							record[writeindex + 1],
							record[writeindex + 2],
							record[writeindex + 3],
							record[writeindex + 4],
							record[writeindex + 5],
							record[writeindex + 6],
							record[writeindex + 7],
							record[writeindex + 8],
							record[writeindex + 9],
							record[writeindex + 10],
							record[writeindex + 11],
							record[writeindex + 12],
							record[writeindex + 13],
							record[writeindex + 14],
							record[writeindex + 15],
							record[writeindex + 16]));

						if (fmt[strlen(fmt) - 1] != '\n') {
							
							DHD_EVENT(("\n"));
						}
					}

					writeindex = writeindex + event_hdr.count;
				}

				if (record) {
					MFREE(dhd_pub->osh, record, malloc_len);
					record = NULL;
				}
			} else {
				while (datalen > 4) {
					p += 4;
					datalen -= 4;
					
					DHD_EVENT((" %8.8x", *((uint32 *) p)));
				}
				DHD_EVENT(("\n"));
			}
			datalen = 0;
		}
		break;
	}
#endif 

	case WLC_E_RSSI:
		DHD_EVENT(("MACEVENT: %s %d\n", event_name, ntoh32(*((int *)event_data))));
		break;

	case WLC_E_SERVICE_FOUND:
	case WLC_E_P2PO_ADD_DEVICE:
	case WLC_E_P2PO_DEL_DEVICE:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

#ifdef BT_WIFI_HANDOBER
	case WLC_E_BT_WIFI_HANDOVER_REQ:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;
#endif

	default:
		DHD_EVENT(("MACEVENT: %s %d, MAC %s, status %d, reason %d, auth %d\n",
		       event_name, event_type, eabuf, (int)status, (int)reason,
		       (int)auth_type));
		break;
	}

	
	if (DHD_BYTES_ON() && DHD_EVENT_ON() && datalen) {
		buf = (uchar *) event_data;
		BCM_REFERENCE(buf);
		DHD_EVENT((" data (%d) : ", datalen));
		for (i = 0; i < datalen; i++)
			DHD_EVENT((" 0x%02x ", *buf++));
		DHD_EVENT(("\n"));
	}
}
#endif 

int
wl_host_event(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata,
	wl_event_msg_t *event, void **data_ptr, void *raw_event)
{
	
	bcm_event_t *pvt_data = (bcm_event_t *)pktdata;
	uint8 *event_data;
	uint32 type, status, datalen;
	uint16 flags;
	int evlen;
	int hostidx;

	if (bcmp(BRCM_OUI, &pvt_data->bcm_hdr.oui[0], DOT11_OUI_LEN)) {
		DHD_ERROR(("%s: mismatched OUI, bailing\n", __FUNCTION__));
		
		
		
		
		
		return (BCME_ERROR);
	}

	
	if (ntoh16_ua((void *)&pvt_data->bcm_hdr.usr_subtype) != BCMILCP_BCM_SUBTYPE_EVENT) {
		DHD_ERROR(("%s: mismatched subtype, bailing\n", __FUNCTION__));
		return (BCME_ERROR);
	}

	*data_ptr = &pvt_data[1];
	event_data = *data_ptr;


	
	memcpy(event, &pvt_data->event, sizeof(wl_event_msg_t));

	type = ntoh32_ua((void *)&event->event_type);
	flags = ntoh16_ua((void *)&event->flags);
	status = ntoh32_ua((void *)&event->status);
	datalen = ntoh32_ua((void *)&event->datalen);
	evlen = datalen + sizeof(bcm_event_t);

	
	hostidx = dhd_ifidx2hostidx(dhd_pub->info, event->ifidx);

	switch (type) {
#ifdef PROP_TXSTATUS
	case WLC_E_FIFO_CREDIT_MAP:
		dhd_wlfc_enable(dhd_pub);
		dhd_wlfc_FIFOcreditmap_event(dhd_pub, event_data);
		WLFC_DBGMESG(("WLC_E_FIFO_CREDIT_MAP:(AC0,AC1,AC2,AC3),(BC_MC),(OTHER): "
			"(%d,%d,%d,%d),(%d),(%d)\n", event_data[0], event_data[1],
			event_data[2],
			event_data[3], event_data[4], event_data[5]));
		break;

	case WLC_E_BCMC_CREDIT_SUPPORT:
		dhd_wlfc_BCMCCredit_support_event(dhd_pub);
		break;
#endif

	case WLC_E_IF:
		{
		struct wl_event_data_if *ifevent = (struct wl_event_data_if *)event_data;

		
		if (ifevent->reserved & WLC_E_IF_FLAGS_BSSCFG_NOIF) {
			DHD_ERROR(("WLC_E_IF: NO_IF set, event Ignored\r\n"));
			return (BCME_UNSUPPORTED);
		}
#ifdef PCIE_FULL_DONGLE
		dhd_update_interface_flow_info(dhd_pub, ifevent->ifidx,
			ifevent->opcode, ifevent->role);
#endif
#ifdef PROP_TXSTATUS
		{
			uint8* ea = pvt_data->eth.ether_dhost;
			WLFC_DBGMESG(("WLC_E_IF: idx:%d, action:%s, iftype:%s, "
			              "[%02x:%02x:%02x:%02x:%02x:%02x]\n",
			              ifevent->ifidx,
			              ((ifevent->opcode == WLC_E_IF_ADD) ? "ADD":"DEL"),
			              ((ifevent->role == 0) ? "STA":"AP "),
			              ea[0], ea[1], ea[2], ea[3], ea[4], ea[5]));
			(void)ea;

			if (ifevent->opcode == WLC_E_IF_CHANGE)
				dhd_wlfc_interface_event(dhd_pub,
					eWLFC_MAC_ENTRY_ACTION_UPDATE,
					ifevent->ifidx, ifevent->role, ea);
			else
				dhd_wlfc_interface_event(dhd_pub,
					((ifevent->opcode == WLC_E_IF_ADD) ?
					eWLFC_MAC_ENTRY_ACTION_ADD : eWLFC_MAC_ENTRY_ACTION_DEL),
					ifevent->ifidx, ifevent->role, ea);

			
			if (ifevent->ifidx == 0)
				break;
		}
#endif 

		if (ifevent->ifidx > 0 && ifevent->ifidx < DHD_MAX_IFS) {
			if (ifevent->opcode == WLC_E_IF_ADD) {
				if (dhd_event_ifadd(dhd_pub->info, ifevent, event->ifname,
					event->addr.octet)) {

					DHD_ERROR(("%s: dhd_event_ifadd failed ifidx: %d  %s\n",
						__FUNCTION__, ifevent->ifidx, event->ifname));
					return (BCME_ERROR);
				}
			} else if (ifevent->opcode == WLC_E_IF_DEL) {
				dhd_event_ifdel(dhd_pub->info, ifevent, event->ifname,
					event->addr.octet);
			} else if (ifevent->opcode == WLC_E_IF_CHANGE) {
#ifdef WL_CFG80211
				wl_cfg80211_notify_ifchange(ifevent->ifidx,
					event->ifname, event->addr.octet, ifevent->bssidx);
#endif 
			}
		} else {
#if !defined(PROP_TXSTATUS) || !defined(PCIE_FULL_DONGLE)
			DHD_ERROR(("%s: Invalid ifidx %d for %s\n",
			           __FUNCTION__, ifevent->ifidx, event->ifname));
#endif 
		}
			
			*ifidx = hostidx;
			
			dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
		break;
	}

#ifdef WLMEDIA_HTSF
	case WLC_E_HTSFSYNC:
		htsf_update(dhd_pub->info, event_data);
		break;
#endif 
#if defined(NDISVER) && (NDISVER >= 0x0630)
	case WLC_E_NDIS_LINK:
		break;
#else
	case WLC_E_NDIS_LINK: {
		uint32 temp = hton32(WLC_E_LINK);

		memcpy((void *)(&pvt_data->event.event_type), &temp,
		       sizeof(pvt_data->event.event_type));
		break;
	}
#endif 
	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
		break;
#if defined(PNO_SUPPORT)
	case WLC_E_PFN_BSSID_NET_FOUND:
	case WLC_E_PFN_BSSID_NET_LOST:
	case WLC_E_PFN_BEST_BATCHING:
		dhd_pno_event_handler(dhd_pub, event, (void *)event_data);
		break;
#endif 
		
	case WLC_E_ASSOC_IND:
	case WLC_E_AUTH_IND:
	case WLC_E_REASSOC_IND:
		dhd_findadd_sta(dhd_pub, hostidx, &event->addr.octet);
		break;
	case WLC_E_LINK:
#ifdef PCIE_FULL_DONGLE
		if (dhd_update_interface_link_status(dhd_pub, (uint8)hostidx,
			(uint8)flags) != BCME_OK)
			break;
		if (!flags) {
			dhd_flow_rings_delete(dhd_pub, hostidx);
		}
		
#endif
	case WLC_E_DEAUTH:
	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC:
	case WLC_E_DISASSOC_IND:
		if (type != WLC_E_LINK) {
			dhd_del_sta(dhd_pub, hostidx, &event->addr.octet);
		}
		DHD_EVENT(("%s: Link event %d, flags %x, status %x\n",
		           __FUNCTION__, type, flags, status));
#ifdef PCIE_FULL_DONGLE
		if (type != WLC_E_LINK) {
			uint8 ifindex = (uint8)hostidx;
			uint8 role = dhd_flow_rings_ifindex2role(dhd_pub, ifindex);
			if (DHD_IF_ROLE_STA(role)) {
				dhd_flow_rings_delete(dhd_pub, ifindex);
			} else {
				dhd_flow_rings_delete_for_peer(dhd_pub, ifindex,
					&event->addr.octet[0]);
			}
		}
#endif
		
	default:
		*ifidx = hostidx;
		
		dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
		DHD_TRACE(("%s: MAC event %d, flags %x, status %x\n",
		           __FUNCTION__, type, flags, status));
		BCM_REFERENCE(flags);
		BCM_REFERENCE(status);

		break;
	}

#ifdef SHOW_EVENTS
	wl_show_host_event(dhd_pub, event,
		(void *)event_data, raw_event, dhd_pub->enable_log);
#endif 

	return (BCME_OK);
}

void
wl_event_to_host_order(wl_event_msg_t * evt)
{
	evt->event_type = ntoh32(evt->event_type);
	evt->flags = ntoh16(evt->flags);
	evt->status = ntoh32(evt->status);
	evt->reason = ntoh32(evt->reason);
	evt->auth_type = ntoh32(evt->auth_type);
	evt->datalen = ntoh32(evt->datalen);
	evt->version = ntoh16(evt->version);
}

void
dhd_print_buf(void *pbuf, int len, int bytes_per_line)
{
#ifdef DHD_DEBUG
	int i, j = 0;
	unsigned char *buf = pbuf;
	char line[80] = "";

	if (bytes_per_line == 0) {
		bytes_per_line = len;
	}

	for (i = 0; i < len; i++) {
		snprintf(line, sizeof(line), "%s%2.2x", line, *buf++);
		j++;
		if (j == bytes_per_line) {
			snprintf(line, sizeof(line), "%s\n", line);
			printf("%s", line);
			j = 0;
			line[0] = '\0';
		} else {
			snprintf(line, sizeof(line), "%s:", line);
		}
	}
	if (j) {
		snprintf(line, sizeof(line), "%s\n", line);
		printf("%s", line);
	}
	printf("\n");
#endif 
}
#ifndef strtoul
#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))
#endif

#ifdef PKT_FILTER_SUPPORT
static int
wl_pattern_atoh(char *src, char *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 &&
	    strncmp(src, "0X", 2) != 0) {
		DHD_ERROR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + 2; 
	if (strlen(src) % 2 != 0) {
		DHD_ERROR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[3];
		bcm_strncpy_s(num, sizeof(num), src, 2);
		num[2] = '\0';
		dst[i] = (uint8)strtoul(num, NULL, 16);
		src += 2;
	}
	return i;
}

void
dhd_pktfilter_offload_enable(dhd_pub_t * dhd, char *arg, int enable, int master_mode)
{
	char				*argv[8];
	int					i = 0;
	const char			*str;
	int					buf_len;
	int					str_len;
	char				*arg_save = 0, *arg_org = 0;
	int					rc;
	char				buf[32] = {0};
	wl_pkt_filter_enable_t	enable_parm;
	wl_pkt_filter_enable_t	* pkt_filterp;

	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}
	arg_org = arg_save;
	memcpy(arg_save, arg, strlen(arg) + 1);

	argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_enable";
	str_len = strlen(str);
	bcm_strncpy_s(buf, sizeof(buf) - 1, str, sizeof(buf) - 1);
	buf[ sizeof(buf) - 1 ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_enable_t *)(buf + str_len + 1);

	
	enable_parm.id = htod32(strtoul(argv[i], NULL, 0));

	
	enable_parm.enable = htod32(enable);

	buf_len += sizeof(enable_parm);
	memcpy((char *)pkt_filterp,
	       &enable_parm,
	       sizeof(enable_parm));

	
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

	
	bcm_mkiovar("pkt_filter_mode", (char *)&master_mode, 4, buf, sizeof(buf));
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);
}

#define PATTERN_ITEMS 5
#ifdef PACKET_FILTER2
#define PATTERN_ELEMENT 2
#else
#define PATTERN_ELEMENT 1
#endif
#define MAX_PKTFILTER_ARGV (1 + PATTERN_ITEMS * PATTERN_ELEMENT + 1)

void
dhd_pktfilter_offload_set(dhd_pub_t * dhd, char *arg)
{
	const char			*str;
	wl_pkt_filter_t		*pkt_filterp;
#ifdef PACKET_FILTER2
	wl_pkt_filter_pattern_listel_t *pattern_listelp;
	uint32 patterns = 0;
#endif
#ifdef PACKET_FILTER2_DEBUG
	char *p;
#endif
	int					totsize = 0;
	int					buf_len;
	int					str_len;
	int					rc;
	uint32				mask_size;
	uint32				pattern_size;
	char				*argv[MAX_PKTFILTER_ARGV], * buf = 0;
	int					i = 0;
	char				*arg_save = 0, *arg_org = 0;
#define BUF_SIZE		2048

#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: enter max argv %d\n", __FUNCTION__, MAX_PKTFILTER_ARGV));
#endif
	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}

	arg_org = arg_save;

	if (!(buf = MALLOC(dhd->osh, BUF_SIZE))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}
	memset(buf, 0, BUF_SIZE);

	memcpy(arg_save, arg, strlen(arg) + 1);

	if (strlen(arg) > BUF_SIZE) {
		DHD_ERROR(("Not enough buffer %d < %d\n", (int)strlen(arg), (int)sizeof(buf)));
		goto fail;
	}

	do {
		argv[i] = bcmstrtok(&arg_save, " ", 0);
#ifdef PACKET_FILTER2_DEBUG
		if (argv[i] != NULL) {
			DHD_ERROR(("argv %d %s\n", i, argv[i]));
		} else {
			DHD_ERROR(("argv %d NULL parse done\n", i));
		}
#endif
	} while (argv[i++] && i < MAX_PKTFILTER_ARGV);
#ifdef PACKET_FILTER2
	patterns = i / PATTERN_ITEMS;
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("argvs %d patterns %d\n", i, patterns));
#endif
#endif 

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_add";
	str_len = strlen(str);
	bcm_strncpy_s(buf, BUF_SIZE, str, str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	
	pkt_filterp->id = htod32(strtoul(argv[i], NULL, 0));
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d id %d\n", __FUNCTION__, i, pkt_filterp->id));
#endif

#ifdef PACKET_FILTER2
	if (patterns > 1) {
		
		pkt_filterp->negate_match = htod32(0);
		
		pkt_filterp->type = htod32(WL_PKT_FILTER_TYPE_PATTERN_LIST_MATCH);
		pkt_filterp->u.patlist.list_cnt = patterns;
		
		pattern_listelp = pkt_filterp->u.patlist.patterns;
		totsize = WL_PKT_FILTER_PATTERN_LIST_FIXED_LEN;
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: PATTERN_LIST type %d cnt %d\n",
			__FUNCTION__, pkt_filterp->type, pkt_filterp->u.patlist.list_cnt));
#endif
	}
parse_pattern:
#endif 

	if (argv[++i] == NULL) {
		DHD_ERROR(("Polarity not provided\n"));
		goto fail;
	}

	
#ifdef PACKET_FILTER2
	if (patterns > 1) {
		pattern_listelp->match_flags = htod16(strtoul(argv[i], NULL, 0));
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: i %d match_flags %d\n",
			__FUNCTION__, i, pattern_listelp->match_flags));
#endif
	} else
#endif 
{
	pkt_filterp->negate_match = htod32(strtoul(argv[i], NULL, 0));
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d negate_match %d\n", __FUNCTION__, i, pkt_filterp->negate_match));
#endif
}

	if (argv[++i] == NULL) {
		DHD_ERROR(("Filter type not provided\n"));
		goto fail;
	}

	
#ifdef PACKET_FILTER2
	if (patterns > 1) {
		
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: i %d ignore type %ld\n",
			__FUNCTION__, i, strtoul(argv[i], NULL, 0)));
#endif
	} else
#endif 
{
	pkt_filterp->type = htod32(strtoul(argv[i], NULL, 0));
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d type %d\n", __FUNCTION__, i, pkt_filterp->type));
#endif
}

	if (argv[++i] == NULL) {
		DHD_ERROR(("Offset not provided\n"));
		goto fail;
	}

	
#ifdef PACKET_FILTER2
	if (patterns > 1) {
		
		pattern_listelp->rel_offs = htod16(strtoul(argv[i], NULL, 0));
		pattern_listelp->base_offs = htod16(0);
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: i %d rel_offs %d\n", __FUNCTION__, i, pattern_listelp->rel_offs));
#endif
	} else
#endif 
{
	pkt_filterp->u.pattern.offset = htod32(strtoul(argv[i], NULL, 0));
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d offset %d\n", __FUNCTION__, i, pkt_filterp->u.pattern.offset));
#endif
}

	if (argv[++i] == NULL) {
		DHD_ERROR(("Bitmask not provided\n"));
		goto fail;
	}

	
#ifdef PACKET_FILTER2
	if (patterns > 1) {
		mask_size =
			wl_pattern_atoh(argv[i], (char *) pattern_listelp->mask_and_data);
	} else
#endif
	mask_size =
		wl_pattern_atoh(argv[i], (char *) pkt_filterp->u.pattern.mask_and_pattern);
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d size %d mask %s\n", __FUNCTION__, i, mask_size, argv[i]));
#endif

	if (argv[++i] == NULL) {
		DHD_ERROR(("Pattern not provided\n"));
		goto fail;
	}

	
#ifdef PACKET_FILTER2
	if (patterns > 1) {
		pattern_size =
			wl_pattern_atoh(argv[i],
			(char *) &pattern_listelp->mask_and_data[mask_size]);
	} else
#endif
	pattern_size =
		wl_pattern_atoh(argv[i],
		(char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]);
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: i %d size %d pattern %s\n", __FUNCTION__, i, pattern_size, argv[i]));
#endif

	if (mask_size != pattern_size) {
		DHD_ERROR(("Mask and pattern not the same size\n"));
		goto fail;
	}

#ifdef PACKET_FILTER2
	if (patterns > 1) {
		pattern_listelp->size_bytes = htod16(mask_size);
		totsize += (WL_PKT_FILTER_PATTERN_LISTEL_FIXED_LEN + 2 * mask_size);
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: size_bytes %d\n", __FUNCTION__, pattern_listelp->size_bytes));
#endif
	} else
#endif 
{
	pkt_filterp->u.pattern.size_bytes = htod32(mask_size);
	totsize += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);
#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("%s: size_bytes %d\n", __FUNCTION__, pkt_filterp->u.pattern.size_bytes));
#endif
}

#ifdef PACKET_FILTER2
	if (i < PATTERN_ITEMS * patterns) {
		pattern_listelp = (wl_pkt_filter_pattern_listel_t *)
			(buf + str_len + 1 + WL_PKT_FILTER_FIXED_LEN + totsize);
		goto parse_pattern;
	} else if (patterns > 1) {
		pkt_filterp->u.patlist.totsize = htod32(totsize);
#ifdef PACKET_FILTER2_DEBUG
		DHD_ERROR(("%s: patlist totsize %d\n",
			__FUNCTION__, pkt_filterp->u.patlist.totsize));
#endif
	}
#endif 
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += totsize;

#ifdef PACKET_FILTER2_DEBUG
	DHD_ERROR(("totsize %d buf_len %d\n", totsize, buf_len));

	p = buf;
	for (i = 0; i < buf_len; i++, p++) {
		DHD_ERROR((" %02x \n", *p));
	}
#endif

	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;

	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);

	if (buf)
		MFREE(dhd->osh, buf, BUF_SIZE);
}

void dhd_pktfilter_offload_delete(dhd_pub_t *dhd, int id)
{
	char iovbuf[32];
	int ret;

	bcm_mkiovar("pkt_filter_delete", (char *)&id, 4, iovbuf, sizeof(iovbuf));
	ret = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	if (ret < 0) {
		DHD_ERROR(("%s: Failed to delete filter ID:%d, ret=%d\n",
			__FUNCTION__, id, ret));
	}
}
#endif 

#ifdef ARP_OFFLOAD_SUPPORT
void
dhd_arp_offload_set(dhd_pub_t * dhd, int arp_mode)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int iovar_len;
	int retcode;

	iovar_len = bcm_mkiovar("arp_ol", (char *)&arp_mode, 4, iovbuf, sizeof(iovbuf));
	if (!iovar_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return;
	}

	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iovar_len, TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to set ARP offload mode to 0x%x, retcode = %d\n",
			__FUNCTION__, arp_mode, retcode));
	else
		DHD_TRACE(("%s: successfully set ARP offload mode to 0x%x\n",
			__FUNCTION__, arp_mode));
}

void
dhd_arp_offload_enable(dhd_pub_t * dhd, int arp_enable)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int iovar_len;
	int retcode;

	iovar_len = bcm_mkiovar("arpoe", (char *)&arp_enable, 4, iovbuf, sizeof(iovbuf));
	if (!iovar_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return;
	}

	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iovar_len, TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to enabe ARP offload to %d, retcode = %d\n",
			__FUNCTION__, arp_enable, retcode));
	else
		DHD_TRACE(("%s: successfully enabed ARP offload to %d\n",
			__FUNCTION__, arp_enable));
	if (arp_enable) {
		uint32 version;
		bcm_mkiovar("arp_version", 0, 0, iovbuf, sizeof(iovbuf));
		retcode = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, iovbuf, sizeof(iovbuf), FALSE, 0);
		if (retcode) {
			DHD_INFO(("%s: fail to get version (maybe version 1:retcode = %d\n",
				__FUNCTION__, retcode));
			dhd->arp_version = 1;
		}
		else {
			memcpy(&version, iovbuf, sizeof(version));
			DHD_INFO(("%s: ARP Version= %x\n", __FUNCTION__, version));
			dhd->arp_version = version;
		}
	}
}

void
dhd_aoe_arp_clr(dhd_pub_t *dhd, int idx)
{
	int ret = 0;
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];

	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;

	iov_len = bcm_mkiovar("arp_table_clear", 0, 0, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return;
	}
	if ((ret  = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx)) < 0)
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
}

void
dhd_aoe_hostip_clr(dhd_pub_t *dhd, int idx)
{
	int ret = 0;
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];

	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;

	iov_len = bcm_mkiovar("arp_hostip_clear", 0, 0, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return;
	}
	if ((ret  = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx)) < 0)
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
}

void
dhd_arp_offload_add_ip(dhd_pub_t *dhd, uint32 ipaddr, int idx)
{
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int retcode;


	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;
	iov_len = bcm_mkiovar("arp_hostip", (char *)&ipaddr,
		sizeof(ipaddr), iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);

	if (retcode)
		DHD_TRACE(("%s: ARP ip addr add failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: sARP H ipaddr entry added \n",
		__FUNCTION__));
}

int
dhd_arp_get_arp_hostip_table(dhd_pub_t *dhd, void *buf, int buflen, int idx)
{
	int retcode, i;
	int iov_len;
	uint32 *ptr32 = buf;
	bool clr_bottom = FALSE;

	if (!buf)
		return -1;
	if (dhd == NULL) return -1;
	if (dhd->arp_version == 1)
		idx = 0;

	iov_len = bcm_mkiovar("arp_hostip", 0, 0, buf, buflen);
	BCM_REFERENCE(iov_len);
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, buf, buflen, FALSE, idx);

	if (retcode) {
		DHD_TRACE(("%s: ioctl WLC_GET_VAR error %d\n",
		__FUNCTION__, retcode));

		return -1;
	}

	
	for (i = 0; i < MAX_IPV4_ENTRIES; i++) {
		if (!clr_bottom) {
			if (*ptr32 == 0)
				clr_bottom = TRUE;
		} else {
			*ptr32 = 0;
		}
		ptr32++;
	}

	return 0;
}
#endif 

int
dhd_ndo_enable(dhd_pub_t * dhd, int ndo_enable)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int iov_len;
	int retcode;

	if (dhd == NULL)
		return -1;

	iov_len = bcm_mkiovar("ndoe", (char *)&ndo_enable, 4, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return -1;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, 0);
	if (retcode)
		DHD_ERROR(("%s: failed to enabe ndo to %d, retcode = %d\n",
			__FUNCTION__, ndo_enable, retcode));
	else
		DHD_TRACE(("%s: successfully enabed ndo offload to %d\n",
			__FUNCTION__, ndo_enable));

	return retcode;
}

int
dhd_ndo_add_ip(dhd_pub_t *dhd, char* ipv6addr, int idx)
{
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int retcode;

	if (dhd == NULL)
		return -1;

	iov_len = bcm_mkiovar("nd_hostip", (char *)ipv6addr,
		IPV6_ADDR_LEN, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return -1;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);

	if (retcode)
		DHD_ERROR(("%s: ndo ip addr add failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: ndo ipaddr entry added \n",
		__FUNCTION__));

	return retcode;
}
int
dhd_ndo_remove_ip(dhd_pub_t *dhd, int idx)
{
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int retcode;

	if (dhd == NULL)
		return -1;

	iov_len = bcm_mkiovar("nd_hostip_clear", NULL,
		0, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return -1;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);

	if (retcode)
		DHD_ERROR(("%s: ndo ip addr remove failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: ndo ipaddr entry removed \n",
		__FUNCTION__));

	return retcode;
}

void
dhd_sendup_event_common(dhd_pub_t *dhdp, wl_event_msg_t *event, void *data)
{
	switch (ntoh32(event->event_type)) {
#ifdef WLBTAMP
	case WLC_E_BTA_HCI_EVENT:
		break;
#endif 
	default:
		break;
	}

	
	dhd_sendup_event(dhdp, event, data);
}


bool dhd_is_associated(dhd_pub_t *dhd, void *bss_buf, int *retval)
{
	char bssid[6], zbuf[6];
	int ret = -1;

	bzero(bssid, 6);
	bzero(zbuf, 6);

	ret  = dhd_wl_ioctl_cmd(dhd, WLC_GET_BSSID, (char *)&bssid, ETHER_ADDR_LEN, FALSE, 0);
	DHD_TRACE((" %s WLC_GET_BSSID ioctl res = %d\n", __FUNCTION__, ret));

	if (ret == BCME_NOTASSOCIATED) {
		DHD_TRACE(("%s: not associated! res:%d\n", __FUNCTION__, ret));
	}

	if (retval)
		*retval = ret;

	if (ret < 0)
		return FALSE;

	if ((memcmp(bssid, zbuf, ETHER_ADDR_LEN) != 0)) {
		

		if (bss_buf) {
			
			memcpy(bss_buf, bssid, ETHER_ADDR_LEN);
		}
		return TRUE;
	} else {
		DHD_TRACE(("%s: WLC_GET_BSSID ioctl returned zero bssid\n", __FUNCTION__));
		return FALSE;
	}
}

int
dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd)
{
	int bcn_li_dtim = 1; 
	int ret = -1;
	int dtim_period = 0;
	int ap_beacon = 0;
	int allowed_skip_dtim_cnt = 0;
	
	if (dhd_is_associated(dhd, NULL, NULL) == FALSE) {
		DHD_TRACE(("%s NOT assoc ret %d\n", __FUNCTION__, ret));
		goto exit;
	}

	
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_BCNPRD,
		&ap_beacon, sizeof(ap_beacon), FALSE, 0)) < 0) {
		DHD_ERROR(("%s get beacon failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_DTIMPRD,
		&dtim_period, sizeof(dtim_period), FALSE, 0)) < 0) {
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	
	if (dtim_period == 0) {
		goto exit;
	}

	
	bcn_li_dtim = dhd->suspend_bcn_li_dtim;

	
	if (dtim_period > CUSTOM_LISTEN_INTERVAL) {
		
		bcn_li_dtim = NO_DTIM_SKIP;
		DHD_ERROR(("%s DTIM=%d > Listen=%d : too big ...\n",
			__FUNCTION__, dtim_period, CUSTOM_LISTEN_INTERVAL));
		goto exit;
	}

	if ((dtim_period * ap_beacon * bcn_li_dtim) > MAX_DTIM_ALLOWED_INTERVAL) {
		 allowed_skip_dtim_cnt = MAX_DTIM_ALLOWED_INTERVAL / (dtim_period * ap_beacon);
		 bcn_li_dtim = (allowed_skip_dtim_cnt != 0) ? allowed_skip_dtim_cnt : NO_DTIM_SKIP;
	}

	if ((bcn_li_dtim * dtim_period) > CUSTOM_LISTEN_INTERVAL) {
		
		bcn_li_dtim = (int)(CUSTOM_LISTEN_INTERVAL / dtim_period);
		DHD_TRACE(("%s agjust dtim_skip as %d\n", __FUNCTION__, bcn_li_dtim));
	}

	DHD_ERROR(("%s beacon=%d bcn_li_dtim=%d DTIM=%d Listen=%d\n",
		__FUNCTION__, ap_beacon, bcn_li_dtim, dtim_period, CUSTOM_LISTEN_INTERVAL));

exit:
	return bcn_li_dtim;
}

bool dhd_support_sta_mode(dhd_pub_t *dhd)
{

#ifdef  WL_CFG80211
	if (!(dhd->op_mode & DHD_FLAG_STA_MODE))
		return FALSE;
	else
#endif 
		return TRUE;
}

#if defined(KEEP_ALIVE)
int dhd_keep_alive_onoff(dhd_pub_t *dhd)
{
	char				buf[32] = {0};
	const char			*str;
	wl_mkeep_alive_pkt_t	mkeep_alive_pkt = {0};
	wl_mkeep_alive_pkt_t	*mkeep_alive_pktp;
	int					buf_len;
	int					str_len;
	int res					= -1;

	if (!dhd_support_sta_mode(dhd))
		return res;

	DHD_TRACE(("%s execution\n", __FUNCTION__));

#ifdef CUSTOMER_HW_ONE
	memset(&mkeep_alive_pkt, 0, sizeof(mkeep_alive_pkt));
#endif
	str = "mkeep_alive";
	str_len = strlen(str);
	strncpy(buf, str, sizeof(buf) - 1);
	buf[ sizeof(buf) - 1 ] = '\0';
	mkeep_alive_pktp = (wl_mkeep_alive_pkt_t *) (buf + str_len + 1);
	mkeep_alive_pkt.period_msec = CUSTOM_KEEP_ALIVE_SETTING;
	buf_len = str_len + 1;
	mkeep_alive_pkt.version = htod16(WL_MKEEP_ALIVE_VERSION);
	mkeep_alive_pkt.length = htod16(WL_MKEEP_ALIVE_FIXED_LEN);
	
	mkeep_alive_pkt.keep_alive_id = 0;
	mkeep_alive_pkt.len_bytes = 0;
	buf_len += WL_MKEEP_ALIVE_FIXED_LEN;
	bzero(mkeep_alive_pkt.data, sizeof(mkeep_alive_pkt.data));
	memcpy((char *)mkeep_alive_pktp, &mkeep_alive_pkt, WL_MKEEP_ALIVE_FIXED_LEN);

	res = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);

	return res;
}
#endif 

int
wl_iw_parse_data_tlv(char** list_str, void *dst, int dst_size, const char token,
                     int input_size, int *bytes_left)
{
	char* str;
	uint16 short_temp;
	uint32 int_temp;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}
	str = *list_str;

	
	memset(dst, 0, dst_size);
	while (*bytes_left > 0) {

		if (str[0] != token) {
			DHD_TRACE(("%s NOT Type=%d get=%d left_parse=%d \n",
				__FUNCTION__, token, str[0], *bytes_left));
			return -1;
		}

		*bytes_left -= 1;
		str += 1;

		if (input_size == 1) {
			memcpy(dst, str, input_size);
		}
		else if (input_size == 2) {
			memcpy(dst, (char *)htod16(memcpy(&short_temp, str, input_size)),
				input_size);
		}
		else if (input_size == 4) {
			memcpy(dst, (char *)htod32(memcpy(&int_temp, str, input_size)),
				input_size);
		}

		*bytes_left -= input_size;
		str += input_size;
		*list_str = str;
		return 1;
	}
	return 1;
}

int
wl_iw_parse_channel_list_tlv(char** list_str, uint16* channel_list,
                             int channel_num, int *bytes_left)
{
	char* str;
	int idx = 0;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}
	str = *list_str;

	while (*bytes_left > 0) {

		if (str[0] != CSCAN_TLV_TYPE_CHANNEL_IE) {
			*list_str = str;
			DHD_TRACE(("End channel=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}
		
		*bytes_left -= 1;
		str += 1;

		if (str[0] == 0) {
			
			channel_list[idx] = 0x0;
		}
		else {
			channel_list[idx] = (uint16)str[0];
			DHD_TRACE(("%s channel=%d \n", __FUNCTION__,  channel_list[idx]));
		}
		*bytes_left -= 1;
		str += 1;

		if (idx++ > 255) {
			DHD_ERROR(("%s Too many channels \n", __FUNCTION__));
			return -1;
		}
	}

	*list_str = str;
	return idx;
}

int
wl_iw_parse_ssid_list_tlv(char** list_str, wlc_ssid_t* ssid, int max, int *bytes_left)
{
	char* str;
	int idx = 0;

	if ((list_str == NULL) || (*list_str == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}
	str = *list_str;
	while (*bytes_left > 0) {

		if (str[0] != CSCAN_TLV_TYPE_SSID_IE) {
			*list_str = str;
			DHD_TRACE(("nssid=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}

		
		*bytes_left -= 1;
		str += 1;

		if (str[0] == 0) {
			
			ssid[idx].SSID_len = 0;
			memset((char*)ssid[idx].SSID, 0x0, DOT11_MAX_SSID_LEN);
			*bytes_left -= 1;
			str += 1;

			DHD_TRACE(("BROADCAST SCAN  left=%d\n", *bytes_left));
		}
		else if (str[0] <= DOT11_MAX_SSID_LEN) {
			
			ssid[idx].SSID_len = str[0];
			*bytes_left -= 1;
			str += 1;

			
			if (ssid[idx].SSID_len > *bytes_left) {
				DHD_ERROR(("%s out of memory range len=%d but left=%d\n",
				__FUNCTION__, ssid[idx].SSID_len, *bytes_left));
				return -1;
			}

			memcpy((char*)ssid[idx].SSID, str, ssid[idx].SSID_len);

			*bytes_left -= ssid[idx].SSID_len;
			str += ssid[idx].SSID_len;

			DHD_TRACE(("%s :size=%d left=%d\n",
				(char*)ssid[idx].SSID, ssid[idx].SSID_len, *bytes_left));
		}
		else {
			DHD_ERROR(("### SSID size more that %d\n", str[0]));
			return -1;
		}

		if (idx++ >  max) {
			DHD_ERROR(("%s number of SSIDs more that %d\n", __FUNCTION__, idx));
			return -1;
		}
	}

	*list_str = str;
	return idx;
}

int
wl_iw_parse_ssid_list(char** list_str, wlc_ssid_t* ssid, int idx, int max)
{
	char* str, *ptr;

	if ((list_str == NULL) || (*list_str == NULL))
		return -1;

	for (str = *list_str; str != NULL; str = ptr) {

		
		if (!strncmp(str, GET_CHANNEL, strlen(GET_CHANNEL))) {
			*list_str	 = str + strlen(GET_CHANNEL);
			return idx;
		}

		if ((ptr = strchr(str, ',')) != NULL) {
			*ptr++ = '\0';
		}

		if (strlen(str) > DOT11_MAX_SSID_LEN) {
			DHD_ERROR(("ssid <%s> exceeds %d\n", str, DOT11_MAX_SSID_LEN));
			return -1;
		}

		if (strlen(str) == 0)
			ssid[idx].SSID_len = 0;

		if (idx < max) {
			bzero(ssid[idx].SSID, sizeof(ssid[idx].SSID));
			strncpy((char*)ssid[idx].SSID, str, sizeof(ssid[idx].SSID) - 1);
			ssid[idx].SSID_len = strlen(str);
		}
		idx++;
	}
	return idx;
}

int
wl_iw_parse_channel_list(char** list_str, uint16* channel_list, int channel_num)
{
	int num;
	int val;
	char* str;
	char* endptr = NULL;

	if ((list_str == NULL)||(*list_str == NULL))
		return -1;

	str = *list_str;
	num = 0;
	while (strncmp(str, GET_NPROBE, strlen(GET_NPROBE))) {
		val = (int)strtoul(str, &endptr, 0);
		if (endptr == str) {
			DHD_ERROR(("could not parse channel number starting at"
				" substring \"%s\" in list:\n%s\n",
				str, *list_str));
			return -1;
		}
		str = endptr + strspn(endptr, " ,");

		if (num == channel_num) {
			DHD_ERROR(("too many channels (more than %d) in channel list:\n%s\n",
				channel_num, *list_str));
			return -1;
		}

		channel_list[num++] = (uint16)val;
	}
	*list_str = str;
	return num;
}

#ifdef CUSTOMER_HW_ONE
int dhd_set_pktfilter(dhd_pub_t * dhd, int add, int id, int offset, char *mask, char *pattern)
{
#ifdef PKT_FILTER_SUPPORT
	char 				*str;
	wl_pkt_filter_t		pkt_filter;
	wl_pkt_filter_t		*pkt_filterp;
	int						buf_len;
	int						str_len;
	uint32					mask_size;
	uint32					pattern_size;
	char buf[256];
	int pkt_id = id;
	wl_pkt_filter_enable_t	enable_parm;

	DHD_ERROR(("Enter set packet filter\n"));

	
	enable_parm.id = htod32(pkt_id);
	enable_parm.enable = htod32(0);
	bcm_mkiovar("pkt_filter_enable", (char *)&enable_parm,
		sizeof(wl_pkt_filter_enable_t), buf, sizeof(buf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);

	
	bcm_mkiovar("pkt_filter_delete", (char *)&pkt_id, 4, buf, sizeof(buf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);

	if (!add) {
		return 0;
	}

	DHD_ERROR(("start to add pkt filter %d\n", pkt_id));
	memset(buf, 0, sizeof(buf));
	
	str = "pkt_filter_add";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	
	pkt_filter.id = htod32(pkt_id);

	
	pkt_filter.negate_match = htod32(0);

	
	pkt_filter.type = htod32(0);

	
	pkt_filter.u.pattern.offset = htod32(offset);

	
	mask_size =	htod32(wl_pattern_atoh(mask,
		(char *) pkt_filterp->u.pattern.mask_and_pattern));

	
	if (add == 1 && id == 101) {
		DHD_ERROR(("Update dtim after connected AP, screen_off:%d\n", is_screen_off));
		dhdhtc_update_dtim_listen_interval(is_screen_off);
	}

	
	pattern_size = htod32(wl_pattern_atoh(pattern,
		(char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]));

	if (mask_size != pattern_size) {
		DHD_ERROR(("Mask and pattern not the same size\n"));
		return -EINVAL;
	}

	pkt_filter.u.pattern.size_bytes = mask_size;
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);

	memcpy((char *)pkt_filterp, &pkt_filter,
		WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);

	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);

	enable_parm.id = htod32(pkt_id);
	enable_parm.enable = htod32(1);

	bcm_mkiovar("pkt_filter_enable", (char *)&enable_parm,
		sizeof(wl_pkt_filter_enable_t), buf, sizeof(buf));
	dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
#endif 

	return 0;
}

bool dhd_check_ap_mode_set(dhd_pub_t *dhd)
{
#ifdef WL_CFG80211
	if ((dhd->op_mode & DHD_FLAG_HOSTAP_MODE) == DHD_FLAG_HOSTAP_MODE)
		return TRUE;
	else
#endif 
		return FALSE;
}

#ifdef PNO_SUPPORT
typedef struct pfn_ssid {
	char		ssid[32];
	int32		ssid_len;
	uint32		weight;
} pfn_ssid_t;

extern int dhd_pno_enable(dhd_pub_t *dhd, int pfn_enabled);
extern int dhd_pno_clean(dhd_pub_t *dhd);
extern int dhd_pno_set_ssid(dhd_pub_t *dhd, wlc_ssid_t* ssids_local, int nssid,
                       ushort  scan_fr, int pno_repeat, int pno_freq_expo_max);

int
dhd_pno_set_ssid(dhd_pub_t *dhd, wlc_ssid_t* ssids_local, int nssid, ushort scan_fr,
	int pno_repeat, int pno_freq_expo_max)
{
	int err = -1;
	char iovbuf[128];
	int k, i;
	wl_pfn_param_t pfn_param;
	wl_pfn_t	pfn_element;
	uint len = 0;

	DHD_TRACE(("%s nssid=%d nchan=%d\n", __FUNCTION__, nssid, scan_fr));

	if ((!dhd) || (!ssids_local)) {
		DHD_ERROR(("%s error exit(%s %s)\n", __FUNCTION__,
		(!dhd)?"dhd is null":"", (!ssids_local)?"ssid is null":""));
		err = -1;
		return err;
	}
#ifndef WL_SCHED_SCAN
	if (!dhd_support_sta_mode(dhd))
		return err;
#endif 

	
	for (k = 0; k < nssid; k++) {
		if (!ssids_local[k].SSID_len) {
			DHD_ERROR(("%d: Broadcast SSID is ilegal for PNO setting\n", k));
			return err;
		}
	}
#ifdef PNO_DUMP
	{
		int j;
		for (j = 0; j < nssid; j++) {
			DHD_ERROR(("%d: scan  for  %s size =%d\n", j,
				ssids_local[j].SSID, ssids_local[j].SSID_len));
		}
	}
#endif 

	
	if  ((err = dhd_pno_clean(dhd)) < 0) {
		DHD_ERROR(("%s failed error=%d\n", __FUNCTION__, err));
		return err;
	}
	memset(iovbuf, 0, sizeof(iovbuf));
	memset(&pfn_param, 0, sizeof(pfn_param));
	memset(&pfn_element, 0, sizeof(pfn_element));

	
	pfn_param.version = htod32(PFN_VERSION);
	pfn_param.flags = htod16((PFN_LIST_ORDER << SORT_CRITERIA_BIT));

	
	if ((pno_repeat != 0) || (pno_freq_expo_max != 0)) {
		pfn_param.flags |= htod16(ENABLE << ENABLE_ADAPTSCAN_BIT);
		pfn_param.repeat = (uchar) (pno_repeat);
		pfn_param.exp = (uchar) (pno_freq_expo_max);
	}
	
	if (scan_fr  != 0)
		pfn_param.scan_freq = htod32(scan_fr);

	if (pfn_param.scan_freq > PNO_SCAN_MAX_FW_SEC) {
		DHD_ERROR(("%s pno freq above %d sec\n", __FUNCTION__, PNO_SCAN_MAX_FW_SEC));
		return err;
	}
	if (pfn_param.scan_freq < PNO_SCAN_MIN_FW_SEC) {
		DHD_ERROR(("%s pno freq less %d sec\n", __FUNCTION__, PNO_SCAN_MIN_FW_SEC));
		return err;
	}

	len = bcm_mkiovar("pfn_set", (char *)&pfn_param, sizeof(pfn_param), iovbuf, sizeof(iovbuf));
	if ((err = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, len, TRUE, 0)) < 0) {
				DHD_ERROR(("%s pfn_set failed for error=%d\n",
					__FUNCTION__, err));
				return err;
	}

	
	for (i = 0; i < nssid; i++) {

		pfn_element.infra = htod32(DOT11_BSSTYPE_INFRASTRUCTURE);
		pfn_element.auth = (DOT11_OPEN_SYSTEM);
		pfn_element.wpa_auth = htod32(WPA_AUTH_PFN_ANY);
		pfn_element.wsec = htod32(0);
		pfn_element.infra = htod32(1);
		pfn_element.flags = htod32(ENABLE << WL_PFN_HIDDEN_BIT);
		memcpy((char *)pfn_element.ssid.SSID, ssids_local[i].SSID, ssids_local[i].SSID_len);
		pfn_element.ssid.SSID_len = ssids_local[i].SSID_len;

		if ((len =
		bcm_mkiovar("pfn_add", (char *)&pfn_element,
			sizeof(pfn_element), iovbuf, sizeof(iovbuf))) > 0) {
			if ((err =
			dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, len, TRUE, 0)) < 0) {
				DHD_ERROR(("%s failed for i=%d error=%d\n",
					__FUNCTION__, i, err));
				return err;
			}
			else
				DHD_TRACE(("%s set OK with PNO time=%d repeat=%d max_adjust=%d\n",
					__FUNCTION__, pfn_param.scan_freq,
					pfn_param.repeat, pfn_param.exp));
		}
		else DHD_ERROR(("%s failed err=%d\n", __FUNCTION__, err));
	}

	
	
	return err;
}
#endif 
#endif 
