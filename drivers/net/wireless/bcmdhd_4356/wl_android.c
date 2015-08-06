/*
 * Linux cfg80211 driver - Android related functions
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
 * $Id: wl_android.c 556862 2015-05-15 04:01:30Z $
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <net/netlink.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <wl_android.h>
#include <wldev_common.h>
#include <wlioctl.h>
#include <bcmutils.h>
#include <linux_osl.h>
#include <dhd_dbg.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <proto/bcmip.h>
#ifdef PNO_SUPPORT
#include <dhd_pno.h>
#endif
#ifdef BCMSDIO
#include <bcmsdbus.h>
#endif
#ifdef WL_CFG80211
#include <wl_cfg80211.h>
#endif
#ifdef WL_NAN
#include <wl_cfgnan.h>
#endif 


#define CMD_START		"START"
#define CMD_STOP		"STOP"
#define	CMD_SCAN_ACTIVE		"SCAN-ACTIVE"
#define	CMD_SCAN_PASSIVE	"SCAN-PASSIVE"
#define CMD_RSSI		"RSSI"
#define CMD_LINKSPEED		"LINKSPEED"
#ifdef PKT_FILTER_SUPPORT
#define CMD_RXFILTER_START	"RXFILTER-START"
#define CMD_RXFILTER_STOP	"RXFILTER-STOP"
#define CMD_RXFILTER_ADD	"RXFILTER-ADD"
#define CMD_RXFILTER_REMOVE	"RXFILTER-REMOVE"
#if defined(CUSTOM_PLATFORM_NV_TEGRA)
#define CMD_PKT_FILTER_MODE		"PKT_FILTER_MODE"
#define CMD_PKT_FILTER_PORTS	"PKT_FILTER_PORTS"
#endif 
#endif 
#define CMD_BTCOEXSCAN_START	"BTCOEXSCAN-START"
#define CMD_BTCOEXSCAN_STOP	"BTCOEXSCAN-STOP"
#define CMD_BTCOEXMODE		"BTCOEXMODE"
#define CMD_SETSUSPENDOPT	"SETSUSPENDOPT"
#define CMD_SETSUSPENDMODE      "SETSUSPENDMODE"
#define CMD_P2P_DEV_ADDR	"P2P_DEV_ADDR"
#define CMD_SETFWPATH		"SETFWPATH"
#define CMD_SETBAND		"SETBAND"
#define CMD_GETBAND		"GETBAND"
#define CMD_COUNTRY		"COUNTRY"
#define CMD_P2P_SET_NOA		"P2P_SET_NOA"
#if !defined WL_ENABLE_P2P_IF
#define CMD_P2P_GET_NOA			"P2P_GET_NOA"
#endif 
#define CMD_P2P_SD_OFFLOAD		"P2P_SD_"
#define CMD_P2P_SET_PS		"P2P_SET_PS"
#define CMD_SET_AP_WPS_P2P_IE 		"SET_AP_WPS_P2P_IE"
#define CMD_SETROAMMODE 	"SETROAMMODE"
#ifdef CUSTOMER_HW_ONE
#define CMD_LTECOEX		"SETLTECOEX"
#endif
#define CMD_SETIBSSBEACONOUIDATA	"SETIBSSBEACONOUIDATA"
#define CMD_MIRACAST		"MIRACAST"
#define CMD_NAN		"NAN_"

#if defined(WL_SUPPORT_AUTO_CHANNEL)
#define CMD_GET_BEST_CHANNELS	"GET_BEST_CHANNELS"
#endif 

#if defined(CUSTOM_PLATFORM_NV_TEGRA)
#define CMD_SETMIRACAST 	"SETMIRACAST"
#define CMD_ASSOCRESPIE 	"ASSOCRESPIE"
#define CMD_RXRATESTATS        "RXRATESTATS"
#endif 

#define CMD_KEEP_ALIVE		"KEEPALIVE"

#ifdef BCMCCX
#define CMD_GETCCKM_RN		"get cckm_rn"
#define CMD_SETCCKM_KRK		"set cckm_krk"
#define CMD_GET_ASSOC_RES_IES	"get assoc_res_ies"
#endif

#ifdef PNO_SUPPORT
#define CMD_PNOSSIDCLR_SET	"PNOSSIDCLR"
#define CMD_PNOSETUP_SET	"PNOSETUP "
#define CMD_PNOENABLE_SET	"PNOFORCE"
#define CMD_PNODEBUG_SET	"PNODEBUG"
#define CMD_WLS_BATCHING	"WLS_BATCHING"
#endif 

#define CMD_OKC_SET_PMK		"SET_PMK"
#define CMD_OKC_ENABLE		"OKC_ENABLE"

#define	CMD_HAPD_MAC_FILTER	"HAPD_MAC_FILTER"

#ifdef WLFBT
#define CMD_GET_FTKEY      "GET_FTKEY"
#endif

#ifdef WLAIBSS
#define CMD_SETIBSSTXFAILEVENT		"SETIBSSTXFAILEVENT"
#define CMD_GET_IBSS_PEER_INFO		"GETIBSSPEERINFO"
#define CMD_GET_IBSS_PEER_INFO_ALL	"GETIBSSPEERINFOALL"
#define CMD_SETIBSSROUTETABLE		"SETIBSSROUTETABLE"
#define CMD_SETIBSSAMPDU			"SETIBSSAMPDU"
#define CMD_SETIBSSANTENNAMODE		"SETIBSSANTENNAMODE"
#endif 

#define CMD_ROAM_OFFLOAD			"SETROAMOFFLOAD"
#define CMD_ROAM_OFFLOAD_APLIST		"SETROAMOFFLAPLIST"
#define CMD_GET_LINK_STATUS			"GETLINKSTATUS"

#ifdef P2PRESP_WFDIE_SRC
#define CMD_P2P_SET_WFDIE_RESP      "P2P_SET_WFDIE_RESP"
#define CMD_P2P_GET_WFDIE_RESP      "P2P_GET_WFDIE_RESP"
#endif 

#define WL_ANDROID_LINK_VHT					0x01
#define WL_ANDROID_LINK_MIMO					0x02
#define WL_ANDROID_LINK_AP_VHT_SUPPORT		0x04
#define WL_ANDROID_LINK_AP_MIMO_SUPPORT	0x08

#define MIRACAST_MODE_OFF	0
#define MIRACAST_MODE_SOURCE	1
#define MIRACAST_MODE_SINK	2

#ifndef MIRACAST_AMPDU_SIZE
#define MIRACAST_AMPDU_SIZE	8
#endif

#ifndef MIRACAST_MCHAN_ALGO
#define MIRACAST_MCHAN_ALGO     1
#endif

#ifndef MIRACAST_MCHAN_BW
#define MIRACAST_MCHAN_BW       25
#endif

#ifdef CONNECTION_STATISTICS
#define CMD_GET_CONNECTION_STATS	"GET_CONNECTION_STATS"

struct connection_stats {
	u32 txframe;
	u32 txbyte;
	u32 txerror;
	u32 rxframe;
	u32 rxbyte;
	u32 txfail;
	u32 txretry;
	u32 txretrie;
	u32 txrts;
	u32 txnocts;
	u32 txexptime;
	u32 txrate;
	u8	chan_idle;
};
#endif 

static LIST_HEAD(miracast_resume_list);
static u8 miracast_cur_mode;

struct io_cfg {
	s8 *iovar;
	s32 param;
	u32 ioctl;
	void *arg;
	u32 len;
	struct list_head list;
};

typedef struct _android_wifi_priv_cmd {
	char *buf;
	int used_len;
	int total_len;
} android_wifi_priv_cmd;

#ifdef CONFIG_COMPAT
typedef struct _compat_android_wifi_priv_cmd {
	compat_caddr_t buf;
	int used_len;
	int total_len;
} compat_android_wifi_priv_cmd;
#endif 

#if defined(BCMFW_ROAM_ENABLE)
#define CMD_SET_ROAMPREF	"SET_ROAMPREF"

#define MAX_NUM_SUITES		10
#define WIDTH_AKM_SUITE		8
#define JOIN_PREF_RSSI_LEN		0x02
#define JOIN_PREF_RSSI_SIZE		4	
#define JOIN_PREF_WPA_HDR_SIZE		4 
#define JOIN_PREF_WPA_TUPLE_SIZE	12	
#define JOIN_PREF_MAX_WPA_TUPLES	16
#define MAX_BUF_SIZE		(JOIN_PREF_RSSI_SIZE + JOIN_PREF_WPA_HDR_SIZE +	\
				           (JOIN_PREF_WPA_TUPLE_SIZE * JOIN_PREF_MAX_WPA_TUPLES))
#endif 

#ifdef WL_GENL
static s32 wl_genl_handle_msg(struct sk_buff *skb, struct genl_info *info);
static int wl_genl_init(void);
static int wl_genl_deinit(void);

extern struct net init_net;
static struct nla_policy wl_genl_policy[BCM_GENL_ATTR_MAX + 1] = {
	[BCM_GENL_ATTR_STRING] = { .type = NLA_NUL_STRING },
	[BCM_GENL_ATTR_MSG] = { .type = NLA_BINARY },
};

#define WL_GENL_VER 1
static struct genl_family wl_genl_family = {
	.id = GENL_ID_GENERATE,    
	.hdrsize = 0,
	.name = "bcm-genl",        
	.version = WL_GENL_VER,     
	.maxattr = BCM_GENL_ATTR_MAX,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
struct genl_ops wl_genl_ops[] = {
	{
	.cmd = BCM_GENL_CMD_MSG,
	.flags = 0,
	.policy = wl_genl_policy,
	.doit = wl_genl_handle_msg,
	.dumpit = NULL,
	},
};
#else
struct genl_ops wl_genl_ops = {
	.cmd = BCM_GENL_CMD_MSG,
	.flags = 0,
	.policy = wl_genl_policy,
	.doit = wl_genl_handle_msg,
	.dumpit = NULL,

};
#endif 

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
static struct genl_multicast_group wl_genl_mcast[] = {
	 { .name = "bcm-genl-mcast", },
};
#else
static struct genl_multicast_group wl_genl_mcast = {
	.id = GENL_ID_GENERATE,    
	.name = "bcm-genl-mcast",
};
#endif 
#endif 

int dhd_net_bus_devreset(struct net_device *dev, uint8 flag);
int dhd_dev_init_ioctl(struct net_device *dev);
#ifdef WL_CFG80211
int wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr);
int wl_cfg80211_set_btcoex_dhcp(struct net_device *dev, dhd_pub_t *dhd, char *command);
#else
int wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr)
{ return 0; }
int wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len)
{ return 0; }
#endif 


#ifdef ENABLE_4335BT_WAR
extern int bcm_bt_lock(int cookie);
extern void bcm_bt_unlock(int cookie);
static int lock_cookie_wifi = 'W' | 'i'<<8 | 'F'<<16 | 'i'<<24;	
#endif 

extern bool ap_fw_loaded;
#if defined(CUSTOMER_HW2)
extern char iface_name[IFNAMSIZ];
#endif 

#ifdef CUSTOMER_HW_ONE
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

#ifdef CONFIG_PERFLOCK
struct perf_lock *wlan_perf_lock;
#endif
int block_ap_event = 0;
static uint32 last_txframes = 0xffffffff;
static uint32 last_txretrans = 0xffffffff;
static uint32 last_txerror = 0xffffffff;
#if defined(USE_STATIC_MEMDUMP)
extern int dhd_pub_mem_dump(dhd_pub_t *dhd);
#endif

extern int dhdcdc_wifiLock;
static struct mac_list_set android_mac_list_buf;
static struct mflist android_ap_white_list;
static struct mflist android_ap_black_list;
static int android_ap_macmode = MACLIST_MODE_DISABLED;

static struct mutex wl_wificall_mutex;
void wl_cfg80211_set_btcoex_done(struct net_device *dev);
extern int net_os_send_hang_message(struct net_device *dev);
int wl_cfg80211_get_mac_addr(struct net_device *net, struct ether_addr *mac_addr);
void wl_cfg80211_set_btcoex_done(struct net_device *dev);
extern bool check_hang_already(struct net_device *dev);
static int wl_android_get_mac_addr(struct net_device *ndev, char *command, int total_len);
static int wl_android_get_tx_fail(struct net_device *dev, char *command, int total_len);
static int wl_android_get_dtim_skip(struct net_device *dev, char *command, int total_len);
static int wl_android_set_dtim_skip(struct net_device *dev, char *command, int total_len);
static int wl_android_set_txpower(struct net_device *dev, char *command, int total_len);
static int wl_android_set_power_mode(struct net_device *dev, char *command, int total_len);
static int wl_android_get_power_mode(struct net_device *dev, char *command, int total_len);
static int wl_android_set_ap_txpower(struct net_device *dev, char *command, int total_len);
static int wl_android_get_assoc_sta_list(struct net_device *dev, char *buf, int len);
static int wl_android_set_ap_mac_list(struct net_device *dev, void *buf);
static int wl_android_set_scan_minrssi(struct net_device *dev, char *command, int total_len);
static int wl_android_low_rssi_set(struct net_device *dev, char *command, int total_len);
static int wl_android_get_wifilock(struct net_device *ndev, char *command, int total_len);
static int wl_android_set_wificall(struct net_device *ndev, char *command, int total_len);
static int wl_android_set_project(struct net_device *ndev, char *command, int total_len);
static int wl_android_gateway_add(struct net_device *ndev, char *command, int total_len);
static int wl_android_auto_channel(struct net_device *dev, char *command, int total_len);
static int wl_android_allow_p2p_event(struct net_device *dev, char *command, int total_len);
extern int wldev_get_conap_ctrl_channel(struct net_device *dev, uint8 *ctrl_channel);
static void wlan_init_perf(void);
static void wlan_deinit_perf(void);
int wl_cfg80211_send_priv_event(struct net_device *dev, char *flag);
extern s32 wl_cfg80211_set_scan_abort(struct net_device *ndev);

char project_type[33];
static int wl_android_wifi_call = 0;
static int last_auto_channel = 6;
char wl_abdroid_gatewaybuf[8+1]; 
static int active_level = -80;
static int active_period = 20000; 
static int wl_android_active_expired = 0;
struct timer_list *wl_android_active_timer = NULL;
static int assoc_count_buff = 0;
extern int sta_event_sent;

#define TRAFFIC_SUPER_HIGH_WATER_MARK	2600
#define TRAFFIC_HIGH_WATER_MARK			2300
#define TRAFFIC_LOW_WATER_MARK			256
typedef enum traffic_ind {
	TRAFFIC_STATS_NORMAL = 0,
	TRAFFIC_STATS_HIGH,
	TRAFFIC_STATS_SUPER_HIGH
} traffic_ind_t;

static int screen_off = 0;
int sta_connected = 0;
static int traffic_stats_flag = TRAFFIC_STATS_NORMAL;
static unsigned long current_traffic_count = 0;
static unsigned long last_traffic_count = 0;
static unsigned long last_traffic_count_jiffies = 0;

#define TX_FAIL_CHECK_COUNT		100

#ifdef HTC_TX_TRACKING
static int wl_android_set_tx_tracking(struct net_device *dev, char *command, int total_len);
static uint old_tx_stat_chk;
static uint old_tx_stat_chk_prd;
static uint old_tx_stat_chk_ratio;
static uint old_tx_stat_chk_num;
#endif

#define CMD_MAC_ADDR		"MACADDR"
#define CMD_P2P_SET_MPC 	"P2P_MPC_SET"
#define CMD_DEAUTH_STA 		"DEAUTH_STA"
#define CMD_DTIM_SKIP_SET	"DTIMSKIPSET"
#define CMD_DTIM_SKIP_GET	"DTIMSKIPGET"
#define CMD_TXPOWER_SET		"TXPOWER"
#define CMD_POWER_MODE_SET	"POWERMODE"
#define CMD_POWER_MODE_GET	"GETPOWER"
#define CMD_AP_TXPOWER_SET	"AP_TXPOWER_SET"
#define CMD_AP_ASSOC_LIST_GET	"AP_ASSOC_LIST_GET"
#define CMD_AP_MAC_LIST_SET	"AP_MAC_LIST_SET"
#define CMD_SCAN_MINRSSI_SET	"SCAN_MINRSSI"
#define CMD_LOW_RSSI_SET	"LOW_RSSI_IND_SET"
#define CMD_GET_TX_FAIL        "GET_TX_FAIL"
#if defined(HTC_TX_TRACKING)
#define CMD_TX_TRACKING		"SET_TX_TRACKING"
#endif
#define CMD_GETWIFILOCK		"GETWIFILOCK"
#define CMD_SETWIFICALL		"WIFICALL"
#define CMD_SETPROJECT		"SET_PROJECT"
#define CMD_SETLOWPOWERMODE		"SET_LOWPOWERMODE"
#define CMD_GATEWAYADD		"GATEWAY-ADD"
#define CMD_PFN_REMOVE		"PFN_REMOVE"
#define CMD_GET_AUTO_CHANNEL	"AUTOCHANNELGET"
#define CMD_SET_HOTSPOT_BW	"SET_HOTSPOT_BW"
#define CMD_TRIG_RAMDUMP	"TRIG_RAMDUMP"
#define CMD_ALLOW_P2P_EVENT	"ALLOW_P2P_EVENT"
#endif 


static int g_wifi_on = TRUE;

static int wl_android_get_link_speed(struct net_device *net, char *command, int total_len)
{
	int link_speed;
	int bytes_written;
	int error;

	error = wldev_get_link_speed(net, &link_speed);
	if (error)
		return -1;

	
	link_speed = link_speed / 1000;
	bytes_written = snprintf(command, total_len, "LinkSpeed %d", link_speed);
	DHD_INFO(("%s: command result is %s\n", __FUNCTION__, command));
#ifdef CUSTOMER_HW_ONE
	wl_android_traffic_monitor(net);
#endif
	return bytes_written;
}

static int wl_android_get_rssi(struct net_device *net, char *command, int total_len)
{
	wlc_ssid_t ssid = {0};
	int rssi;
	int bytes_written = 0;
	int error;

	error = wldev_get_rssi(net, &rssi);
	if (error)
		return -1;

	error = wldev_get_ssid(net, &ssid);
	if (error)
		return -1;
	if ((ssid.SSID_len == 0) || (ssid.SSID_len > DOT11_MAX_SSID_LEN)) {
		DHD_ERROR(("%s: wldev_get_ssid failed\n", __FUNCTION__));
	} else {
		memcpy(command, ssid.SSID, ssid.SSID_len);
		bytes_written = ssid.SSID_len;
	}
	bytes_written += snprintf(&command[bytes_written], total_len, " rssi %d", rssi);
	DHD_INFO(("%s: command result is %s (%d)\n", __FUNCTION__, command, bytes_written));
#ifdef CUSTOMER_HW_ONE
	wl_android_traffic_monitor(net);
#endif
	return bytes_written;
}

static int wl_android_set_suspendopt(struct net_device *dev, char *command, int total_len)
{
	int suspend_flag;
	int ret_now;
	int ret = 0;
#ifdef WL_CFG80211
	dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
	dhd_info_t *dhd = DHD_DEV_INFO(dev);
	dhd_pub_t *dhdp = &(dhd->pub);
#endif 

		suspend_flag = *(command + strlen(CMD_SETSUSPENDOPT) + 1) - '0';

		if (suspend_flag != 0)
			suspend_flag = 1;
#ifdef CUSTOMER_HW_ONE
		if (suspend_flag == 1)
			dhdcdc_wifiLock = 0;
		else if (suspend_flag == 0)
			dhdcdc_wifiLock = 1;
#endif
		ret_now = net_os_set_suspend_disable(dev, suspend_flag);

		if ((ret_now != suspend_flag) && dhdp->in_suspend) {
			if (!(ret = net_os_set_suspend(dev, !suspend_flag, 1))) {
				DHD_INFO(("%s: Suspend Flag %d -> %d\n",
					__FUNCTION__, ret_now, suspend_flag));
				
				dhdp->in_suspend = 1;
			}
			else
				DHD_ERROR(("%s: failed %d\n", __FUNCTION__, ret));
		}
	return ret;
}

static int wl_android_set_suspendmode(struct net_device *dev, char *command, int total_len)
{
	int ret = 0;

#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(DHD_USE_EARLYSUSPEND)
	int suspend_flag;

	suspend_flag = *(command + strlen(CMD_SETSUSPENDMODE) + 1) - '0';
	if (suspend_flag != 0)
		suspend_flag = 1;

	if (!(ret = net_os_set_suspend(dev, suspend_flag, 0)))
		DHD_INFO(("%s: Suspend Mode %d\n", __FUNCTION__, suspend_flag));
	else
		DHD_ERROR(("%s: failed %d\n", __FUNCTION__, ret));
#endif

	return ret;
}

static int wl_android_get_band(struct net_device *dev, char *command, int total_len)
{
	uint band;
	int bytes_written;
	int error;

	error = wldev_get_band(dev, &band);
	if (error)
		return -1;
	bytes_written = snprintf(command, total_len, "Band %d", band);
	return bytes_written;
}


#ifdef PNO_SUPPORT
#define PNO_PARAM_SIZE 50
#define VALUE_SIZE 50
#define LIMIT_STR_FMT  ("%50s %50s")
static int
wls_parse_batching_cmd(struct net_device *dev, char *command, int total_len)
{
	int err = BCME_OK;
	uint i, tokens;
	char *pos, *pos2, *token, *token2, *delim;
	char param[PNO_PARAM_SIZE], value[VALUE_SIZE];
	struct dhd_pno_batch_params batch_params;
	DHD_PNO(("%s: command=%s, len=%d\n", __FUNCTION__, command, total_len));
	if (total_len < strlen(CMD_WLS_BATCHING)) {
		DHD_ERROR(("%s argument=%d less min size\n", __FUNCTION__, total_len));
		err = BCME_ERROR;
		goto exit;
	}
	pos = command + strlen(CMD_WLS_BATCHING) + 1;
	memset(&batch_params, 0, sizeof(struct dhd_pno_batch_params));

	if (!strncmp(pos, PNO_BATCHING_SET, strlen(PNO_BATCHING_SET))) {
		pos += strlen(PNO_BATCHING_SET) + 1;
		while ((token = strsep(&pos, PNO_PARAMS_DELIMETER)) != NULL) {
			memset(param, 0, sizeof(param));
			memset(value, 0, sizeof(value));
			if (token == NULL || !*token)
				break;
			if (*token == '\0')
				continue;
			delim = strchr(token, PNO_PARAM_VALUE_DELLIMETER);
			if (delim != NULL)
				*delim = ' ';

			tokens = sscanf(token, LIMIT_STR_FMT, param, value);
			if (!strncmp(param, PNO_PARAM_SCANFREQ, strlen(PNO_PARAM_SCANFREQ))) {
				batch_params.scan_fr = simple_strtol(value, NULL, 0);
				DHD_PNO(("scan_freq : %d\n", batch_params.scan_fr));
			} else if (!strncmp(param, PNO_PARAM_BESTN, strlen(PNO_PARAM_BESTN))) {
				batch_params.bestn = simple_strtol(value, NULL, 0);
				DHD_PNO(("bestn : %d\n", batch_params.bestn));
			} else if (!strncmp(param, PNO_PARAM_MSCAN, strlen(PNO_PARAM_MSCAN))) {
				batch_params.mscan = simple_strtol(value, NULL, 0);
				DHD_PNO(("mscan : %d\n", batch_params.mscan));
			} else if (!strncmp(param, PNO_PARAM_CHANNEL, strlen(PNO_PARAM_CHANNEL))) {
				i = 0;
				pos2 = value;
				tokens = sscanf(value, "<%s>", value);
				if (tokens != 1) {
					err = BCME_ERROR;
					DHD_ERROR(("%s : invalid format for channel"
					" <> params\n", __FUNCTION__));
					goto exit;
				}
					while ((token2 = strsep(&pos2,
					PNO_PARAM_CHANNEL_DELIMETER)) != NULL) {
					if (token2 == NULL || !*token2)
						break;
					if (*token2 == '\0')
						continue;
					if (*token2 == 'A' || *token2 == 'B') {
						batch_params.band = (*token2 == 'A')?
							WLC_BAND_5G : WLC_BAND_2G;
						DHD_PNO(("band : %s\n",
							(*token2 == 'A')? "A" : "B"));
					} else {
						batch_params.chan_list[i++] =
						simple_strtol(token2, NULL, 0);
						batch_params.nchan++;
						DHD_PNO(("channel :%d\n",
						batch_params.chan_list[i-1]));
					}
				 }
			} else if (!strncmp(param, PNO_PARAM_RTT, strlen(PNO_PARAM_RTT))) {
				batch_params.rtt = simple_strtol(value, NULL, 0);
				DHD_PNO(("rtt : %d\n", batch_params.rtt));
			} else {
				DHD_ERROR(("%s : unknown param: %s\n", __FUNCTION__, param));
				err = BCME_ERROR;
				goto exit;
			}
		}
		err = dhd_dev_pno_set_for_batch(dev, &batch_params);
		if (err < 0) {
			DHD_ERROR(("failed to configure batch scan\n"));
		} else {
			memset(command, 0, total_len);
			err = sprintf(command, "%d", err);
		}
	} else if (!strncmp(pos, PNO_BATCHING_GET, strlen(PNO_BATCHING_GET))) {
		err = dhd_dev_pno_get_for_batch(dev, command, total_len);
		if (err < 0) {
			DHD_ERROR(("failed to getting batching results\n"));
		} else {
			err = strlen(command);
		}
	} else if (!strncmp(pos, PNO_BATCHING_STOP, strlen(PNO_BATCHING_STOP))) {
		err = dhd_dev_pno_stop_for_batch(dev);
		if (err < 0) {
			DHD_ERROR(("failed to stop batching scan\n"));
		} else {
			memset(command, 0, total_len);
			err = sprintf(command, "OK");
		}
	} else {
		DHD_ERROR(("%s : unknown command\n", __FUNCTION__));
		err = BCME_ERROR;
		goto exit;
	}
exit:
	return err;
}
#ifndef WL_SCHED_SCAN
static int wl_android_set_pno_setup(struct net_device *dev, char *command, int total_len)
{
	wlc_ssid_t ssids_local[MAX_PFN_LIST_COUNT];
	int res = -1;
	int nssid = 0;
	cmd_tlv_t *cmd_tlv_temp;
	char *str_ptr;
	int tlv_size_left;
	int pno_time = 0;
	int pno_repeat = 0;
	int pno_freq_expo_max = 0;

#ifdef PNO_SET_DEBUG
	int i;
	char pno_in_example[] = {
		'P', 'N', 'O', 'S', 'E', 'T', 'U', 'P', ' ',
		'S', '1', '2', '0',
		'S',
		0x05,
		'd', 'l', 'i', 'n', 'k',
		'S',
		0x04,
		'G', 'O', 'O', 'G',
		'T',
		'0', 'B',
		'R',
		'2',
		'M',
		'2',
		0x00
		};
#endif 
	DHD_PNO(("%s: command=%s, len=%d\n", __FUNCTION__, command, total_len));

	if (total_len < (strlen(CMD_PNOSETUP_SET) + sizeof(cmd_tlv_t))) {
		DHD_ERROR(("%s argument=%d less min size\n", __FUNCTION__, total_len));
		goto exit_proc;
	}
#ifdef PNO_SET_DEBUG
	memcpy(command, pno_in_example, sizeof(pno_in_example));
	total_len = sizeof(pno_in_example);
#endif
	str_ptr = command + strlen(CMD_PNOSETUP_SET);
	tlv_size_left = total_len - strlen(CMD_PNOSETUP_SET);

	cmd_tlv_temp = (cmd_tlv_t *)str_ptr;
	memset(ssids_local, 0, sizeof(ssids_local));

	if ((cmd_tlv_temp->prefix == PNO_TLV_PREFIX) &&
		(cmd_tlv_temp->version == PNO_TLV_VERSION) &&
		(cmd_tlv_temp->subtype == PNO_TLV_SUBTYPE_LEGACY_PNO)) {

		str_ptr += sizeof(cmd_tlv_t);
		tlv_size_left -= sizeof(cmd_tlv_t);

		if ((nssid = wl_iw_parse_ssid_list_tlv(&str_ptr, ssids_local,
			MAX_PFN_LIST_COUNT, &tlv_size_left)) <= 0) {
			DHD_ERROR(("SSID is not presented or corrupted ret=%d\n", nssid));
			goto exit_proc;
		} else {
			if ((str_ptr[0] != PNO_TLV_TYPE_TIME) || (tlv_size_left <= 1)) {
				DHD_ERROR(("%s scan duration corrupted field size %d\n",
					__FUNCTION__, tlv_size_left));
				goto exit_proc;
			}
			str_ptr++;
			pno_time = simple_strtoul(str_ptr, &str_ptr, 16);
			DHD_PNO(("%s: pno_time=%d\n", __FUNCTION__, pno_time));

			if (str_ptr[0] != 0) {
				if ((str_ptr[0] != PNO_TLV_FREQ_REPEAT)) {
					DHD_ERROR(("%s pno repeat : corrupted field\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_repeat = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_PNO(("%s :got pno_repeat=%d\n", __FUNCTION__, pno_repeat));
				if (str_ptr[0] != PNO_TLV_FREQ_EXPO_MAX) {
					DHD_ERROR(("%s FREQ_EXPO_MAX corrupted field size\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_freq_expo_max = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_PNO(("%s: pno_freq_expo_max=%d\n",
					__FUNCTION__, pno_freq_expo_max));
			}
		}
	} else {
		DHD_ERROR(("%s get wrong TLV command\n", __FUNCTION__));
		goto exit_proc;
	}

	res = dhd_dev_pno_set_for_ssid(dev, ssids_local, nssid, pno_time, pno_repeat,
		pno_freq_expo_max, NULL, 0);
exit_proc:
	return res;
}
#endif 
#endif 

static int wl_android_get_p2p_dev_addr(struct net_device *ndev, char *command, int total_len)
{
	int ret;
	int bytes_written = 0;

	ret = wl_cfg80211_get_p2p_dev_addr(ndev, (struct ether_addr*)command);
	if (ret)
		return 0;
	bytes_written = sizeof(struct ether_addr);
	return bytes_written;
}

#ifdef BCMCCX
static int wl_android_get_cckm_rn(struct net_device *dev, char *command)
{
	int error, rn;

	WL_TRACE(("%s:wl_android_get_cckm_rn\n", dev->name));

	error = wldev_iovar_getint(dev, "cckm_rn", &rn);
	if (unlikely(error)) {
		WL_ERR(("wl_android_get_cckm_rn error (%d)\n", error));
		return -1;
	}
	memcpy(command, &rn, sizeof(int));

	return sizeof(int);
}

static int wl_android_set_cckm_krk(struct net_device *dev, char *command)
{
	int error;
	unsigned char key[16];
	static char iovar_buf[WLC_IOCTL_MEDLEN];

	WL_TRACE(("%s: wl_iw_set_cckm_krk\n", dev->name));

	memset(iovar_buf, 0, sizeof(iovar_buf));
	memcpy(key, command+strlen("set cckm_krk")+1, 16);

	error = wldev_iovar_setbuf(dev, "cckm_krk", key, sizeof(key),
		iovar_buf, WLC_IOCTL_MEDLEN, NULL);
	if (unlikely(error))
	{
		WL_ERR((" cckm_krk set error (%d)\n", error));
		return -1;
	}
	return 0;
}

static int wl_android_get_assoc_res_ies(struct net_device *dev, char *command)
{
	int error;
	u8 buf[WL_ASSOC_INFO_MAX];
	wl_assoc_info_t assoc_info;
	u32 resp_ies_len = 0;
	int bytes_written = 0;

	WL_TRACE(("%s: wl_iw_get_assoc_res_ies\n", dev->name));

	error = wldev_iovar_getbuf(dev, "assoc_info", NULL, 0, buf, WL_ASSOC_INFO_MAX, NULL);
	if (unlikely(error)) {
		WL_ERR(("could not get assoc info (%d)\n", error));
		return -1;
	}

	memcpy(&assoc_info, buf, sizeof(wl_assoc_info_t));
	assoc_info.req_len = htod32(assoc_info.req_len);
	assoc_info.resp_len = htod32(assoc_info.resp_len);
	assoc_info.flags = htod32(assoc_info.flags);

	if (assoc_info.resp_len) {
		resp_ies_len = assoc_info.resp_len - sizeof(struct dot11_assoc_resp);
	}

	
	memcpy(command, &resp_ies_len, sizeof(u32));
	bytes_written = sizeof(u32);

	
	if (resp_ies_len) {
		error = wldev_iovar_getbuf(dev, "assoc_resp_ies", NULL, 0,
			buf, WL_ASSOC_INFO_MAX, NULL);
		if (unlikely(error)) {
			WL_ERR(("could not get assoc resp_ies (%d)\n", error));
			return -1;
		}

		memcpy(command+sizeof(u32), buf, resp_ies_len);
		bytes_written += resp_ies_len;
	}
	return bytes_written;
}

#endif 

#ifndef CUSTOMER_HW_ONE
int
wl_android_set_ap_mac_list(struct net_device *dev, int macmode, struct maclist *maclist)
{
	int i, j, match;
	int ret	= 0;
	char mac_buf[MAX_NUM_OF_ASSOCLIST *
		sizeof(struct ether_addr) + sizeof(uint)] = {0};
	struct maclist *assoc_maclist = (struct maclist *)mac_buf;

	
	if ((ret = wldev_ioctl(dev, WLC_SET_MACMODE, &macmode, sizeof(macmode), true)) != 0) {
		DHD_ERROR(("%s : WLC_SET_MACMODE error=%d\n", __FUNCTION__, ret));
		return ret;
	}
	if (macmode != MACLIST_MODE_DISABLED) {
		
		if ((ret = wldev_ioctl(dev, WLC_SET_MACLIST, maclist,
			sizeof(int) + sizeof(struct ether_addr) * maclist->count, true)) != 0) {
			DHD_ERROR(("%s : WLC_SET_MACLIST error=%d\n", __FUNCTION__, ret));
			return ret;
		}
		
		assoc_maclist->count = MAX_NUM_OF_ASSOCLIST;
		if ((ret = wldev_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist,
			sizeof(mac_buf), false)) != 0) {
			DHD_ERROR(("%s : WLC_GET_ASSOCLIST error=%d\n", __FUNCTION__, ret));
			return ret;
		}
		
		if (assoc_maclist->count) {
			
			for (i = 0; i < assoc_maclist->count; i++) {
				match = 0;
				
				for (j = 0; j < maclist->count; j++) {
					DHD_INFO(("%s : associated="MACDBG " list="MACDBG "\n",
					__FUNCTION__, MAC2STRDBG(assoc_maclist->ea[i].octet),
					MAC2STRDBG(maclist->ea[j].octet)));
					if (memcmp(assoc_maclist->ea[i].octet,
						maclist->ea[j].octet, ETHER_ADDR_LEN) == 0) {
						match = 1;
						break;
					}
				}
				
				
				if ((macmode == MACLIST_MODE_ALLOW && !match) ||
					(macmode == MACLIST_MODE_DENY && match)) {
					scb_val_t scbval;

					scbval.val = htod32(1);
					memcpy(&scbval.ea, &assoc_maclist->ea[i],
						ETHER_ADDR_LEN);
					if ((ret = wldev_ioctl(dev,
						WLC_SCB_DEAUTHENTICATE_FOR_REASON,
						&scbval, sizeof(scb_val_t), true)) != 0)
						DHD_ERROR(("%s WLC_SCB_DEAUTHENTICATE error=%d\n",
							__FUNCTION__, ret));
				}
			}
		}
	}
	return ret;
}

static int
wl_android_set_mac_address_filter(struct net_device *dev, const char* str)
{
	int i;
	int ret = 0;
	int macnum = 0;
	int macmode = MACLIST_MODE_DISABLED;
	struct maclist *list;
	char eabuf[ETHER_ADDR_STR_LEN];

	
	

	
	macmode = bcm_atoi(strsep((char**)&str, " "));

	if (macmode < MACLIST_MODE_DISABLED || macmode > MACLIST_MODE_ALLOW) {
		DHD_ERROR(("%s : invalid macmode %d\n", __FUNCTION__, macmode));
		return -1;
	}

	macnum = bcm_atoi(strsep((char**)&str, " "));
	if (macnum < 0 || macnum > MAX_NUM_MAC_FILT) {
		DHD_ERROR(("%s : invalid number of MAC address entries %d\n",
			__FUNCTION__, macnum));
		return -1;
	}
	
	list = (struct maclist*)kmalloc(sizeof(int) +
		sizeof(struct ether_addr) * macnum, GFP_KERNEL);
	if (!list) {
		DHD_ERROR(("%s : failed to allocate memory\n", __FUNCTION__));
		return -1;
	}
	
	list->count = htod32(macnum);
	bzero((char *)eabuf, ETHER_ADDR_STR_LEN);
	for (i = 0; i < list->count; i++) {
		strncpy(eabuf, strsep((char**)&str, " "), ETHER_ADDR_STR_LEN - 1);
		if (!(ret = bcm_ether_atoe(eabuf, &list->ea[i]))) {
			DHD_ERROR(("%s : mac parsing err index=%d, addr=%s\n",
				__FUNCTION__, i, eabuf));
			list->count--;
			break;
		}
		DHD_INFO(("%s : %d/%d MACADDR=%s", __FUNCTION__, i, list->count, eabuf));
	}
	
	if ((ret = wl_android_set_ap_mac_list(dev, macmode, list)) != 0)
		DHD_ERROR(("%s : Setting MAC list failed error=%d\n", __FUNCTION__, ret));

	kfree(list);

	return 0;
}
#endif 


int wl_android_wifi_on(struct net_device *dev)
{
	int ret = 0;
#ifdef CONFIG_MACH_UNIVERSAL5433
	int retry;
	
	if (!check_rev()) {
		retry = 1;
	} else {
		retry = POWERUP_MAX_RETRY;
	}
#else
	int retry = POWERUP_MAX_RETRY;
#endif 

	DHD_ERROR(("%s in\n", __FUNCTION__));
	if (!dev) {
		DHD_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -EINVAL;
	}
#ifdef CUSTOMER_HW_ONE
#if defined(HTC_TX_TRACKING)
	old_tx_stat_chk = 0xff;
	old_tx_stat_chk_prd = 0xff;
	old_tx_stat_chk_ratio = 0xff;
	old_tx_stat_chk_num = 0xff;
	last_txframes = 0xffffffff;
	last_txretrans = 0xffffffff;
	last_txerror = 0xffffffff;
#endif
	smp_mb();
#endif 

	dhd_net_if_lock(dev);
	if (!g_wifi_on) {
		do {
			dhd_net_wifi_platform_set_power(dev, TRUE, WIFI_TURNON_DELAY);
#ifdef BCMSDIO
			ret = dhd_net_bus_resume(dev, 0);
#endif 
#ifdef BCMPCIE
			ret = dhd_net_bus_devreset(dev, FALSE);
#endif 
			if (ret == 0)
				break;
			DHD_ERROR(("\nfailed to power up wifi chip, retry again (%d left) **\n\n",
				retry));
#ifdef BCMPCIE
			dhd_net_bus_devreset(dev, TRUE);
#endif 
			dhd_net_wifi_platform_set_power(dev, FALSE, WIFI_TURNOFF_DELAY);
		} while (retry-- > 0);
		if (ret != 0) {
			DHD_ERROR(("\nfailed to power up wifi chip, max retry reached **\n\n"));
			goto exit;
		}
#ifdef BCMSDIO
		ret = dhd_net_bus_devreset(dev, FALSE);
		dhd_net_bus_resume(dev, 1);
#endif 

#ifndef BCMPCIE
		if (!ret) {
			if (dhd_dev_init_ioctl(dev) < 0)
				ret = -EFAULT;
		}
#endif 
		g_wifi_on = TRUE;
	}

exit:
	dhd_net_if_unlock(dev);

	return ret;
}

int wl_android_wifi_off(struct net_device *dev)
{
	int ret = 0;

	DHD_ERROR(("%s in\n", __FUNCTION__));
	if (!dev) {
		DHD_TRACE(("%s: dev is null\n", __FUNCTION__));
		return -EINVAL;
	}

#ifdef CUSTOMER_HW_ONE
#if defined(HW_OOB)
	
#endif
	
	wl_cfg80211_set_btcoex_done(dev);

	bcm_mdelay(100);

	if (dhd_APUP) {
		DHD_ERROR(("apmode off - AP_DOWN\n"));
		dhd_APUP = false;
		if (check_hang_already(dev)) {
			DHD_ERROR(("Don't send AP_DOWN due to alreayd hang\n"));
		} else {
			wlan_unlock_multi_core(dev);
		}
	}
#endif 
	dhd_net_if_lock(dev);
	if (g_wifi_on) {
#if defined(BCMSDIO) || defined(BCMPCIE)
		ret = dhd_net_bus_devreset(dev, TRUE);
#ifdef BCMSDIO
		dhd_net_bus_suspend(dev);
#endif 
#endif 
		dhd_net_wifi_platform_set_power(dev, FALSE, WIFI_TURNOFF_DELAY);
		g_wifi_on = FALSE;
	}
#ifdef CUSTOMER_HW_ONE
	wlan_unlock_multi_core(dev);
	wlan_unlock_perf();
	bcm_mdelay(500);
#endif
	dhd_net_if_unlock(dev);

	return ret;
}

static int wl_android_set_fwpath(struct net_device *net, char *command, int total_len)
{
	if ((strlen(command) - strlen(CMD_SETFWPATH)) > MOD_PARAM_PATHLEN)
		return -1;
	return dhd_net_set_fw_path(net, command + strlen(CMD_SETFWPATH) + 1);
}

#ifdef CONNECTION_STATISTICS
static int
wl_chanim_stats(struct net_device *dev, u8 *chan_idle)
{
	int err;
	wl_chanim_stats_t *list;
	
	wl_chanim_stats_t param;
	u8 result[WLC_IOCTL_SMLEN];
	chanim_stats_t *stats;

	memset(&param, 0, sizeof(param));
	memset(result, 0, sizeof(result));

	param.buflen = htod32(sizeof(wl_chanim_stats_t));
	param.count = htod32(WL_CHANIM_COUNT_ONE);

	if ((err = wldev_iovar_getbuf(dev, "chanim_stats", (char*)&param, sizeof(wl_chanim_stats_t),
		(char*)result, sizeof(result), 0)) < 0) {
		WL_ERR(("Failed to get chanim results %d \n", err));
		return err;
	}

	list = (wl_chanim_stats_t*)result;

	list->buflen = dtoh32(list->buflen);
	list->version = dtoh32(list->version);
	list->count = dtoh32(list->count);

	if (list->buflen == 0) {
		list->version = 0;
		list->count = 0;
	} else if (list->version != WL_CHANIM_STATS_VERSION) {
		WL_ERR(("Sorry, firmware has wl_chanim_stats version %d "
			"but driver supports only version %d.\n",
				list->version, WL_CHANIM_STATS_VERSION));
		list->buflen = 0;
		list->count = 0;
	}

	stats = list->stats;
	stats->glitchcnt = dtoh32(stats->glitchcnt);
	stats->badplcp = dtoh32(stats->badplcp);
	stats->chanspec = dtoh16(stats->chanspec);
	stats->timestamp = dtoh32(stats->timestamp);
	stats->chan_idle = dtoh32(stats->chan_idle);

	WL_INFORM(("chanspec: 0x%4x glitch: %d badplcp: %d idle: %d timestamp: %d\n",
		stats->chanspec, stats->glitchcnt, stats->badplcp, stats->chan_idle,
		stats->timestamp));

	*chan_idle = stats->chan_idle;

	return (err);
}

static int
wl_android_get_connection_stats(struct net_device *dev, char *command, int total_len)
{
	wl_cnt_t* cnt = NULL;
	int link_speed = 0;
	struct connection_stats *output;
	unsigned int bufsize = 0;
	int bytes_written = 0;
	int ret = 0;

	WL_INFORM(("%s: enter Get Connection Stats\n", __FUNCTION__));

	if (total_len <= 0) {
		WL_ERR(("%s: invalid buffer size %d\n", __FUNCTION__, total_len));
		goto error;
	}

	bufsize = total_len;
	if (bufsize < sizeof(struct connection_stats)) {
		WL_ERR(("%s: not enough buffer size, provided=%u, requires=%u\n",
			__FUNCTION__, bufsize,
			sizeof(struct connection_stats)));
		goto error;
	}

	if ((cnt = kmalloc(sizeof(*cnt), GFP_KERNEL)) == NULL) {
		WL_ERR(("kmalloc failed\n"));
		return -1;
	}
	memset(cnt, 0, sizeof(*cnt));

	ret = wldev_iovar_getbuf(dev, "counters", NULL, 0, (char *)cnt, sizeof(wl_cnt_t), NULL);
	if (ret) {
		WL_ERR(("%s: wldev_iovar_getbuf() failed, ret=%d\n",
			__FUNCTION__, ret));
		goto error;
	}

	if (dtoh16(cnt->version) > WL_CNT_T_VERSION) {
		WL_ERR(("%s: incorrect version of wl_cnt_t, expected=%u got=%u\n",
			__FUNCTION__,  WL_CNT_T_VERSION, cnt->version));
		goto error;
	}

	
	ret = wldev_get_link_speed(dev, &link_speed);
	if (ret || link_speed < 0) {
		WL_ERR(("%s: wldev_get_link_speed() failed, ret=%d, speed=%d\n",
			__FUNCTION__, ret, link_speed));
		goto error;
	}

	output = (struct connection_stats *)command;
	output->txframe   = dtoh32(cnt->txframe);
	output->txbyte    = dtoh32(cnt->txbyte);
	output->txerror   = dtoh32(cnt->txerror);
	output->rxframe   = dtoh32(cnt->rxframe);
	output->rxbyte    = dtoh32(cnt->rxbyte);
	output->txfail    = dtoh32(cnt->txfail);
	output->txretry   = dtoh32(cnt->txretry);
	output->txretrie  = dtoh32(cnt->txretrie);
	output->txrts     = dtoh32(cnt->txrts);
	output->txnocts   = dtoh32(cnt->txnocts);
	output->txexptime = dtoh32(cnt->txexptime);
	output->txrate    = link_speed;

	
	if (wl_chanim_stats(dev, &(output->chan_idle)) < 0) {
		output->chan_idle = 0;
	};

	kfree(cnt);

	bytes_written = sizeof(struct connection_stats);
	return bytes_written;

error:
	if (cnt) {
		kfree(cnt);
	}
	return -1;
}
#endif 

static int
wl_android_set_pmk(struct net_device *dev, char *command, int total_len)
{
	uchar pmk[33];
	int error = 0;
	char smbuf[WLC_IOCTL_SMLEN];
#ifdef OKC_DEBUG
	int i = 0;
#endif

	bzero(pmk, sizeof(pmk));
	memcpy((char *)pmk, command + strlen("SET_PMK "), 32);
	error = wldev_iovar_setbuf(dev, "okc_info_pmk", pmk, 32, smbuf, sizeof(smbuf), NULL);
	if (error) {
		DHD_ERROR(("Failed to set PMK for OKC, error = %d\n", error));
	}
#ifdef OKC_DEBUG
	DHD_ERROR(("PMK is "));
	for (i = 0; i < 32; i++)
		DHD_ERROR(("%02X ", pmk[i]));

	DHD_ERROR(("\n"));
#endif
	return error;
}

#ifdef CUSTOMER_HW_ONE
extern int release_wlan_seci_gpio(void);

static int
wl_android_ltecoex_mode(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int coex_channel = 0;

	if (sscanf(command, "%*s %d", &coex_channel) != 1) {
		DHD_ERROR(("%s: Failed to get Parameter\n", __FUNCTION__));
		return -1;
	}

	
	if (coex_channel != 0)
		release_wlan_seci_gpio();
	

	error = wldev_iovar_setint(dev, "mws_coex_bitmap", coex_channel);
	if (error) {
		DHD_ERROR(("%s: Failed to set ltecoex %x, error = %d\n",
			__FUNCTION__, coex_channel, error));
		return -1;
	} else
		DHD_ERROR(("%s: succeeded to set lte_coex %x, error = %d\n",
			__FUNCTION__, coex_channel, error));
	return 0;
}
#endif 

static int
wl_android_okc_enable(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	char okc_enable = 0;

	okc_enable = command[strlen(CMD_OKC_ENABLE) + 1] - '0';
	error = wldev_iovar_setint(dev, "okc_enable", okc_enable);
	if (error) {
		DHD_ERROR(("Failed to %s OKC, error = %d\n",
			okc_enable ? "enable" : "disable", error));
	}

	wldev_iovar_setint(dev, "ccx_enable", 0);

	return error;
}



int wl_android_set_roam_mode(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int mode = 0;

	if (sscanf(command, "%*s %d", &mode) != 1) {
		DHD_ERROR(("%s: Failed to get Parameter\n", __FUNCTION__));
		return -1;
	}

	error = wldev_iovar_setint(dev, "roam_off", mode);
	if (error) {
		DHD_ERROR(("%s: Failed to set roaming Mode %d, error = %d\n",
		__FUNCTION__, mode, error));
		return -1;
	}
	else
		DHD_ERROR(("%s: succeeded to set roaming Mode %d, error = %d\n",
		__FUNCTION__, mode, error));
	return 0;
}

int wl_android_set_ibss_beacon_ouidata(struct net_device *dev, char *command, int total_len)
{
	char ie_buf[VNDR_IE_MAX_LEN];
	char *ioctl_buf = NULL;
	char hex[] = "XX";
	char *pcmd = NULL;
	int ielen = 0, datalen = 0, idx = 0, tot_len = 0;
	vndr_ie_setbuf_t *vndr_ie = NULL;
	s32 iecount;
	uint32 pktflag;
	u16 kflags = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;
	s32 err = BCME_OK;

	if (wl_cfg80211_ibss_vsie_delete(dev) != BCME_OK) {
		return -EINVAL;
	}

	pcmd = command + strlen(CMD_SETIBSSBEACONOUIDATA) + 1;
	for (idx = 0; idx < DOT11_OUI_LEN; idx++) {
		hex[0] = *pcmd++;
		hex[1] = *pcmd++;
		ie_buf[idx] =  (uint8)simple_strtoul(hex, NULL, 16);
	}
	pcmd++;
	while ((*pcmd != '\0') && (idx < VNDR_IE_MAX_LEN)) {
		hex[0] = *pcmd++;
		hex[1] = *pcmd++;
		ie_buf[idx++] =  (uint8)simple_strtoul(hex, NULL, 16);
		datalen++;
	}
	tot_len = sizeof(vndr_ie_setbuf_t) + (datalen - 1);
	vndr_ie = (vndr_ie_setbuf_t *) kzalloc(tot_len, kflags);
	if (!vndr_ie) {
		WL_ERR(("IE memory alloc failed\n"));
		return -ENOMEM;
	}
	
	strncpy(vndr_ie->cmd, "add", VNDR_IE_CMD_LEN - 1);
	vndr_ie->cmd[VNDR_IE_CMD_LEN - 1] = '\0';

	
	iecount = htod32(1);
	memcpy((void *)&vndr_ie->vndr_ie_buffer.iecount, &iecount, sizeof(s32));

	
	pktflag = htod32(VNDR_IE_BEACON_FLAG | VNDR_IE_PRBRSP_FLAG);
	memcpy((void *)&vndr_ie->vndr_ie_buffer.vndr_ie_list[0].pktflag, &pktflag,
		sizeof(u32));
	
	vndr_ie->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.id = (uchar) DOT11_MNG_PROPR_ID;

	memcpy(&vndr_ie->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.oui, &ie_buf,
		DOT11_OUI_LEN);
	memcpy(&vndr_ie->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.data,
		&ie_buf[DOT11_OUI_LEN], datalen);

	ielen = DOT11_OUI_LEN + datalen;
	vndr_ie->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.len = (uchar) ielen;

	ioctl_buf = kmalloc(WLC_IOCTL_MEDLEN, GFP_KERNEL);
	if (!ioctl_buf) {
		WL_ERR(("ioctl memory alloc failed\n"));
		if (vndr_ie) {
			kfree(vndr_ie);
		}
		return -ENOMEM;
	}
	memset(ioctl_buf, 0, WLC_IOCTL_MEDLEN);	
	err = wldev_iovar_setbuf(dev, "ie", vndr_ie, tot_len, ioctl_buf, WLC_IOCTL_MEDLEN, NULL);


	if (err != BCME_OK) {
		err = -EINVAL;
		if (vndr_ie) {
			kfree(vndr_ie);
		}
	}
	else {
		
		wl_cfg80211_ibss_vsie_set_buffer(vndr_ie, tot_len);
	}

	if (ioctl_buf) {
		kfree(ioctl_buf);
	}

	return err;
}

#if defined(BCMFW_ROAM_ENABLE)
static int
wl_android_set_roampref(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	char smbuf[WLC_IOCTL_SMLEN];
	uint8 buf[MAX_BUF_SIZE];
	uint8 *pref = buf;
	char *pcmd;
	int num_ucipher_suites = 0;
	int num_akm_suites = 0;
	wpa_suite_t ucipher_suites[MAX_NUM_SUITES];
	wpa_suite_t akm_suites[MAX_NUM_SUITES];
	int num_tuples = 0;
	int total_bytes = 0;
	int total_len_left;
	int i, j;
	char hex[] = "XX";

	pcmd = command + strlen(CMD_SET_ROAMPREF) + 1;
	total_len_left = total_len - strlen(CMD_SET_ROAMPREF) + 1;

	num_akm_suites = simple_strtoul(pcmd, NULL, 16);
	
	pcmd += 3;
	total_len_left -= 3;

	
	if (total_len_left < (num_akm_suites * WIDTH_AKM_SUITE))
		return -1;

	memset(buf, 0, sizeof(buf));
	memset(akm_suites, 0, sizeof(akm_suites));
	memset(ucipher_suites, 0, sizeof(ucipher_suites));

	
	for (i = 0; i < num_akm_suites; i++) {
		
		for (j = 0; j < 4; j++) {
			hex[0] = *pcmd++;
			hex[1] = *pcmd++;
			buf[j] = (uint8)simple_strtoul(hex, NULL, 16);
		}
		memcpy((uint8 *)&akm_suites[i], buf, sizeof(uint32));
	}

	total_len_left -= (num_akm_suites * WIDTH_AKM_SUITE);
	num_ucipher_suites = simple_strtoul(pcmd, NULL, 16);
	
	pcmd += 3;
	total_len_left -= 3;

	if (total_len_left < (num_ucipher_suites * WIDTH_AKM_SUITE))
		return -1;

	
	for (i = 0; i < num_ucipher_suites; i++) {
		
		for (j = 0; j < 4; j++) {
			hex[0] = *pcmd++;
			hex[1] = *pcmd++;
			buf[j] = (uint8)simple_strtoul(hex, NULL, 16);
		}
		memcpy((uint8 *)&ucipher_suites[i], buf, sizeof(uint32));
	}

	*pref++ = WL_JOIN_PREF_RSSI;
	*pref++ = JOIN_PREF_RSSI_LEN;
	*pref++ = 0;
	*pref++ = 0;

	num_tuples = num_akm_suites * num_ucipher_suites;
	if (num_tuples != 0) {
		if (num_tuples <= JOIN_PREF_MAX_WPA_TUPLES) {
			*pref++ = WL_JOIN_PREF_WPA;
			*pref++ = 0;
			*pref++ = 0;
			*pref++ = (uint8)num_tuples;
			total_bytes = JOIN_PREF_RSSI_SIZE + JOIN_PREF_WPA_HDR_SIZE +
				(JOIN_PREF_WPA_TUPLE_SIZE * num_tuples);
		} else {
			DHD_ERROR(("%s: Too many wpa configs for join_pref \n", __FUNCTION__));
			return -1;
		}
	} else {
		
		total_bytes = JOIN_PREF_RSSI_SIZE;
	}

	
	for (i = 0; i < num_ucipher_suites; i++) {
		for (j = 0; j < num_akm_suites; j++) {
			memcpy(pref, (uint8 *)&akm_suites[j], WPA_SUITE_LEN);
			pref += WPA_SUITE_LEN;
			memcpy(pref, (uint8 *)&ucipher_suites[i], WPA_SUITE_LEN);
			pref += WPA_SUITE_LEN;
			
			memset(pref, 0, WPA_SUITE_LEN);
			pref += WPA_SUITE_LEN;
		}
	}

	prhex("join pref", (uint8 *)buf, total_bytes);
	error = wldev_iovar_setbuf(dev, "join_pref", buf, total_bytes, smbuf, sizeof(smbuf), NULL);
	if (error) {
		DHD_ERROR(("Failed to set join_pref, error = %d\n", error));
	}
	return error;
}
#endif 

static int
wl_android_iolist_add(struct net_device *dev, struct list_head *head, struct io_cfg *config)
{
	struct io_cfg *resume_cfg;
	s32 ret;

	resume_cfg = kzalloc(sizeof(struct io_cfg), GFP_KERNEL);
	if (!resume_cfg)
		return -ENOMEM;

	if (config->iovar) {
		ret = wldev_iovar_getint(dev, config->iovar, &resume_cfg->param);
		if (ret) {
			DHD_ERROR(("%s: Failed to get current %s value\n",
				__FUNCTION__, config->iovar));
			goto error;
		}

		ret = wldev_iovar_setint(dev, config->iovar, config->param);
		if (ret) {
			DHD_ERROR(("%s: Failed to set %s to %d\n", __FUNCTION__,
				config->iovar, config->param));
			goto error;
		}

		resume_cfg->iovar = config->iovar;
	} else {
		resume_cfg->arg = kzalloc(config->len, GFP_KERNEL);
		if (!resume_cfg->arg) {
			ret = -ENOMEM;
			goto error;
		}
		ret = wldev_ioctl(dev, config->ioctl, resume_cfg->arg, config->len, false);
		if (ret) {
			DHD_ERROR(("%s: Failed to get ioctl %d\n", __FUNCTION__,
				config->ioctl));
			goto error;
		}
		ret = wldev_ioctl(dev, config->ioctl + 1, config->arg, config->len, true);
		if (ret) {
			DHD_ERROR(("%s: Failed to set %s to %d\n", __FUNCTION__,
				config->iovar, config->param));
			goto error;
		}
		if (config->ioctl + 1 == WLC_SET_PM)
			wl_cfg80211_update_power_mode(dev);
		resume_cfg->ioctl = config->ioctl;
		resume_cfg->len = config->len;
	}

	list_add(&resume_cfg->list, head);

	return 0;
error:
	kfree(resume_cfg->arg);
	kfree(resume_cfg);
	return ret;
}

static void
wl_android_iolist_resume(struct net_device *dev, struct list_head *head)
{
	struct io_cfg *config;
	struct list_head *cur, *q;
	s32 ret = 0;

	list_for_each_safe(cur, q, head) {
		config = list_entry(cur, struct io_cfg, list);
		if (config->iovar) {
			if (!ret)
				ret = wldev_iovar_setint(dev, config->iovar,
					config->param);
		} else {
			if (!ret)
				ret = wldev_ioctl(dev, config->ioctl + 1,
					config->arg, config->len, true);
			if (config->ioctl + 1 == WLC_SET_PM)
				wl_cfg80211_update_power_mode(dev);
			kfree(config->arg);
		}
		list_del(cur);
		kfree(config);
	}
}

static int
wl_android_set_miracast(struct net_device *dev, char *command, int total_len)
{
	int mode, val;
	int ret = 0;
	struct io_cfg config;

	if (sscanf(command, "%*s %d", &mode) != 1) {
		DHD_ERROR(("%s: Failed to get Parameter\n", __FUNCTION__));
		return -1;
	}

	DHD_INFO(("%s: enter miracast mode %d\n", __FUNCTION__, mode));

	if (miracast_cur_mode == mode) {
		return 0;
	}

	wl_android_iolist_resume(dev, &miracast_resume_list);
	miracast_cur_mode = MIRACAST_MODE_OFF;

	switch (mode) {
	case MIRACAST_MODE_SOURCE:
		
		config.iovar = "mchan_algo";

		ret = wldev_ioctl(dev, WLC_GET_BCNPRD, &val, sizeof(int), false);
		if (!ret && val > 100) {
			config.param = 0;
			DHD_ERROR(("%s: Connected station's beacon interval: "
				"%d and set mchan_algo to %d \n",
				__FUNCTION__, val, config.param));
		} else {
			config.param = MIRACAST_MCHAN_ALGO;
		}
		ret = wl_android_iolist_add(dev, &miracast_resume_list, &config);
		if (ret) {
			goto resume;
		}

		
		config.iovar = "mchan_bw";
		config.param = MIRACAST_MCHAN_BW;
		ret = wl_android_iolist_add(dev, &miracast_resume_list, &config);
		if (ret) {
			goto resume;
		}

		
		config.iovar = "ampdu_mpdu";
		config.param = MIRACAST_AMPDU_SIZE;
		ret = wl_android_iolist_add(dev, &miracast_resume_list, &config);
		if (ret) {
			goto resume;
		}
		
	case MIRACAST_MODE_SINK:
		
		config.iovar = "roam_off";
		config.param = 1;
		ret = wl_android_iolist_add(dev, &miracast_resume_list, &config);
		if (ret) {
			goto resume;
		}

		
		ret = wldev_ioctl(dev, WLC_GET_PM, &val, sizeof(val), false);
		if (ret) {
			goto resume;
		}

		if (val != PM_OFF) {
			val = PM_OFF;
			config.iovar = NULL;
			config.ioctl = WLC_GET_PM;
			config.arg = &val;
			config.len = sizeof(int);
			ret = wl_android_iolist_add(dev, &miracast_resume_list, &config);
			if (ret) {
				goto resume;
			}
		}

		break;
	case MIRACAST_MODE_OFF:
	default:
		break;
	}
	miracast_cur_mode = mode;

	return 0;

resume:
	DHD_ERROR(("%s: turnoff miracast mode because of err%d\n", __FUNCTION__, ret));
	wl_android_iolist_resume(dev, &miracast_resume_list);
	return ret;
}

#define NETLINK_OXYGEN     30
#define AIBSS_BEACON_TIMEOUT	10

static struct sock *nl_sk = NULL;

static void wl_netlink_recv(struct sk_buff *skb)
{
	WL_ERR(("netlink_recv called\n"));
}

static int wl_netlink_init(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
	struct netlink_kernel_cfg cfg = {
		.input	= wl_netlink_recv,
	};
#endif

	if (nl_sk != NULL) {
		WL_ERR(("nl_sk already exist\n"));
		return BCME_ERROR;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0))
	nl_sk = netlink_kernel_create(&init_net, NETLINK_OXYGEN,
		0, wl_netlink_recv, NULL, THIS_MODULE);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0))
	nl_sk = netlink_kernel_create(&init_net, NETLINK_OXYGEN, THIS_MODULE, &cfg);
#else
	nl_sk = netlink_kernel_create(&init_net, NETLINK_OXYGEN, &cfg);
#endif

	if (nl_sk == NULL) {
		WL_ERR(("nl_sk is not ready\n"));
		return BCME_ERROR;
	}

	return BCME_OK;
}

static void wl_netlink_deinit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

s32
wl_netlink_send_msg(int pid, int type, int seq, void *data, size_t size)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int ret = -1;

	if (nl_sk == NULL) {
		WL_ERR(("nl_sk was not initialized\n"));
		goto nlmsg_failure;
	}

	skb = alloc_skb(NLMSG_SPACE(size), GFP_ATOMIC);
	if (skb == NULL) {
		WL_ERR(("failed to allocate memory\n"));
		goto nlmsg_failure;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, size, 0);
	if (nlh == NULL) {
		WL_ERR(("failed to build nlmsg, skb_tailroom:%d, nlmsg_total_size:%d\n",
			skb_tailroom(skb), nlmsg_total_size(size)));
		dev_kfree_skb(skb);
		goto nlmsg_failure;
	}

	memcpy(nlmsg_data(nlh), data, size);
	nlh->nlmsg_seq = seq;
	nlh->nlmsg_type = type;

	
	ret = netlink_unicast(nl_sk, skb, pid, 0);
	WL_DBG(("netlink_unicast() pid=%d, ret=%d\n", pid, ret));

nlmsg_failure:
	return ret;
}

#ifdef WLAIBSS
static int wl_android_set_ibss_txfail_event(struct net_device *dev, char *command, int total_len)
{
	int err = 0;
	int retry = 0;
	int pid = 0;
	aibss_txfail_config_t txfail_config = {0, 0, 0, 0};
	char smbuf[WLC_IOCTL_SMLEN];

	if (sscanf(command, CMD_SETIBSSTXFAILEVENT " %d %d", &retry, &pid) <= 0) {
		WL_ERR(("Failed to get Parameter from : %s\n", command));
		return -1;
	}

	
	wl_cfg80211_set_txfail_pid(pid);

	
	if (retry > 0) {
		txfail_config.max_tx_retry = retry;
		txfail_config.bcn_timeout = 0;	
	}
	txfail_config.version = AIBSS_TXFAIL_CONFIG_VER_0;
	txfail_config.len = sizeof(txfail_config);

	err = wldev_iovar_setbuf(dev, "aibss_txfail_config", (void *) &txfail_config,
		sizeof(aibss_txfail_config_t), smbuf, WLC_IOCTL_SMLEN, NULL);
	WL_DBG(("retry=%d, pid=%d, err=%d\n", retry, pid, err));

	return ((err == 0)?total_len:err);
}

static int wl_android_get_ibss_peer_info(struct net_device *dev, char *command,
	int total_len, bool bAll)
{
	int error;
	int bytes_written = 0;
	void *buf = NULL;
	bss_peer_list_info_t peer_list_info;
	bss_peer_info_t *peer_info;
	int i;
	bool found = false;
	struct ether_addr mac_ea;

	WL_DBG(("get ibss peer info(%s)\n", bAll?"true":"false"));

	if (!bAll) {
		if (sscanf (command, "GETIBSSPEERINFO %02x:%02x:%02x:%02x:%02x:%02x",
			(unsigned int *)&mac_ea.octet[0], (unsigned int *)&mac_ea.octet[1],
			(unsigned int *)&mac_ea.octet[2], (unsigned int *)&mac_ea.octet[3],
			(unsigned int *)&mac_ea.octet[4], (unsigned int *)&mac_ea.octet[5]) != 6) {
			WL_DBG(("invalid MAC address\n"));
			return -1;
		}
	}

	if ((buf = kmalloc(WLC_IOCTL_MAXLEN, GFP_KERNEL)) == NULL) {
		WL_ERR(("kmalloc failed\n"));
		return -1;
	}

	error = wldev_iovar_getbuf(dev, "bss_peer_info", NULL, 0, buf, WLC_IOCTL_MAXLEN, NULL);
	if (unlikely(error)) {
		WL_ERR(("could not get ibss peer info (%d)\n", error));
		kfree(buf);
		return -1;
	}

	memcpy(&peer_list_info, buf, sizeof(peer_list_info));
	peer_list_info.version = htod16(peer_list_info.version);
	peer_list_info.bss_peer_info_len = htod16(peer_list_info.bss_peer_info_len);
	peer_list_info.count = htod32(peer_list_info.count);

	WL_DBG(("ver:%d, len:%d, count:%d\n", peer_list_info.version,
		peer_list_info.bss_peer_info_len, peer_list_info.count));

	if (peer_list_info.count > 0) {
		if (bAll)
			bytes_written += sprintf(&command[bytes_written], "%u ",
				peer_list_info.count);

		peer_info = (bss_peer_info_t *) ((void *)buf + BSS_PEER_LIST_INFO_FIXED_LEN);


		for (i = 0; i < peer_list_info.count; i++) {

			WL_DBG(("index:%d rssi:%d, tx:%u, rx:%u\n", i, peer_info->rssi,
				peer_info->tx_rate, peer_info->rx_rate));

			if (!bAll &&
				memcmp(&mac_ea, &peer_info->ea, sizeof(struct ether_addr)) == 0) {
				found = true;
			}

			if (bAll || found) {
				bytes_written += sprintf(&command[bytes_written], MACF,
					ETHER_TO_MACF(peer_info->ea));
				bytes_written += sprintf(&command[bytes_written], " %u %d ",
					peer_info->tx_rate/1000, peer_info->rssi);
			}

			if (found)
				break;

			peer_info = (bss_peer_info_t *)((void *)peer_info+sizeof(bss_peer_info_t));
		}
	}
	else {
		WL_ERR(("could not get ibss peer info : no item\n"));
	}
	bytes_written += sprintf(&command[bytes_written], "%s", "\0");

	WL_DBG(("command(%u):%s\n", total_len, command));
	WL_DBG(("bytes_written:%d\n", bytes_written));

	kfree(buf);
	return bytes_written;
}

int wl_android_set_ibss_routetable(struct net_device *dev, char *command, int total_len)
{

	char *pcmd = command;
	char *str = NULL;

	ibss_route_tbl_t *route_tbl = NULL;
	char *ioctl_buf = NULL;
	u16 kflags = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;
	s32 err = BCME_OK;
	uint32 route_tbl_len;
	uint32 entries;
	char *endptr;
	uint32 i = 0;
	struct ipv4_addr  dipaddr;
	struct ether_addr ea;

	route_tbl_len = sizeof(ibss_route_tbl_t) +
		(MAX_IBSS_ROUTE_TBL_ENTRY - 1) * sizeof(ibss_route_entry_t);
	route_tbl = (ibss_route_tbl_t *)kzalloc(route_tbl_len, kflags);
	if (!route_tbl) {
		WL_ERR(("Route TBL alloc failed\n"));
		return -ENOMEM;
	}
	ioctl_buf = kzalloc(WLC_IOCTL_MEDLEN, GFP_KERNEL);
	if (!ioctl_buf) {
		WL_ERR(("ioctl memory alloc failed\n"));
		if (route_tbl) {
			kfree(route_tbl);
		}
		return -ENOMEM;
	}
	memset(ioctl_buf, 0, WLC_IOCTL_MEDLEN);

	
	str = bcmstrtok(&pcmd, " ", NULL);

	
	str = bcmstrtok(&pcmd, " ",  NULL);
	if (!str) {
		WL_ERR(("Invalid number parameter %s\n", str));
		err = -EINVAL;
		goto exit;
	}
	entries = bcm_strtoul(str, &endptr, 0);
	if (*endptr != '\0') {
		WL_ERR(("Invalid number parameter %s\n", str));
		err = -EINVAL;
		goto exit;
	}
	WL_INFORM(("Routing table count:%d\n", entries));
	route_tbl->num_entry = entries;

	for (i = 0; i < entries; i++) {
		str = bcmstrtok(&pcmd, " ", NULL);
		if (!str || !bcm_atoipv4(str, &dipaddr)) {
			WL_ERR(("Invalid ip string %s\n", str));
			err = -EINVAL;
			goto exit;
		}


		str = bcmstrtok(&pcmd, " ", NULL);
		if (!str || !bcm_ether_atoe(str, &ea)) {
			WL_ERR(("Invalid ethernet string %s\n", str));
			err = -EINVAL;
			goto exit;
		}
		bcopy(&dipaddr, &route_tbl->route_entry[i].ipv4_addr, IPV4_ADDR_LEN);
		bcopy(&ea, &route_tbl->route_entry[i].nexthop, ETHER_ADDR_LEN);
	}

	route_tbl_len = sizeof(ibss_route_tbl_t) +
		((!entries?0:(entries - 1)) * sizeof(ibss_route_entry_t));
	err = wldev_iovar_setbuf(dev, "ibss_route_tbl",
		route_tbl, route_tbl_len, ioctl_buf, WLC_IOCTL_MEDLEN, NULL);
	if (err != BCME_OK) {
		WL_ERR(("Fail to set iovar %d\n", err));
		err = -EINVAL;
	}

exit:
	if (route_tbl)
		kfree(route_tbl);
	if (ioctl_buf)
		kfree(ioctl_buf);
	return err;

}

int
wl_android_set_ibss_ampdu(struct net_device *dev, char *command, int total_len)
{
	char *pcmd = command;
	char *str = NULL, *endptr = NULL;
	struct ampdu_aggr aggr;
	char smbuf[WLC_IOCTL_SMLEN];
	int idx;
	int err = 0;
	int wme_AC2PRIO[AC_COUNT][2] = {
		{PRIO_8021D_VO, PRIO_8021D_NC},		
		{PRIO_8021D_CL, PRIO_8021D_VI},		
		{PRIO_8021D_BK, PRIO_8021D_NONE},	
		{PRIO_8021D_BE, PRIO_8021D_EE}};	

	WL_DBG(("set ibss ampdu:%s\n", command));

	memset(&aggr, 0, sizeof(aggr));
	
	aggr.conf_TID_bmap = NBITMASK(NUMPRIO);

	
	
	str = bcmstrtok(&pcmd, " ", NULL);

	for (idx = 0; idx < AC_COUNT; idx++) {
		bool on;
		str = bcmstrtok(&pcmd, " ", NULL);
		if (!str) {
			WL_ERR(("Invalid parameter : %s\n", pcmd));
			return -EINVAL;
		}
		on = bcm_strtoul(str, &endptr, 0) ? TRUE : FALSE;
		if (*endptr != '\0') {
			WL_ERR(("Invalid number format %s\n", str));
			return -EINVAL;
		}
		if (on) {
			setbit(&aggr.enab_TID_bmap, wme_AC2PRIO[idx][0]);
			setbit(&aggr.enab_TID_bmap, wme_AC2PRIO[idx][1]);
		}
	}

	err = wldev_iovar_setbuf(dev, "ampdu_txaggr", (void *)&aggr,
	sizeof(aggr), smbuf, WLC_IOCTL_SMLEN, NULL);

	return ((err == 0) ? total_len : err);
}

int wl_android_set_ibss_antenna(struct net_device *dev, char *command, int total_len)
{
	char *pcmd = command;
	char *str = NULL;
	int txchain, rxchain;
	int err = 0;

	WL_DBG(("set ibss antenna:%s\n", command));

	
	
	str = bcmstrtok(&pcmd, " ", NULL);

	
	str = bcmstrtok(&pcmd, " ", NULL);
	if (!str) {
		WL_ERR(("Invalid parameter : %s\n", pcmd));
		return -EINVAL;
	}
	txchain = bcm_atoi(str);

	
	str = bcmstrtok(&pcmd, " ", NULL);
	if (!str) {
		WL_ERR(("Invalid parameter : %s\n", pcmd));
		return -EINVAL;
	}
	rxchain = bcm_atoi(str);

	err = wldev_iovar_setint(dev, "txchain", txchain);
	if (err != 0)
		return err;
	err = wldev_iovar_setint(dev, "rxchain", rxchain);
	return ((err == 0)?total_len:err);
}
#endif 

int wl_keep_alive_set(struct net_device *dev, char* extra, int total_len)
{
	char 				buf[256];
	const char 			*str;
	wl_mkeep_alive_pkt_t	mkeep_alive_pkt;
	wl_mkeep_alive_pkt_t	*mkeep_alive_pktp;
	int					buf_len;
	int					str_len;
	int res 				= -1;
	uint period_msec = 0;

	if (extra == NULL)
	{
		 DHD_ERROR(("%s: extra is NULL\n", __FUNCTION__));
		 return -1;
	}
	if (sscanf(extra, "%d", &period_msec) != 1)
	{
		 DHD_ERROR(("%s: sscanf error. check period_msec value\n", __FUNCTION__));
		 return -EINVAL;
	}
	DHD_ERROR(("%s: period_msec is %d\n", __FUNCTION__, period_msec));

	memset(&mkeep_alive_pkt, 0, sizeof(wl_mkeep_alive_pkt_t));

	str = "mkeep_alive";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[ str_len ] = '\0';
	mkeep_alive_pktp = (wl_mkeep_alive_pkt_t *) (buf + str_len + 1);
	mkeep_alive_pkt.period_msec = period_msec;
	buf_len = str_len + 1;
	mkeep_alive_pkt.version = htod16(WL_MKEEP_ALIVE_VERSION);
	mkeep_alive_pkt.length = htod16(WL_MKEEP_ALIVE_FIXED_LEN);

	
	mkeep_alive_pkt.keep_alive_id = 0;
	mkeep_alive_pkt.len_bytes = 0;
	buf_len += WL_MKEEP_ALIVE_FIXED_LEN;
	memcpy((char *)mkeep_alive_pktp, &mkeep_alive_pkt, WL_MKEEP_ALIVE_FIXED_LEN);

	if ((res = wldev_ioctl(dev, WLC_SET_VAR, buf, buf_len, TRUE)) < 0)
	{
		DHD_ERROR(("%s:keep_alive set failed. res[%d]\n", __FUNCTION__, res));
	}
	else
	{
		DHD_ERROR(("%s:keep_alive set ok. res[%d]\n", __FUNCTION__, res));
	}

	return res;
}


static const char *
get_string_by_separator(char *result, int result_len, const char *src, char separator)
{
	char *end = result + result_len - 1;
	while ((result != end) && (*src != separator) && (*src)) {
		*result++ = *src++;
	}
	*result = 0;
	if (*src == separator)
		++src;
	return src;
}

int
wl_android_set_roam_offload_bssid_list(struct net_device *dev, const char *cmd)
{
	char sbuf[32];
	int i, cnt, size, err, ioctl_buf_len;
	roamoffl_bssid_list_t *bssid_list;
	const char *str = cmd;
	char *ioctl_buf;

	str = get_string_by_separator(sbuf, 32, str, ',');
	cnt = bcm_atoi(sbuf);
	cnt = MIN(cnt, MAX_ROAMOFFL_BSSID_NUM);
	size = sizeof(int) + sizeof(struct ether_addr) * cnt;
	WL_ERR(("ROAM OFFLOAD BSSID LIST %d BSSIDs, size %d\n", cnt, size));
	bssid_list = kmalloc(size, GFP_KERNEL);
	if (bssid_list == NULL) {
		WL_ERR(("%s: memory alloc for bssid list(%d) failed\n",
			__FUNCTION__, size));
		return -ENOMEM;
	}
	ioctl_buf_len = size + 64;
	ioctl_buf = kmalloc(ioctl_buf_len, GFP_KERNEL);
	if (ioctl_buf == NULL) {
		WL_ERR(("%s: memory alloc for ioctl_buf(%d) failed\n",
			__FUNCTION__, ioctl_buf_len));
		kfree(bssid_list);
		return -ENOMEM;
	}

	for (i = 0; i < cnt; i++) {
		str = get_string_by_separator(sbuf, 32, str, ',');
		if (bcm_ether_atoe(sbuf, &bssid_list->bssid[i]) == 0) {
			DHD_ERROR(("%s: Invalid station MAC Address!!!\n", __FUNCTION__));
			kfree(bssid_list);
			kfree(ioctl_buf);
			return -1;
		}
	}

	bssid_list->cnt = cnt;
	err = wldev_iovar_setbuf(dev, "roamoffl_bssid_list",
		bssid_list, size, ioctl_buf, ioctl_buf_len, NULL);
	kfree(bssid_list);
	kfree(ioctl_buf);

	return err;
}

#ifdef P2PRESP_WFDIE_SRC
static int wl_android_get_wfdie_resp(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int bytes_written = 0;
	int only_resp_wfdsrc = 0;

	error = wldev_iovar_getint(dev, "p2p_only_resp_wfdsrc", &only_resp_wfdsrc);
	if (error) {
		DHD_ERROR(("%s: Failed to get the mode for only_resp_wfdsrc, error = %d\n",
			__FUNCTION__, error));
		return -1;
	}

	bytes_written = snprintf(command, total_len, "%s %d",
		CMD_P2P_GET_WFDIE_RESP, only_resp_wfdsrc);

	return bytes_written;
}

static int wl_android_set_wfdie_resp(struct net_device *dev, int only_resp_wfdsrc)
{
	int error = 0;

	error = wldev_iovar_setint(dev, "p2p_only_resp_wfdsrc", only_resp_wfdsrc);
	if (error) {
		DHD_ERROR(("%s: Failed to set only_resp_wfdsrc %d, error = %d\n",
			__FUNCTION__, only_resp_wfdsrc, error));
		return -1;
	}

	return 0;
}
#endif 

static int wl_android_get_link_status(struct net_device *dev, char *command,
	int total_len)
{
	int bytes_written, error, result = 0, single_stream, stf = -1, i, nss = 0, mcs_map;
	uint32 rspec;
	uint encode, rate, txexp;
	struct wl_bss_info *bi;
	int datalen = sizeof(uint32) + sizeof(wl_bss_info_t);
	char buf[datalen];

	
	*(u32 *) buf = htod32(datalen);
	error = wldev_ioctl(dev, WLC_GET_BSS_INFO, (void *)buf, datalen, false);
	if (unlikely(error)) {
		WL_ERR(("Could not get bss info %d\n", error));
		return -1;
	}

	bi = (struct wl_bss_info *) (buf + sizeof(uint32));

	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		if (bi->BSSID.octet[i] > 0) {
			break;
		}
	}

	if (i == ETHER_ADDR_LEN) {
		WL_DBG(("No BSSID\n"));
		return -1;
	}

	
	if (bi->vht_cap) {
		if (CHSPEC_IS5G(bi->chanspec)) {
			result |= WL_ANDROID_LINK_AP_VHT_SUPPORT;
		}
	}

	
	error = wldev_iovar_getint(dev, "nrate", &rspec);
	if (unlikely(error) || rspec == 0) {
		WL_ERR(("get link status error (%d)\n", error));
		return -1;
	}

	encode = (rspec & WL_RSPEC_ENCODING_MASK);
	rate = (rspec & WL_RSPEC_RATE_MASK);
	txexp = (rspec & WL_RSPEC_TXEXP_MASK) >> WL_RSPEC_TXEXP_SHIFT;

	switch (encode) {
	case WL_RSPEC_ENCODE_HT:
		
		for (i = 0; i < MAX_STREAMS_SUPPORTED; i++) {
			int8 bitmap = 0xFF;
			if (i == MAX_STREAMS_SUPPORTED-1) {
				bitmap = 0x7F;
			}
			if (bi->basic_mcs[i] & bitmap) {
				nss++;
			}
		}
		break;
	case WL_RSPEC_ENCODE_VHT:
		
		for (i = 1; i <= VHT_CAP_MCS_MAP_NSS_MAX; i++) {
			mcs_map = VHT_MCS_MAP_GET_MCS_PER_SS(i, dtoh16(bi->vht_rxmcsmap));
			if (mcs_map != VHT_CAP_MCS_MAP_NONE) {
				nss++;
			}
		}
		break;
	}

	
	if (nss > 1) {
		result |= WL_ANDROID_LINK_AP_MIMO_SUPPORT;
	}

	single_stream = (encode == WL_RSPEC_ENCODE_RATE) ||
		((encode == WL_RSPEC_ENCODE_HT) && rate < 8) ||
		((encode == WL_RSPEC_ENCODE_VHT) &&
		((rspec & WL_RSPEC_VHT_NSS_MASK) >> WL_RSPEC_VHT_NSS_SHIFT) == 1);

	if (txexp == 0) {
		if ((rspec & WL_RSPEC_STBC) && single_stream) {
			stf = OLD_NRATE_STF_STBC;
		} else {
			stf = (single_stream) ? OLD_NRATE_STF_SISO : OLD_NRATE_STF_SDM;
		}
	} else if (txexp == 1 && single_stream) {
		stf = OLD_NRATE_STF_CDD;
	}

	
	if (encode == WL_RSPEC_ENCODE_VHT) {
		if (CHSPEC_IS5G(bi->chanspec)) {
			result |= WL_ANDROID_LINK_VHT;
		}
	}

	
	if (result & WL_ANDROID_LINK_AP_MIMO_SUPPORT) {
		switch (stf) {
		case OLD_NRATE_STF_SISO:
			break;
		case OLD_NRATE_STF_CDD:
		case OLD_NRATE_STF_STBC:
			result |= WL_ANDROID_LINK_MIMO;
			break;
		case OLD_NRATE_STF_SDM:
			if (!single_stream) {
				result |= WL_ANDROID_LINK_MIMO;
			}
			break;
		}
	}

	WL_DBG(("%s:result=%d, stf=%d, single_stream=%d, mcs map=%d\n",
		__FUNCTION__, result, stf, single_stream, nss));

	bytes_written = sprintf(command, "%s %d", CMD_GET_LINK_STATUS, result);

	return bytes_written;
}


int wl_android_priv_cmd(struct net_device *net, struct ifreq *ifr, int cmd)
{
#define PRIVATE_COMMAND_MAX_LEN	8192
	int ret = 0;
	char *command = NULL;
	int bytes_written = 0;
	android_wifi_priv_cmd priv_cmd;

	net_os_wake_lock(net);

	if (!ifr->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		compat_android_wifi_priv_cmd compat_priv_cmd;
		if (copy_from_user(&compat_priv_cmd, ifr->ifr_data,
			sizeof(compat_android_wifi_priv_cmd))) {
			ret = -EFAULT;
			goto exit;

		}
		priv_cmd.buf = compat_ptr(compat_priv_cmd.buf);
		priv_cmd.used_len = compat_priv_cmd.used_len;
		priv_cmd.total_len = compat_priv_cmd.total_len;
	} else
#endif 
	{
		if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(android_wifi_priv_cmd))) {
			ret = -EFAULT;
			goto exit;
		}
	}
	if ((priv_cmd.total_len > PRIVATE_COMMAND_MAX_LEN) || (priv_cmd.total_len < 0)) {
		DHD_ERROR(("%s: too long priavte command\n", __FUNCTION__));
		ret = -EINVAL;
		goto exit;
	}
	command = kmalloc((priv_cmd.total_len + 1), GFP_KERNEL);
	if (!command)
	{
		DHD_ERROR(("%s: failed to allocate memory\n", __FUNCTION__));
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto exit;
	}
	command[priv_cmd.total_len] = '\0';

	DHD_INFO(("%s: Android private cmd \"%s\" on %s\n", __FUNCTION__, command, ifr->ifr_name));

	if (strnicmp(command, CMD_START, strlen(CMD_START)) == 0) {
		DHD_INFO(("%s, Received regular START command\n", __FUNCTION__));
		bytes_written = wl_android_wifi_on(net);
	}
	else if (strnicmp(command, CMD_SETFWPATH, strlen(CMD_SETFWPATH)) == 0) {
		bytes_written = wl_android_set_fwpath(net, command, priv_cmd.total_len);
	}

	if (!g_wifi_on) {
		DHD_ERROR(("%s: Ignore private cmd \"%s\" - iface %s is down\n",
			__FUNCTION__, command, ifr->ifr_name));
		ret = 0;
		goto exit;
	}

	if (strnicmp(command, CMD_STOP, strlen(CMD_STOP)) == 0) {
		bytes_written = wl_android_wifi_off(net);
	}
	else if (strnicmp(command, CMD_SCAN_ACTIVE, strlen(CMD_SCAN_ACTIVE)) == 0) {
		
	}
	else if (strnicmp(command, CMD_SCAN_PASSIVE, strlen(CMD_SCAN_PASSIVE)) == 0) {
		
	}
	else if (strnicmp(command, CMD_RSSI, strlen(CMD_RSSI)) == 0) {
		bytes_written = wl_android_get_rssi(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_LINKSPEED, strlen(CMD_LINKSPEED)) == 0) {
		bytes_written = wl_android_get_link_speed(net, command, priv_cmd.total_len);
	}
#ifdef PKT_FILTER_SUPPORT
	else if (strnicmp(command, CMD_RXFILTER_START, strlen(CMD_RXFILTER_START)) == 0) {
#ifdef CUSTOMER_HW_ONE
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
#else
		bytes_written = net_os_enable_packet_filter(net, 1);
#endif
	}
	else if (strnicmp(command, CMD_RXFILTER_STOP, strlen(CMD_RXFILTER_STOP)) == 0) {
#ifdef CUSTOMER_HW_ONE
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
#else
		bytes_written = net_os_enable_packet_filter(net, 0);
#endif
	}
	else if (strnicmp(command, CMD_RXFILTER_ADD, strlen(CMD_RXFILTER_ADD)) == 0) {
#ifdef CUSTOMER_HW_ONE
		
		DHD_ERROR(("RXFILTER-ADD MULTICAST filter\n"));
		wl_android_enable_pktfilter(net, 1);
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
#else
		int filter_num = *(command + strlen(CMD_RXFILTER_ADD) + 1) - '0';
		bytes_written = net_os_rxfilter_add_remove(net, TRUE, filter_num);
#endif
	}
	else if (strnicmp(command, CMD_RXFILTER_REMOVE, strlen(CMD_RXFILTER_REMOVE)) == 0) {
#ifdef CUSTOMER_HW_ONE
		DHD_ERROR(("RXFILTER-REMOVE MULTICAST filter\n"));
		wl_android_enable_pktfilter(net, 0);
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
#else
		int filter_num = *(command + strlen(CMD_RXFILTER_REMOVE) + 1) - '0';
		bytes_written = net_os_rxfilter_add_remove(net, FALSE, filter_num);
#endif
	}
#if defined(CUSTOM_PLATFORM_NV_TEGRA)
	else if (strnicmp(command, CMD_PKT_FILTER_MODE, strlen(CMD_PKT_FILTER_MODE)) == 0) {
		dhd_set_packet_filter_mode(net, &command[strlen(CMD_PKT_FILTER_MODE) + 1]);
	} else if (strnicmp(command, CMD_PKT_FILTER_PORTS, strlen(CMD_PKT_FILTER_PORTS)) == 0) {
		bytes_written = dhd_set_packet_filter_ports(net,
			&command[strlen(CMD_PKT_FILTER_PORTS) + 1]);
		ret = bytes_written;
	}
#endif 
#endif 
	else if (strnicmp(command, CMD_BTCOEXSCAN_START, strlen(CMD_BTCOEXSCAN_START)) == 0) {
		
	}
	else if (strnicmp(command, CMD_BTCOEXSCAN_STOP, strlen(CMD_BTCOEXSCAN_STOP)) == 0) {
		
	}
	else if (strnicmp(command, CMD_BTCOEXMODE, strlen(CMD_BTCOEXMODE)) == 0) {
#ifdef WL_CFG80211
		void *dhdp = wl_cfg80211_get_dhdp();
		bytes_written = wl_cfg80211_set_btcoex_dhcp(net, dhdp, command);
#else
#ifdef PKT_FILTER_SUPPORT
		uint mode = *(command + strlen(CMD_BTCOEXMODE) + 1) - '0';

		if (mode == 1)
			net_os_enable_packet_filter(net, 0); 
		else
			net_os_enable_packet_filter(net, 1); 
#endif 
#endif 
	}
	else if (strnicmp(command, CMD_SETSUSPENDOPT, strlen(CMD_SETSUSPENDOPT)) == 0) {
		bytes_written = wl_android_set_suspendopt(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETSUSPENDMODE, strlen(CMD_SETSUSPENDMODE)) == 0) {
		bytes_written = wl_android_set_suspendmode(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETBAND, strlen(CMD_SETBAND)) == 0) {
		uint band = *(command + strlen(CMD_SETBAND) + 1) - '0';
#ifdef WL_HOST_BAND_MGMT
		s32 ret = 0;
		if ((ret = wl_cfg80211_set_band(net, band)) < 0) {
			if (ret == BCME_UNSUPPORTED) {
				
				WL_ERR(("WL_HOST_BAND_MGMT defined, "
					"but roam_band iovar unsupported in the firmware\n"));
			} else {
				bytes_written = -1;
				goto exit;
			}
		}
		if ((band == WLC_BAND_AUTO) || (ret == BCME_UNSUPPORTED))
			bytes_written = wldev_set_band(net, band);
#else
		bytes_written = wldev_set_band(net, band);
#endif 
	}
	else if (strnicmp(command, CMD_GETBAND, strlen(CMD_GETBAND)) == 0) {
		bytes_written = wl_android_get_band(net, command, priv_cmd.total_len);
	}
#ifdef WL_CFG80211
	
	else if (strnicmp(command, CMD_COUNTRY, strlen(CMD_COUNTRY)) == 0) {
#ifdef CUSTOMER_HW_ONE
		char country_code[3];
		country_code[0] = *(command + strlen(CMD_COUNTRY) + 1);
		country_code[1] = *(command + strlen(CMD_COUNTRY) + 2);
		country_code[2] = '\0';
#else
		char *country_code = command + strlen(CMD_COUNTRY) + 1;
#endif
#ifdef CUSTOMER_HW5
		
		bytes_written = wldev_set_country(net, country_code, true, false);
#else
		bytes_written = wldev_set_country(net, country_code, true, true);
#endif
	}
#endif 


#ifdef PNO_SUPPORT
	else if (strnicmp(command, CMD_PNOSSIDCLR_SET, strlen(CMD_PNOSSIDCLR_SET)) == 0) {
		bytes_written = dhd_dev_pno_stop_for_ssid(net);
	}
#ifndef WL_SCHED_SCAN
	else if (strnicmp(command, CMD_PNOSETUP_SET, strlen(CMD_PNOSETUP_SET)) == 0) {
		bytes_written = wl_android_set_pno_setup(net, command, priv_cmd.total_len);
	}
#endif 
	else if (strnicmp(command, CMD_PNOENABLE_SET, strlen(CMD_PNOENABLE_SET)) == 0) {
		int enable = *(command + strlen(CMD_PNOENABLE_SET) + 1) - '0';
		bytes_written = (enable)? 0 : dhd_dev_pno_stop_for_ssid(net);
	}
	else if (strnicmp(command, CMD_WLS_BATCHING, strlen(CMD_WLS_BATCHING)) == 0) {
		bytes_written = wls_parse_batching_cmd(net, command, priv_cmd.total_len);
	}
#endif 
	else if (strnicmp(command, CMD_P2P_DEV_ADDR, strlen(CMD_P2P_DEV_ADDR)) == 0) {
		bytes_written = wl_android_get_p2p_dev_addr(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_P2P_SET_NOA, strlen(CMD_P2P_SET_NOA)) == 0) {
		int skip = strlen(CMD_P2P_SET_NOA) + 1;
		bytes_written = wl_cfg80211_set_p2p_noa(net, command + skip,
			priv_cmd.total_len - skip);
	}
#ifdef WL_SDO
	else if (strnicmp(command, CMD_P2P_SD_OFFLOAD, strlen(CMD_P2P_SD_OFFLOAD)) == 0) {
		u8 *buf = command;
		u8 *cmd_id = NULL;
		int len;

		cmd_id = strsep((char **)&buf, " ");
		
		if (buf == NULL)
			len = 0;
		else
			len = strlen(buf);

		bytes_written = wl_cfg80211_sd_offload(net, cmd_id, buf, len);
	}
#endif 
#ifdef WL_NAN
	else if (strnicmp(command, CMD_NAN, strlen(CMD_NAN)) == 0) {
		bytes_written = wl_cfg80211_nan_cmd_handler(net, command,
			priv_cmd.total_len);
	}
#endif 
#if !defined WL_ENABLE_P2P_IF
	else if (strnicmp(command, CMD_P2P_GET_NOA, strlen(CMD_P2P_GET_NOA)) == 0) {
		bytes_written = wl_cfg80211_get_p2p_noa(net, command, priv_cmd.total_len);
	}
#endif 
	else if (strnicmp(command, CMD_P2P_SET_PS, strlen(CMD_P2P_SET_PS)) == 0) {
		int skip = strlen(CMD_P2P_SET_PS) + 1;
		bytes_written = wl_cfg80211_set_p2p_ps(net, command + skip,
			priv_cmd.total_len - skip);
	}
#ifdef WL_CFG80211
	else if (strnicmp(command, CMD_SET_AP_WPS_P2P_IE,
		strlen(CMD_SET_AP_WPS_P2P_IE)) == 0) {
		int skip = strlen(CMD_SET_AP_WPS_P2P_IE) + 3;
		bytes_written = wl_cfg80211_set_wps_p2p_ie(net, command + skip,
			priv_cmd.total_len - skip, *(command + skip - 2) - '0');
	}
#ifdef WLFBT
	else if (strnicmp(command, CMD_GET_FTKEY, strlen(CMD_GET_FTKEY)) == 0) {
		wl_cfg80211_get_fbt_key(command);
		bytes_written = FBT_KEYLEN;
	}
#endif 
#endif 
	else if (strnicmp(command, CMD_OKC_SET_PMK, strlen(CMD_OKC_SET_PMK)) == 0)
		bytes_written = wl_android_set_pmk(net, command, priv_cmd.total_len);
	else if (strnicmp(command, CMD_OKC_ENABLE, strlen(CMD_OKC_ENABLE)) == 0)
		bytes_written = wl_android_okc_enable(net, command, priv_cmd.total_len);
#ifdef BCMCCX
	else if (strnicmp(command, CMD_GETCCKM_RN, strlen(CMD_GETCCKM_RN)) == 0) {
		bytes_written = wl_android_get_cckm_rn(net, command);
	}
	else if (strnicmp(command, CMD_SETCCKM_KRK, strlen(CMD_SETCCKM_KRK)) == 0) {
		bytes_written = wl_android_set_cckm_krk(net, command);
	}
	else if (strnicmp(command, CMD_GET_ASSOC_RES_IES, strlen(CMD_GET_ASSOC_RES_IES)) == 0) {
		bytes_written = wl_android_get_assoc_res_ies(net, command);
	}
#endif 
#if defined(WL_SUPPORT_AUTO_CHANNEL)
	else if (strnicmp(command, CMD_GET_BEST_CHANNELS,
		strlen(CMD_GET_BEST_CHANNELS)) == 0) {
		bytes_written = wl_cfg80211_get_best_channels(net, command,
			priv_cmd.total_len);
	}
#endif 
#ifndef CUSTOMER_HW_ONE
	else if (strnicmp(command, CMD_HAPD_MAC_FILTER, strlen(CMD_HAPD_MAC_FILTER)) == 0) {
		int skip = strlen(CMD_HAPD_MAC_FILTER) + 1;
		wl_android_set_mac_address_filter(net, (const char*)command+skip);
	}
#endif
#ifdef CUSTOMER_HW_ONE
	else if (strnicmp(command, CMD_LTECOEX, strlen(CMD_LTECOEX)) == 0)
		bytes_written = wl_android_ltecoex_mode(net, command, priv_cmd.total_len);
#endif
	else if (strnicmp(command, CMD_SETROAMMODE, strlen(CMD_SETROAMMODE)) == 0)
		bytes_written = wl_android_set_roam_mode(net, command, priv_cmd.total_len);
#if defined(BCMFW_ROAM_ENABLE)
	else if (strnicmp(command, CMD_SET_ROAMPREF, strlen(CMD_SET_ROAMPREF)) == 0) {
		bytes_written = wl_android_set_roampref(net, command, priv_cmd.total_len);
	}
#endif 
	else if (strnicmp(command, CMD_MIRACAST, strlen(CMD_MIRACAST)) == 0)
		bytes_written = wl_android_set_miracast(net, command, priv_cmd.total_len);
#if defined(CUSTOM_PLATFORM_NV_TEGRA)
	else if (strnicmp(command, CMD_SETMIRACAST, strlen(CMD_SETMIRACAST)) == 0)
		bytes_written = wldev_miracast_tuning(net, command, priv_cmd.total_len);
	else if (strnicmp(command, CMD_ASSOCRESPIE, strlen(CMD_ASSOCRESPIE)) == 0)
		bytes_written = wldev_get_assoc_resp_ie(net, command, priv_cmd.total_len);
	else if (strnicmp(command, CMD_RXRATESTATS, strlen(CMD_RXRATESTATS)) == 0)
		bytes_written = wldev_get_rx_rate_stats(net, command, priv_cmd.total_len);
#endif 
	else if (strnicmp(command, CMD_SETIBSSBEACONOUIDATA, strlen(CMD_SETIBSSBEACONOUIDATA)) == 0)
		bytes_written = wl_android_set_ibss_beacon_ouidata(net,
		command, priv_cmd.total_len);
#ifdef WLAIBSS
	else if (strnicmp(command, CMD_SETIBSSTXFAILEVENT,
		strlen(CMD_SETIBSSTXFAILEVENT)) == 0)
		bytes_written = wl_android_set_ibss_txfail_event(net, command, priv_cmd.total_len);
	else if (strnicmp(command, CMD_GET_IBSS_PEER_INFO_ALL,
		strlen(CMD_GET_IBSS_PEER_INFO_ALL)) == 0)
		bytes_written = wl_android_get_ibss_peer_info(net, command, priv_cmd.total_len,
			TRUE);
	else if (strnicmp(command, CMD_GET_IBSS_PEER_INFO,
		strlen(CMD_GET_IBSS_PEER_INFO)) == 0)
		bytes_written = wl_android_get_ibss_peer_info(net, command, priv_cmd.total_len,
			FALSE);
	else if (strnicmp(command, CMD_SETIBSSROUTETABLE,
		strlen(CMD_SETIBSSROUTETABLE)) == 0)
		bytes_written = wl_android_set_ibss_routetable(net, command,
			priv_cmd.total_len);
	else if (strnicmp(command, CMD_SETIBSSAMPDU, strlen(CMD_SETIBSSAMPDU)) == 0)
		bytes_written = wl_android_set_ibss_ampdu(net, command, priv_cmd.total_len);
	else if (strnicmp(command, CMD_SETIBSSANTENNAMODE, strlen(CMD_SETIBSSANTENNAMODE)) == 0)
		bytes_written = wl_android_set_ibss_antenna(net, command, priv_cmd.total_len);
#endif 
	else if (strnicmp(command, CMD_KEEP_ALIVE, strlen(CMD_KEEP_ALIVE)) == 0) {
		int skip = strlen(CMD_KEEP_ALIVE) + 1;
		bytes_written = wl_keep_alive_set(net, command + skip, priv_cmd.total_len - skip);
	}
	else if (strnicmp(command, CMD_ROAM_OFFLOAD, strlen(CMD_ROAM_OFFLOAD)) == 0) {
		int enable = *(command + strlen(CMD_ROAM_OFFLOAD) + 1) - '0';
		bytes_written = wl_cfg80211_enable_roam_offload(net, enable);
	}
	else if (strnicmp(command, CMD_ROAM_OFFLOAD_APLIST, strlen(CMD_ROAM_OFFLOAD_APLIST)) == 0) {
		bytes_written = wl_android_set_roam_offload_bssid_list(net,
			command + strlen(CMD_ROAM_OFFLOAD_APLIST) + 1);
	}
#ifdef P2PRESP_WFDIE_SRC
	else if (strnicmp(command, CMD_P2P_SET_WFDIE_RESP,
		strlen(CMD_P2P_SET_WFDIE_RESP)) == 0) {
		int mode = *(command + strlen(CMD_P2P_SET_WFDIE_RESP) + 1) - '0';
		bytes_written = wl_android_set_wfdie_resp(net, mode);
	} else if (strnicmp(command, CMD_P2P_GET_WFDIE_RESP,
		strlen(CMD_P2P_GET_WFDIE_RESP)) == 0) {
		bytes_written = wl_android_get_wfdie_resp(net, command, priv_cmd.total_len);
	}
#endif 
	else if (strnicmp(command, CMD_GET_LINK_STATUS, strlen(CMD_GET_LINK_STATUS)) == 0) {
		bytes_written = wl_android_get_link_status(net, command, priv_cmd.total_len);
	}
#ifdef CONNECTION_STATISTICS
	else if (strnicmp(command, CMD_GET_CONNECTION_STATS,
		strlen(CMD_GET_CONNECTION_STATS)) == 0) {
		bytes_written = wl_android_get_connection_stats(net, command,
			priv_cmd.total_len);
	}
#endif
#ifdef CUSTOMER_HW_ONE
#if defined(HTC_TX_TRACKING)
	else if (strnicmp(command, CMD_TX_TRACKING, strlen(CMD_TX_TRACKING)) == 0) {
		bytes_written = wl_android_set_tx_tracking(net, command, priv_cmd.total_len);
	}
#endif
#ifdef WL_CFG80211
	else if (strnicmp(command, CMD_MAC_ADDR, strlen(CMD_MAC_ADDR)) == 0) {
		bytes_written = wl_android_get_mac_addr(net, command, priv_cmd.total_len);
	}
#endif
	else if (strnicmp(command, CMD_GET_TX_FAIL, strlen(CMD_GET_TX_FAIL)) == 0) {
		bytes_written = wl_android_get_tx_fail(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_P2P_SET_MPC, strlen(CMD_P2P_SET_MPC)) == 0) {
		int skip = strlen(CMD_P2P_SET_MPC) + 1;
		bytes_written = wl_cfg80211_set_mpc(net, command + skip, priv_cmd.total_len - skip);
	} else if (strnicmp(command, CMD_DEAUTH_STA, strlen(CMD_DEAUTH_STA)) == 0) {
		int skip = strlen(CMD_DEAUTH_STA) + 1;
		bytes_written =
			wl_cfg80211_deauth_sta(net, command + skip, priv_cmd.total_len - skip);
	}
	else if (strnicmp(command, CMD_DTIM_SKIP_GET, strlen(CMD_DTIM_SKIP_GET)) == 0) {
		bytes_written = wl_android_get_dtim_skip(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_DTIM_SKIP_SET, strlen(CMD_DTIM_SKIP_SET)) == 0) {
		bytes_written = wl_android_set_dtim_skip(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_TXPOWER_SET, strlen(CMD_TXPOWER_SET)) == 0) {
		bytes_written = wl_android_set_txpower(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_POWER_MODE_SET, strlen(CMD_POWER_MODE_SET)) == 0) {
		bytes_written = wl_android_set_power_mode(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_POWER_MODE_GET, strlen(CMD_POWER_MODE_GET)) == 0) {
		bytes_written = wl_android_get_power_mode(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_AP_TXPOWER_SET, strlen(CMD_AP_TXPOWER_SET)) == 0) {
		bytes_written = wl_android_set_ap_txpower(net, command, priv_cmd.total_len);
	} else if (strnicmp(command, CMD_AP_ASSOC_LIST_GET, strlen(CMD_AP_ASSOC_LIST_GET)) == 0) {
		bytes_written = wl_android_get_assoc_sta_list(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_AP_MAC_LIST_SET, strlen(CMD_AP_MAC_LIST_SET)) == 0) {
#ifdef CUSTOMER_HW_ONE
#ifdef WL_CFG80211
		dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
		dhd_info_t *dhd = DHD_DEV_INFO(dev);
		dhd_pub_t *dhdp = &(dhd->pub);
#endif 
		if (dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)
			bytes_written = wl_android_set_ap_mac_list(net, command + PROFILE_OFFSET);
		else
			DHD_ERROR(("Not in AP mode skip this %s cmmd\n", CMD_AP_MAC_LIST_SET));
#else
		bytes_written = wl_android_set_ap_mac_list(net, command + PROFILE_OFFSET);
#endif 
	}
	else if (strnicmp(command, CMD_SCAN_MINRSSI_SET, strlen(CMD_SCAN_MINRSSI_SET)) == 0) {
		bytes_written = wl_android_set_scan_minrssi(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_LOW_RSSI_SET, strlen(CMD_LOW_RSSI_SET)) == 0) {
		bytes_written = wl_android_low_rssi_set(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_GETWIFILOCK, strlen(CMD_GETWIFILOCK)) == 0) {
		bytes_written = wl_android_get_wifilock(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETWIFICALL, strlen(CMD_SETWIFICALL)) == 0) {
		bytes_written = wl_android_set_wificall(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETPROJECT, strlen(CMD_SETPROJECT)) == 0) {
		bytes_written = wl_android_set_project(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_GATEWAYADD, strlen(CMD_GATEWAYADD)) == 0) {
		bytes_written = wl_android_gateway_add(net, command, priv_cmd.total_len);
	} else if (strnicmp(command, CMD_GET_AUTO_CHANNEL, strlen(CMD_GET_AUTO_CHANNEL)) == 0) {
		int skip = strlen(CMD_GET_AUTO_CHANNEL) + 1;
		block_ap_event = 1;
		bytes_written = wl_android_auto_channel(net, command + skip,
		priv_cmd.total_len - skip) + skip;
		block_ap_event = 0;
	} else if (strnicmp(command, CMD_SET_HOTSPOT_BW, strlen(CMD_SET_HOTSPOT_BW)) == 0) {
		int skip  = strlen(CMD_SET_HOTSPOT_BW) + 1;
		int bw = bcm_atoi(command + skip);
		DHD_ERROR(("%s CMD_SET_HOTSPOT_BW  bw = %d\n", __FUNCTION__, bw));
		wl_cfg80211_set_hotspot_bw(bw);
#if defined(USE_STATIC_MEMDUMP)
	} else if (strnicmp(command, CMD_TRIG_RAMDUMP, strlen(CMD_TRIG_RAMDUMP)) == 0) {
#ifdef WL_CFG80211
		dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
		dhd_info_t *dhd = DHD_DEV_INFO(dev);
		dhd_pub_t *dhdp = &(dhd->pub);
#endif 
		DHD_ERROR(("%s CMD_TRIG_RAMDUMP \n", __FUNCTION__));
		dhd_pub_mem_dump(dhdp);
#endif 
	}
	else if (strnicmp(command, CMD_ALLOW_P2P_EVENT, strlen(CMD_ALLOW_P2P_EVENT)) == 0) {
		bytes_written = wl_android_allow_p2p_event(net, command, priv_cmd.total_len);
	}
#endif 
	else {
		DHD_ERROR(("Unknown PRIVATE command %s - ignored\n", command));
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
	}

	if (bytes_written >= 0) {
		if ((bytes_written == 0) && (priv_cmd.total_len > 0))
			command[0] = '\0';
		if (bytes_written >= priv_cmd.total_len) {
			DHD_ERROR(("%s: bytes_written = %d\n", __FUNCTION__, bytes_written));
			bytes_written = priv_cmd.total_len;
		} else {
			bytes_written++;
		}
		priv_cmd.used_len = bytes_written;
		if (copy_to_user(priv_cmd.buf, command, bytes_written)) {
			DHD_ERROR(("%s: failed to copy data to user buffer\n", __FUNCTION__));
			ret = -EFAULT;
		}
	}
	else {
		ret = bytes_written;
	}

exit:
	net_os_wake_unlock(net);
	if (command) {
		kfree(command);
	}

	return ret;
}

int wl_android_init(void)
{
	int ret = 0;

#ifdef ENABLE_INSMOD_NO_FW_LOAD
	dhd_download_fw_on_driverload = FALSE;
#endif 
#if defined(CUSTOMER_HW2)
	if (!iface_name[0]) {
		memset(iface_name, 0, IFNAMSIZ);
		bcm_strncpy_s(iface_name, IFNAMSIZ, "wlan", IFNAMSIZ);
	}
#endif 
#ifdef CUSTOMER_HW_ONE
	mutex_init(&wl_wificall_mutex);
	wlan_init_perf();
	dhd_msg_level |= DHD_ERROR_VAL | DHD_PNO_VAL;
#endif

#ifdef WL_GENL
	wl_genl_init();
#endif
	wl_netlink_init();

	return ret;
}

int wl_android_exit(void)
{
	int ret = 0;
	struct io_cfg *cur, *q;

#ifdef CUSTOMER_HW_ONE
	wlan_deinit_perf();
#endif
#ifdef WL_GENL
	wl_genl_deinit();
#endif 
	wl_netlink_deinit();

	list_for_each_entry_safe(cur, q, &miracast_resume_list, list) {
		list_del(&cur->list);
		kfree(cur);
	}

	return ret;
}

void wl_android_post_init(void)
{

#ifdef ENABLE_4335BT_WAR
	bcm_bt_unlock(lock_cookie_wifi);
	DHD_ERROR(("%s: btlock released\n", __FUNCTION__));
#endif 

	if (!dhd_download_fw_on_driverload)
		g_wifi_on = FALSE;
}

#ifdef WL_GENL
static int wl_genl_init(void)
{
	int ret;

	WL_DBG(("GEN Netlink Init\n\n"));

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
	
	ret = genl_register_family(&wl_genl_family);
	if (ret != 0)
		goto failure;

	
	ret = genl_register_ops(&wl_genl_family, &wl_genl_ops);
	if (ret != 0) {
		WL_ERR(("register ops failed: %i\n", ret));
		genl_unregister_family(&wl_genl_family);
		goto failure;
	}

	ret = genl_register_mc_group(&wl_genl_family, &wl_genl_mcast);
#else
	ret = genl_register_family_with_ops_groups(&wl_genl_family, wl_genl_ops, wl_genl_mcast);
#endif 
	if (ret != 0) {
		WL_ERR(("register mc_group failed: %i\n", ret));
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
		genl_unregister_ops(&wl_genl_family, &wl_genl_ops);
#endif
		genl_unregister_family(&wl_genl_family);
		goto failure;
	}

	return 0;

failure:
	WL_ERR(("Registering Netlink failed!!\n"));
	return -1;
}

static int wl_genl_deinit(void)
{

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
	if (genl_unregister_ops(&wl_genl_family, &wl_genl_ops) < 0)
		WL_ERR(("Unregister wl_genl_ops failed\n"));
#endif
	if (genl_unregister_family(&wl_genl_family) < 0)
		WL_ERR(("Unregister wl_genl_ops failed\n"));

	return 0;
}

s32 wl_event_to_bcm_event(u16 event_type)
{
	u16 event = -1;

	switch (event_type) {
		case WLC_E_SERVICE_FOUND:
			event = BCM_E_SVC_FOUND;
			break;
		case WLC_E_P2PO_ADD_DEVICE:
			event = BCM_E_DEV_FOUND;
			break;
		case WLC_E_P2PO_DEL_DEVICE:
			event = BCM_E_DEV_LOST;
			break;
	
#ifdef BT_WIFI_HANDOVER
		case WLC_E_BT_WIFI_HANDOVER_REQ:
			event = BCM_E_DEV_BT_WIFI_HO_REQ;
			break;
#endif 

		default:
			WL_ERR(("Event not supported\n"));
	}

	return event;
}

s32
wl_genl_send_msg(
	struct net_device *ndev,
	u32 event_type,
	u8 *buf,
	u16 len,
	u8 *subhdr,
	u16 subhdr_len)
{
	int ret = 0;
	struct sk_buff *skb = NULL;
	void *msg;
	u32 attr_type = 0;
	bcm_event_hdr_t *hdr = NULL;
	int mcast = 1; 
	int pid = 0;
	u8 *ptr = NULL, *p = NULL;
	u32 tot_len = sizeof(bcm_event_hdr_t) + subhdr_len + len;
	u16 kflags = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;


	WL_DBG(("Enter \n"));

	
	if (event_type == 0)
		attr_type = BCM_GENL_ATTR_STRING;
	else
		attr_type = BCM_GENL_ATTR_MSG;

	skb = genlmsg_new(NLMSG_GOODSIZE, kflags);
	if (skb == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	msg = genlmsg_put(skb, 0, 0, &wl_genl_family, 0, BCM_GENL_CMD_MSG);
	if (msg == NULL) {
		ret = -ENOMEM;
		goto out;
	}


	if (attr_type == BCM_GENL_ATTR_STRING) {
		if (subhdr || subhdr_len) {
			WL_ERR(("No sub hdr support for the ATTR STRING type \n"));
			ret =  -EINVAL;
			goto out;
		}

		ret = nla_put_string(skb, BCM_GENL_ATTR_STRING, buf);
		if (ret != 0) {
			WL_ERR(("nla_put_string failed\n"));
			goto out;
		}
	} else {
		

		
		p = ptr = kzalloc(tot_len, kflags);
		if (!ptr) {
			ret = -ENOMEM;
			WL_ERR(("ENOMEM!!\n"));
			goto out;
		}

		
		hdr = (bcm_event_hdr_t *)ptr;
		hdr->event_type = wl_event_to_bcm_event(event_type);
		hdr->len = len + subhdr_len;
		ptr += sizeof(bcm_event_hdr_t);

		
		if (subhdr && subhdr_len) {
			memcpy(ptr, subhdr, subhdr_len);
			ptr += subhdr_len;
		}

		
		if (buf && len) {
			memcpy(ptr, buf, len);
		}

		ret = nla_put(skb, BCM_GENL_ATTR_MSG, tot_len, p);
		if (ret != 0) {
			WL_ERR(("nla_put_string failed\n"));
			goto out;
		}
	}

	if (mcast) {
		int err = 0;
		
		genlmsg_end(skb, msg);
		

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
		if ((err = genlmsg_multicast(skb, 0, wl_genl_mcast.id, GFP_ATOMIC)) < 0)
#else
		if ((err = genlmsg_multicast(&wl_genl_family, skb, 0, 0, GFP_ATOMIC)) < 0)
#endif
			WL_ERR(("genlmsg_multicast for attr(%d) failed. Error:%d \n",
				attr_type, err));
		else
			WL_DBG(("Multicast msg sent successfully. attr_type:%d len:%d \n",
				attr_type, tot_len));
	} else {
		NETLINK_CB(skb).dst_group = 0; 

		
		genlmsg_end(skb, msg);

		
		if (genlmsg_unicast(&init_net, skb, pid) < 0)
			WL_ERR(("genlmsg_unicast failed\n"));
	}

out:
	if (p)
		kfree(p);
	if (ret)
		nlmsg_free(skb);

	return ret;
}

static s32
wl_genl_handle_msg(
	struct sk_buff *skb,
	struct genl_info *info)
{
	struct nlattr *na;
	u8 *data = NULL;

	WL_DBG(("Enter \n"));

	if (info == NULL) {
		return -EINVAL;
	}

	na = info->attrs[BCM_GENL_ATTR_MSG];
	if (!na) {
		WL_ERR(("nlattribute NULL\n"));
		return -EINVAL;
	}

	data = (char *)nla_data(na);
	if (!data) {
		WL_ERR(("Invalid data\n"));
		return -EINVAL;
	} else {
		
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)) || defined(WL_COMPAT_WIRELESS)
		WL_DBG(("%s: Data received from pid (%d) \n", __func__,
			info->snd_pid));
#else
		WL_DBG(("%s: Data received from pid (%d) \n", __func__,
			info->snd_portid));
#endif 
	}

	return 0;
}
#endif 

#ifdef CUSTOMER_HW_ONE
static int wl_android_get_tx_fail(struct net_device *dev, char *command, int total_len)
{
	int bytes_written;
	wl_cnt_t cnt;
	int error = 0;
	uint32 curr_txframes = 0;
	uint32 curr_txretrans = 0;
	uint32 curr_txerror = 0;
	uint32 txframes_diff = 0;
	uint32 txretrans_diff = 0;
	uint32 txerror_diff = 0;
	uint32 diff_ratio = 0;
	uint32 total_cnt = 0;

	memset(&cnt, 0, sizeof(wl_cnt_t));
	strcpy((char *)&cnt, "counters");

	if ((error = wldev_ioctl(dev, WLC_GET_VAR, &cnt, sizeof(wl_cnt_t), 0)) < 0) {
		DHD_ERROR(("%s: get tx fail fail\n", __func__));
		last_txframes = 0xffffffff;
		last_txretrans = 0xffffffff;
		last_txerror = 0xffffffff;
		goto exit;
	}

	curr_txframes = cnt.txframe;
	curr_txretrans = cnt.txretrans;
	curr_txerror = cnt.txerror;

	if (last_txframes != 0xffffffff) {
		if ((curr_txframes >= last_txframes) && (curr_txretrans >= last_txretrans) &&
			(curr_txerror >= last_txerror)) {

			txframes_diff = curr_txframes - last_txframes;
			txretrans_diff = curr_txretrans - last_txretrans;
			txerror_diff = curr_txerror - last_txerror;
			total_cnt = txframes_diff + txretrans_diff + txerror_diff;

			if (total_cnt > TX_FAIL_CHECK_COUNT) {
				diff_ratio = ((txretrans_diff + txerror_diff)  * 100) / total_cnt;
			}
		}
	}
	last_txframes = curr_txframes;
	last_txretrans = curr_txretrans;
	last_txerror = curr_txerror;

exit:
	DHD_ERROR(("TXPER:%d, txframes: %d ,txretrans: %d, txerror: %d, total: %d\n",
		diff_ratio, txframes_diff, txretrans_diff, txerror_diff, total_cnt));
	bytes_written = snprintf(command, total_len, "%s %d",
		CMD_GET_TX_FAIL, diff_ratio);

	return bytes_written;
}

void wlan_lock_perf(void)
{
#ifdef CONFIG_PERFLOCK
	if (!is_perf_lock_active(wlan_perf_lock))
		perf_lock(wlan_perf_lock);
#endif

}

void wlan_unlock_perf(void)
{
#ifdef CONFIG_PERFLOCK
	if (is_perf_lock_active(wlan_perf_lock))
		perf_unlock(wlan_perf_lock);
#endif
}

static void wlan_init_perf(void)
{
#ifdef CONFIG_PERFLOCK
	wlan_perf_lock = perflock_acquire("bcmdhd");
	perf_lock_init(wlan_perf_lock, TYPE_PERF_LOCK, PERF_LOCK_HIGHEST, "bcmdhd");
#endif
}

static void wlan_deinit_perf(void)
{
#ifdef CONFIG_PERFLOCK
	if (is_perf_lock_active(wlan_perf_lock))
		perf_unlock(wlan_perf_lock);
	perflock_release("bcmdhd");
#endif
}

extern int wl_cfg80211_send_priv_event(struct net_device *dev, char *flag);

int multi_core_locked = 0;

void wlan_lock_multi_core(struct net_device *dev)
{
#ifdef WL_CFG80211
	dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
	dhd_info_t *dhd = DHD_DEV_INFO(dev);
	dhd_pub_t *dhdp = &(dhd->pub);
#endif
	char buf[32];

	sprintf(buf, "PERF_LOCK freq=high cpu=%u", (nr_cpu_ids <= 2)? nr_cpu_ids: 2);
	wl_cfg80211_send_priv_event(dev, buf);
	multi_core_locked = 1;
	if (dhdp) {
		dhd_sched_dpc(dhdp);
	} else {
		DHD_ERROR(("%s: dhdp is null", __func__));
	}
}

void wlan_unlock_multi_core(struct net_device *dev)
{
#ifdef WL_CFG80211
	dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
	dhd_info_t *dhd = DHD_DEV_INFO(dev);
	dhd_pub_t *dhdp = &(dhd->pub);
#endif

	multi_core_locked = 0;
	if (dhdp) {
		dhd_sched_dpc(dhdp);
	} else {
		DHD_ERROR(("%s: dhdp is null", __func__));
	}
	wl_cfg80211_send_priv_event(dev, "PERF_UNLOCK");
}

void wl_android_traffic_monitor(struct net_device *dev)
{
	unsigned long rx_packets_count = 0;
	unsigned long tx_packets_count = 0;
	unsigned long traffic_diff = 0;
	unsigned long jiffies_diff = 0;

	
	dhd_get_txrx_stats(dev, &rx_packets_count, &tx_packets_count);
	current_traffic_count = rx_packets_count + tx_packets_count;

	
	if ((current_traffic_count >= last_traffic_count &&
		jiffies > (last_traffic_count_jiffies + 2*HZ)) || screen_off || !sta_connected) {

		
		if (screen_off || !sta_connected) {
			DHD_ERROR(("set traffic = 0 and relase performace lock when %s",
				screen_off ? "screen off": "disconnected"));
			traffic_diff = 0;
		}
		else {
			
			jiffies_diff = jiffies - last_traffic_count_jiffies;

			if (jiffies_diff < 7*HZ) {
				traffic_diff = (current_traffic_count - last_traffic_count) /
					jiffies_diff * HZ;
			}
			else {
				traffic_diff = 0;
			}
		}
		switch (traffic_stats_flag) {
		case TRAFFIC_STATS_NORMAL:
			if (traffic_diff > TRAFFIC_HIGH_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_HIGH;
				wlan_lock_perf();
				DHD_ERROR(("lock cpu here, traffic-count=%ld\n", traffic_diff));
				if (traffic_diff > TRAFFIC_SUPER_HIGH_WATER_MARK) {
					traffic_stats_flag = TRAFFIC_STATS_SUPER_HIGH;
					wlan_lock_multi_core(dev);
					DHD_ERROR(("lock 2nd cpu here, traffic-count=%ld\n",
						traffic_diff));
				}
			}
			break;
		case TRAFFIC_STATS_HIGH:
			if (traffic_diff > TRAFFIC_SUPER_HIGH_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_SUPER_HIGH;
				wlan_lock_multi_core(dev);
				DHD_ERROR(("lock 2nd cpu here, traffic-count=%ld\n", traffic_diff));
			}
			else if (traffic_diff < TRAFFIC_LOW_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_NORMAL;
				wlan_unlock_perf();
				DHD_ERROR(("unlock cpu here, traffic-count=%ld\n", traffic_diff));
			}
			break;
		case TRAFFIC_STATS_SUPER_HIGH:
			if (traffic_diff < TRAFFIC_SUPER_HIGH_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_HIGH;
				wlan_unlock_multi_core(dev);
				DHD_ERROR(("unlock 2nd cpu here, traffic-count=%ld\n",
					traffic_diff));
				if (traffic_diff < TRAFFIC_LOW_WATER_MARK) {
					traffic_stats_flag = TRAFFIC_STATS_NORMAL;
					wlan_unlock_perf();
					DHD_ERROR(("unlock cpu here, traffic-count=%ld\n",
						traffic_diff));
				}
			}
			break;
		default:
			break;
		}
		last_traffic_count_jiffies = jiffies;
		last_traffic_count = current_traffic_count;
	}
	
}

#if defined(HTC_TX_TRACKING)
static int wl_android_set_tx_tracking(struct net_device *dev, char *command, int total_len)
{
	int bytes_written = 0;
	char iovbuf[32];
	
	uint tx_stat_chk = 0; 
	uint tx_stat_chk_prd = 5; 
	uint tx_stat_chk_ratio = 8; 
	uint tx_stat_chk_num = 5; 

	sscanf(command + strlen(CMD_TX_TRACKING) + 1, "%u %u %u %u",
		&tx_stat_chk, &tx_stat_chk_prd, &tx_stat_chk_ratio, &tx_stat_chk_num);
	DHD_ERROR(("wl_android_set_tx_tracking command=%s", command));

	if (tx_stat_chk_num != old_tx_stat_chk_num) {
		bcm_mkiovar("tx_stat_chk_num", (char *)&tx_stat_chk_num, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);
		old_tx_stat_chk_num = tx_stat_chk_num;
	} else
		DHD_INFO(("tx_stat_chk_num duplicate, ignore!\n"));

	if (tx_stat_chk != old_tx_stat_chk) {
		bcm_mkiovar("tx_stat_chk", (char *)&tx_stat_chk, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);
		old_tx_stat_chk = tx_stat_chk;
	} else
		DHD_INFO(("tx_stat_chk duplicate, ignore!\n"));

	if (tx_stat_chk_ratio != old_tx_stat_chk_ratio) {
		bcm_mkiovar("tx_stat_chk_ratio",
			(char *)&tx_stat_chk_ratio, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);
		old_tx_stat_chk_ratio = tx_stat_chk_ratio;
	} else
		DHD_INFO(("tx_stat_chk_ratio duplicate, ignore!\n"));

	if (tx_stat_chk_prd != old_tx_stat_chk_prd) {
		bcm_mkiovar("tx_stat_chk_prd", (char *)&tx_stat_chk_prd, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);
		old_tx_stat_chk_prd = tx_stat_chk_prd;
	} else
		DHD_INFO(("tx_stat_chk_prd duplicate, ignore!\n"));

	return bytes_written;
}
#endif 

static int wl_android_get_dtim_skip(struct net_device *dev, char *command, int total_len)
{
	char iovbuf[32];
	int bytes_written;
	int error = 0;

	memset(iovbuf, 0, sizeof(iovbuf));
	strcpy(iovbuf, "bcn_li_dtim");

	if ((error = wldev_ioctl(dev, WLC_GET_VAR, &iovbuf, sizeof(iovbuf), 0)) >= 0) {
		bytes_written = snprintf(command, total_len, "Dtim_skip %d", iovbuf[0]);
		DHD_INFO(("%s: get dtim_skip = %d\n", __FUNCTION__, iovbuf[0]));
		return bytes_written;
	}
	else  {
		DHD_ERROR(("%s: get dtim_skip failed code %d\n", __FUNCTION__, error));
		return -1;
	}
}

static int wl_android_set_dtim_skip(struct net_device *dev, char *command, int total_len)
{
	char iovbuf[32];
	int bytes_written;
	int error = -1;
	int bcn_li_dtim;

	bcn_li_dtim = htod32((uint)*(command + strlen(CMD_DTIM_SKIP_SET) + 1) - '0');

	if ((bcn_li_dtim >= 0) || ((bcn_li_dtim <= 5))) {
		memset(iovbuf, 0, sizeof(iovbuf));
		bcm_mkiovar("bcn_li_dtim", (char *)&bcn_li_dtim, 4, iovbuf, sizeof(iovbuf));

		if ((error = wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1)) >= 0) {
			bytes_written = snprintf(command, total_len, "Dtim_skip %d", iovbuf[0]);
			DHD_INFO(("%s: set dtim_skip = %d\n", __FUNCTION__, iovbuf[0]));
			return bytes_written;
		}
		else {
			DHD_ERROR(("%s: set dtim_skip failed code %d\n", __FUNCTION__, error));
		}
	}
	else {
		DHD_ERROR(("%s Incorrect dtim_skip setting %d, ignored\n",
			__FUNCTION__, bcn_li_dtim));
	}
	return -1;
}

static int wl_android_set_txpower(struct net_device *dev, char *command, int total_len)
{
	int bytes_written;
	int error = -1;
	int txpower = -1;

	txpower = bcm_atoi(command + strlen(CMD_TXPOWER_SET) + 1);

	if ((txpower >= 0) || ((txpower <= 127))) {
		txpower |= WL_TXPWR_OVERRIDE;
		txpower = htod32(txpower);
		if ((error = wldev_iovar_setint(dev, "qtxpower", txpower)) >= 0)  {
			bytes_written = snprintf(command, total_len, "OK");
			DHD_INFO(("%s: set TXpower 0x%X is OK\n", __FUNCTION__, txpower));
			return bytes_written;
		}
		else {
			DHD_ERROR(("%s: set tx power failed, txpower=%d\n", __FUNCTION__, txpower));
		}
	} else {
		DHD_ERROR(("%s: Unsupported tx power value, txpower=%d\n", __FUNCTION__, txpower));
	}

	bytes_written = snprintf(command, total_len, "FAIL");
	return -1;
}

static int wl_android_set_ap_txpower(struct net_device *dev, char *command, int total_len)
{
	int ap_txpower = 0, ap_txpower_orig = 0;
	char iovbuf[32];

	ap_txpower = bcm_atoi(command + strlen(CMD_AP_TXPOWER_SET) + 1);
	if (ap_txpower == 0) {
		
		memset(iovbuf, 0, sizeof(iovbuf));
		bcm_mkiovar("qtxpower", (char *)&ap_txpower_orig, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_GET_VAR, &iovbuf, sizeof(iovbuf), 0);
		DHD_ERROR(("default tx power is %d\n", ap_txpower_orig));
		ap_txpower_orig |= WL_TXPWR_OVERRIDE;
	}

	if (ap_txpower == 99) {
		
		ap_txpower = ap_txpower_orig;
	} else {
		ap_txpower |= WL_TXPWR_OVERRIDE;
	}

	DHD_ERROR(("set tx power to 0x%x\n", ap_txpower));
	bcm_mkiovar("qtxpower", (char *)&ap_txpower, 4, iovbuf, sizeof(iovbuf));
	wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);

	return 0;
}

static int wl_android_set_scan_minrssi(struct net_device *dev, char *command, int total_len)
{
	int minrssi = 0;
	int err = 0;
	int bytes_written;

	DHD_TRACE(("%s\n", __FUNCTION__));

	minrssi = bcm_strtoul((char *)(command + strlen("SCAN_MINRSSI") + 1), NULL, 10);

	err = wldev_iovar_setint(dev, "scanresults_minrssi", minrssi);

	if (err) {
		DHD_ERROR(("set scan_minrssi fail!\n"));
		bytes_written = snprintf(command, total_len, "FAIL");
	} else
		bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

#ifdef WLC_E_RSSI_LOW
static int wl_android_low_rssi_set(struct net_device *dev, char *command, int total_len)
{
	int low_rssi_trigger;
	int low_rssi_duration;
	int trigger_offset;
	int duration_offset;
	char tran_buf[16] = {0};
	char *pp;
	int err = 0;
	int bytes_written;

	DHD_TRACE(("%s\n", __FUNCTION__));

	trigger_offset = strcspn(command, " ");
	pp = command + trigger_offset + 1;
	duration_offset = strcspn(pp, " ");

	memcpy(tran_buf, pp, duration_offset);
	low_rssi_trigger = bcm_strtoul(tran_buf, NULL, 10);
	err |= wldev_iovar_setint(dev, "low_rssi_trigger", low_rssi_trigger);

	memset(tran_buf, 0, 16);
	pp = command + trigger_offset + duration_offset + 2;
	memcpy(tran_buf, pp, strlen(command) - (trigger_offset+duration_offset+1));
	low_rssi_duration = bcm_strtoul(tran_buf, NULL, 10);
	err |= wldev_iovar_setint(dev, "low_rssi_duration", low_rssi_duration);

	DHD_TRACE(("set low rssi trigger %d, duration %d\n", low_rssi_trigger, low_rssi_duration));

	if (err) {
		DHD_ERROR(("set low rssi ind fail!\n"));
		bytes_written = snprintf(command, total_len, "FAIL");
	} else
		bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}
#endif 

int wl_android_black_list_match(char *mac)
{
	int i;

	if (android_ap_macmode) {
		for (i = 0; i < android_ap_black_list.count; i++) {
			if (!bcmp(mac, &android_ap_black_list.ea[i],
				sizeof(struct ether_addr))) {
				DHD_ERROR(("mac in black list, ignore it\n"));
				break;
			}
		}

		if (i == android_ap_black_list.count)
			return 1;
	}

	return 0;
}

static int
wl_android_get_assoc_sta_list(struct net_device *dev, char *buf, int len)
{
	struct maclist *maclist = (struct maclist *) buf;
	int ret, i;
	char mac_lst[256];
	char *p_mac_str;

	bcm_mdelay(500);
	maclist->count = 10;
	ret = wldev_ioctl(dev, WLC_GET_ASSOCLIST, buf, len, 0);

	memset(mac_lst, 0, sizeof(mac_lst));
	p_mac_str = mac_lst;


	p_mac_str += snprintf(p_mac_str, 80, "%d|", maclist->count);

	for (i = 0; i < maclist->count; i++) {
		struct ether_addr *id = &maclist->ea[i];


		p_mac_str += snprintf(p_mac_str, 80, "%02X:%02X:%02X:%02X:%02X:%02X,",
			id->octet[0], id->octet[1], id->octet[2],
			id->octet[3], id->octet[4], id->octet[5]);
	}

	if (ret != 0) {
		DHD_ERROR(("get assoc count fail\n"));
		maclist->count = 0;
	} else
		DHD_ERROR(("get assoc count %d, len %d\n", maclist->count, len));

#ifdef CUSTOMER_HW_ONE
	if (!sta_event_sent && assoc_count_buff && (assoc_count_buff != maclist->count)) {
		wl_cfg80211_send_priv_event(dev, "STA_LEAVE");
	}

	assoc_count_buff = maclist->count;
	sta_event_sent = 0;
#endif
	memset(buf, 0x0, len);
	memcpy(buf, mac_lst, len);
	return len;
}

static int wl_android_set_ap_mac_list(struct net_device *dev, void *buf)
{
	struct mac_list_set *mac_list_set = (struct mac_list_set *)buf;
	struct maclist *white_maclist = (struct maclist *)&mac_list_set->white_list;
	struct maclist *black_maclist = (struct maclist *)&mac_list_set->black_list;
	int mac_mode = mac_list_set->mode;
	uint length;
	uint i;
	DHD_ERROR(("%s in\n", __func__));


	if (white_maclist->count > 16 || black_maclist->count > 16) {
		DHD_TRACE(("invalid count white: %d black: %d\n",
			white_maclist->count, black_maclist->count));
		return 0;
	}

	if (buf != (char *)&android_mac_list_buf) {
		DHD_TRACE(("Backup the mac list\n"));
		memcpy((char *)&android_mac_list_buf, buf, sizeof(struct mac_list_set));
	} else
		DHD_TRACE(("recover, don't back up mac list\n"));

	android_ap_macmode = mac_mode;
	if (mac_mode == MACLIST_MODE_DISABLED) {

		bzero(&android_ap_black_list, sizeof(struct mflist));

		wldev_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode), 1);
	} else {
		scb_val_t scbval;
		char mac_buf[256] = {0};
		struct maclist *assoc_maclist = (struct maclist *) mac_buf;

		mac_mode = MACLIST_MODE_ALLOW;

		wldev_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode), 1);

		
		bcopy(white_maclist, &android_ap_white_list, sizeof(android_ap_white_list));

		length = sizeof(white_maclist->count)+white_maclist->count*ETHER_ADDR_LEN;
		wldev_ioctl(dev, WLC_SET_MACLIST, white_maclist, length, 1);
		DHD_ERROR(("White List, length %d:\n", length));
		for (i = 0; (i < android_ap_white_list.count) && (i < 16); i++)
			DHD_ERROR(("wmac %d: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
				android_ap_white_list.ea[i].octet[0],
				android_ap_white_list.ea[i].octet[1],
				android_ap_white_list.ea[i].octet[2],
				android_ap_white_list.ea[i].octet[3],
				android_ap_white_list.ea[i].octet[4],
				android_ap_white_list.ea[i].octet[5]));

		
		bcopy(black_maclist, &android_ap_black_list, sizeof(android_ap_black_list));

		DHD_ERROR(("Black List, size %d:\n", (int)sizeof(android_ap_black_list)));
		for (i = 0; (i < android_ap_black_list.count) && (i < 16); i++)
			DHD_ERROR(("bmac %d: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
				android_ap_black_list.ea[i].octet[0],
				android_ap_black_list.ea[i].octet[1],
				android_ap_black_list.ea[i].octet[2],
				android_ap_black_list.ea[i].octet[3],
				android_ap_black_list.ea[i].octet[4],
				android_ap_black_list.ea[i].octet[5]));

		
		assoc_maclist->count = 8;
		wldev_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist, 256, 0);
		if (assoc_maclist->count) {
			uint j;
			for (i = 0; (i < assoc_maclist->count) && (i < 8); i++) {
				for (j = 0; (j < android_ap_white_list.count) && (j < 16); j++) {
					if (!bcmp(&assoc_maclist->ea[i],
						&android_ap_white_list.ea[j], ETHER_ADDR_LEN)) {
						DHD_TRACE(("match allow, let it be\n"));
						break;
					}
				}
				if (j == android_ap_white_list.count) {
					DHD_TRACE(("match black, deauth it\n"));
					scbval.val = htod32(1);
					bcopy(&assoc_maclist->ea[i], &scbval.ea, ETHER_ADDR_LEN);
					wldev_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
						sizeof(scb_val_t), 1);
				}
			}
		}
	}
	return 0;
}

#ifdef WL_CFG80211
static int wl_android_get_mac_addr(struct net_device *ndev, char *command, int total_len)
{
	int ret;
	int bytes_written = 0;
	struct ether_addr id;

	ret = wl_cfg80211_get_mac_addr(ndev, &id);
	if (ret)
		return 0;
	bytes_written = snprintf(command, total_len, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
		id.octet[0], id.octet[1], id.octet[2],
		id.octet[3], id.octet[4], id.octet[5]);
	return bytes_written;
}
#endif

static void wl_android_act_time_expire(void)
{
	struct timer_list **timer;
	timer = &wl_android_active_timer;

	if (*timer) {
		WL_TRACE(("ac timer expired\n"));
		del_timer_sync(*timer);
		kfree(*timer);
		*timer = NULL;
		if (screen_off)
			return;
		wl_android_active_expired = 1;
	}
	return;
}

static void wl_android_deactive(void)
{
	struct timer_list **timer;
	timer = &wl_android_active_timer;

	if (*timer) {
		WL_TRACE(("previous ac exist\n"));
		del_timer_sync(*timer);
		kfree(*timer);
		*timer = NULL;
	}
	wl_android_active_expired = 0;
	WL_TRACE(("wl_android_deactive\n"));
	return;
}

void wl_android_set_active_level(int level)
{
	active_level = level;
	DHD_ERROR(("set active level to %d\n", active_level));
	return;
}

void wl_android_set_active_period(int period)
{
	active_period = period;
	DHD_ERROR(("set active_period to %d\n", active_period));
	return;
}

int wl_android_get_active_level(void)
{
	return active_level;
}

int wl_android_get_active_period(void)
{
	return active_period;
}

void wl_android_set_screen_off(int off)
{
	screen_off = off;
	DHD_ERROR(("wl_android_set_screen_off %d\n", screen_off));
	if (screen_off)
		wl_android_deactive();

	return;
}

static int wl_android_set_power_mode(struct net_device *dev, char *command, int total_len)
{
	int bytes_written;
	int pm_val;
	int force_pm = 3;
	int mpc = 0;

	pm_val = bcm_atoi(command + strlen(CMD_POWER_MODE_SET) + 1);

		switch (pm_val) {
		
		case 0:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_ANDROID_NORMAL);
			dhdhtc_update_wifi_power_mode(screen_off);
			mpc = 1;
			wldev_iovar_setint(dev, "mpc", mpc);
			wl_cfg80211_set_btcoex_done(dev);
			break;
		case 1:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_ANDROID_NORMAL);
			dhdhtc_update_wifi_power_mode(screen_off);
			mpc = 0;
			wldev_iovar_setint(dev, "mpc", mpc);
			wldev_ioctl(dev, WLC_SET_PM, &force_pm, sizeof(force_pm), 1);
			break;
		
		case 10:
			wl_android_deactive();
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_BROWSER_LOAD_PAGE);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 11:
			if (!screen_off) {
				struct timer_list **timer;
				timer = &wl_android_active_timer;

				if (*timer) {
					mod_timer(*timer, jiffies + active_period * HZ / 1000);
					
				} else {
					*timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
					if (*timer) {
						(*timer)->function =
							(void *)wl_android_act_time_expire;
						init_timer(*timer);
						(*timer)->expires =
							jiffies + active_period * HZ / 1000;
						add_timer(*timer);
						
					}
				}
				dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_BROWSER_LOAD_PAGE);
				dhdhtc_update_wifi_power_mode(screen_off);
			}
			break;
		 
		case 20:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_USER_CONFIG);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 21:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_USER_CONFIG);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		 
		case 30:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_FOTA_DOWNLOADING);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 31:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_FOTA_DOWNLOADING);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		
		case 40:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_KDDI_APK);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 41:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_KDDI_APK);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;

		case 87: 
			DHD_ERROR(("wifilock release\n"));
			dhdcdc_wifiLock = 0;
			dhdhtc_update_wifi_power_mode(screen_off);
			dhdhtc_update_dtim_listen_interval(screen_off);
			break;

		case 88: 
			DHD_ERROR(("wifilock accquire\n"));
			dhdcdc_wifiLock = 1;
			dhdhtc_update_wifi_power_mode(screen_off);
			dhdhtc_update_dtim_listen_interval(screen_off);
			break;

		case 99: 
			dhdcdc_power_active_while_plugin = !dhdcdc_power_active_while_plugin;
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		default:
			DHD_ERROR(("%s: not support mode: %d\n", __func__, pm_val));
			break;

	}

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;

}

static int wl_android_get_power_mode(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int pm_local = PM_FAST;
	int bytes_written;

	error = wldev_ioctl(dev, WLC_GET_PM, &pm_local, sizeof(pm_local), 0);
	if (!error) {
		DHD_TRACE(("%s: Powermode = %d\n", __func__, pm_local));
		if (pm_local == PM_OFF)
			pm_local = 1;
		else
			pm_local = 0;
		bytes_written = snprintf(command, total_len, "powermode = %d\n", pm_local);

	} else {
		DHD_TRACE(("%s: Error = %d\n", __func__, error));
		bytes_written = snprintf(command, total_len, "FAIL\n");
	}
	return bytes_written;
}

static int wl_android_get_wifilock(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;

	bytes_written = snprintf(command, total_len, "%d", dhdcdc_wifiLock);
	DHD_ERROR(("dhdcdc_wifiLock: %s\n", command));

	return bytes_written;
}

int wl_android_is_during_wifi_call(void)
{
	return wl_android_wifi_call;
}

static int wl_android_set_wificall(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;
	char *s;
	int set_val;

	mutex_lock(&wl_wificall_mutex);
	s =  command + strlen("WIFICALL") + 1;
	set_val = bcm_atoi(s);



	switch (set_val) {
	case 0:
		if (wl_android_is_during_wifi_call() == 0) {
			DHD_ERROR(("wifi call is in disconnected state!\n"));
			break;
		}

		DHD_ERROR(("wifi call ends: %d\n", set_val));
		wl_android_wifi_call = 0;

		dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_WIFI_PHONE);
		dhdhtc_update_wifi_power_mode(screen_off);

		dhdhtc_update_dtim_listen_interval(screen_off);

		break;
	case 1:
		if (wl_android_is_during_wifi_call() == 1) {
			DHD_ERROR(("wifi call is already in running state!\n"));
			break;
		}

		DHD_ERROR(("wifi call comes: %d\n", set_val));
		wl_android_wifi_call = 1;

		dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_WIFI_PHONE);
		dhdhtc_update_wifi_power_mode(screen_off);

		dhdhtc_update_dtim_listen_interval(screen_off);

		break;
	default:
		DHD_ERROR(("%s: not support mode: %d\n", __func__, set_val));
		break;

	}

	bytes_written = snprintf(command, total_len, "OK");
	mutex_unlock(&wl_wificall_mutex);

	return bytes_written;
}

int dhd_set_project(char * project, int project_len)
{

	if ((project_len < 1) || (project_len > 32)) {
		DHD_ERROR(("Invaild project name length!\n"));
		return -1;
	}

	strncpy(project_type, project, project_len);
	DHD_INFO(("%s: set project type: %s\n", __FUNCTION__, project_type));

	return 0;
}

static int wl_android_set_project(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;
	char project[33];
	int project_offset;
	int project_size;

	DHD_TRACE(("%s\n", __FUNCTION__));
	memset(project, 0, sizeof(project));

	project_offset = strcspn(command, " ");
	project_size = strlen(command) - project_offset;

	if (project_offset == 0) {
		DHD_ERROR(("%s, no project specified\n", __FUNCTION__));
		return 0;
	}

	if (project_size > 32) {
		DHD_ERROR(("%s: project name is too long: %s\n",
			__FUNCTION__, (char *)command + project_offset + 1));
		return 0;
	}

	strncpy(project, command + project_offset + 1, MIN(project_size, sizeof(project)));

	DHD_INFO(("%s: set project: %s\n", __FUNCTION__, project));
	dhd_set_project(project, project_size);

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

static int wl_android_gateway_add(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;

	int i;
	DHD_TRACE(("Driver GET GATEWAY-ADD CMD!!!\n"));
	sscanf(command+12, "%d", &i);
	sprintf(wl_abdroid_gatewaybuf, "%02x%02x%02x%02x",
		i & 0xff, ((i >> 8) & 0xff), ((i >> 16) & 0xff), ((i >> 24) & 0xff));

	if (strcmp(wl_abdroid_gatewaybuf, "00000000") == 0)
		sprintf(wl_abdroid_gatewaybuf, "FFFFFFFF");

	DHD_TRACE(("gatewaybuf: %s", wl_abdroid_gatewaybuf));

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

static int wl_android_auto_channel(struct net_device *dev, char *command, int total_len)
{
	int chosen = 0;
	char req_buf[64] = {0};
	wl_uint32_list_t *request = (wl_uint32_list_t *)req_buf;
	int rescan = 0;
	int retry = 0;
	int updown = 0;
	wlc_ssid_t null_ssid;
	int res = 0;
	int spec = 0;
	int start_channel = 1, end_channel = 14;
	int i = 0;
	int channel = 0;
	int isup = 0;
	int bytes_written = 0;
	int apsta_var = 0;
	
	int band = WLC_BAND_2G;
	

	DHD_TRACE(("Enter %s\n", __func__));

	channel = bcm_atoi(command);

	wldev_ioctl(dev, WLC_GET_UP, &isup, sizeof(isup), 0);

	res = wldev_ioctl(dev, WLC_DOWN, &updown, sizeof(updown), 1);
	if (res) {
		DHD_ERROR(("%s fail to set updown\n", __func__));
		goto fail;
	}

	apsta_var = 0;
	res = wldev_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var), 1);
	if (res) {
		DHD_ERROR(("%s fail to set apsta_var 0\n", __func__));
		goto fail;
	}
	apsta_var = 1;
	res = wldev_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var), 1);
	if (res) {
		DHD_ERROR(("%s fail to set apsta_var 1\n", __func__));
		goto fail;
	}
	res = wldev_ioctl(dev, WLC_GET_AP, &apsta_var, sizeof(apsta_var), 0);

	updown = 1;
	res = wldev_ioctl(dev, WLC_UP, &updown, sizeof(updown), 1);
	if (res < 0) {
		DHD_ERROR(("%s fail to set apsta \n", __func__));
		goto fail;
	}

auto_channel_retry:
	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	
	#if 1 
	null_ssid.SSID_len = strlen("");
	strncpy(null_ssid.SSID, "", null_ssid.SSID_len);
	#else
	null_ssid.SSID_len = strlen("test");
	strncpy(null_ssid.SSID, "test", null_ssid.SSID_len);
	#endif
	

	res |= wldev_ioctl(dev, WLC_SET_SPECT_MANAGMENT, &spec, sizeof(spec), 1);
	res |= wldev_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid), 1);
	
	res |= wldev_ioctl(dev, WLC_SET_BAND, &band, sizeof(band), 1);
	
	res |= wldev_ioctl(dev, WLC_UP, &updown, sizeof(updown), 1);

	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	res |= wldev_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid), 1);

	request->count = htod32(0);
	if (channel >> 8) {
		start_channel = (channel >> 8) & 0xff;
		end_channel = channel & 0xff;
		request->count = end_channel - start_channel + 1;
		DHD_ERROR(("request channel: %d to %d ,request->count =%d\n",
			start_channel, end_channel, request->count));
		for (i = 0; i < request->count; i++) {
			request->element[i] = CH20MHZ_CHSPEC((start_channel + i));
			
			DHD_ERROR(("request.element[%d]=0x%x\n", i, request->element[i]));
		}
	}

	res = wldev_ioctl(dev, WLC_START_CHANNEL_SEL, request, sizeof(req_buf), 1);
	if (res < 0) {
		DHD_ERROR(("can't start auto channel\n"));
		chosen = 6;
		goto fail;
	}

get_channel_retry:
		bcm_mdelay(500);

	res = wldev_ioctl(dev, WLC_GET_CHANNEL_SEL, &chosen, sizeof(chosen), 0);

	
	chosen &= 0x00FF;

	if (res < 0 || dtoh32(chosen) == 0) {
		if (retry++ < 6)
			goto get_channel_retry;
		else {
			DHD_ERROR(("can't get auto channel sel, err = %d, "
						"chosen = %d\n", res, chosen));
			chosen = 6; 
		}
	}

	if ((chosen == start_channel) && (!rescan++)) {
		retry = 0;
		goto auto_channel_retry;
	}

	if (channel == 0) {
		channel = chosen;
		last_auto_channel = chosen;
	} else {
		DHD_ERROR(("channel range from %d to %d, chosen = %d\n",
			start_channel, end_channel, chosen));

		if (chosen > end_channel) {
			if (chosen <= 6)
				chosen = end_channel;
			else
				chosen = start_channel;
		} else if (chosen < start_channel)
			chosen = start_channel;

		channel = chosen;
	}

	res = wldev_ioctl(dev, WLC_DOWN, &updown, sizeof(updown), 1);
	if (res < 0) {
		DHD_ERROR(("%s fail to set up err =%d\n", __func__, res));
		goto fail;
	}
	band = WLC_BAND_AUTO;
	res |= wldev_ioctl(dev, WLC_SET_BAND, &band, sizeof(band), 1);

fail :

	bytes_written = snprintf(command, total_len, "%d", channel);
	return bytes_written;

}

static int wl_android_allow_p2p_event(struct net_device *dev, char *command, int total_len)
{
	int bytes_written = 0;
	int ret = 0;
	char *s;
	int set_val;
#ifdef WL_CFG80211
	dhd_pub_t *dhdp = (dhd_pub_t *)wl_cfg80211_get_dhdp();
#else
	dhd_info_t *dhd = DHD_DEV_INFO(dev);
	dhd_pub_t *dhdp = &(dhd->pub);
#endif 
	char iovbuf[WL_EVENTING_MASK_LEN+32];
	char eventmask[WL_EVENTING_MASK_LEN];

	s =  command + strlen(CMD_ALLOW_P2P_EVENT) + 1;
	set_val = bcm_atoi(s);

	DHD_ERROR(("%s: allow p2p event %d\n", __FUNCTION__, set_val));
	switch (set_val) {
	case 0:
		dhdp->allow_p2p_event = set_val;
		if (screen_off && !dhdp->suspend_disable_flag) {
			DHD_ERROR(("%s: screen is off, clear event mask\n", __FUNCTION__));
			memset(eventmask, 0, sizeof(eventmask));
			ret = wldev_iovar_getbuf(dev, "event_msgs",
				NULL, 0, eventmask, sizeof(eventmask), NULL);
			if (ret) {
				WL_ERR(("%s: wldev_iovar_getbuf() failed, ret=%d\n",
					__FUNCTION__, ret));
				goto error;
			}

			clrbit(eventmask, WLC_E_ACTION_FRAME_RX);
			
			
			
			clrbit(eventmask, WLC_E_P2P_DISC_LISTEN_COMPLETE);

			ret = wldev_iovar_setbuf(dev, "event_msgs",
				eventmask, sizeof(eventmask), iovbuf, sizeof(iovbuf), NULL);
			if (ret) {
				WL_ERR(("%s: wldev_iovar_setbuf() failed, ret=%d\n",
					__FUNCTION__, ret));
				goto error;
			}
		} else {
			DHD_ERROR(("%s: screen is on or suspend disabled, do nothing\n",
				__FUNCTION__));
		}
		break;
	case 1:
		dhdp->allow_p2p_event = set_val;
		DHD_ERROR(("%s: set event mask\n", __FUNCTION__));
		memset(eventmask, 0, sizeof(eventmask));
		ret = wldev_iovar_getbuf(dev, "event_msgs",
			NULL, 0, eventmask, sizeof(eventmask), NULL);
		if (ret) {
			WL_ERR(("%s: wldev_iovar_getbuf() failed, ret=%d\n",
				__FUNCTION__, ret));
			goto error;
		}

		setbit(eventmask, WLC_E_ACTION_FRAME_RX);
		
		
		
		setbit(eventmask, WLC_E_P2P_DISC_LISTEN_COMPLETE);

		ret = wldev_iovar_setbuf(dev, "event_msgs",
			eventmask, sizeof(eventmask), iovbuf, sizeof(iovbuf), NULL);
		if (ret) {
			WL_ERR(("%s: wldev_iovar_setbuf() failed, ret=%d\n",
				__FUNCTION__, ret));
			goto error;
		}
		break;
	default:
		DHD_ERROR(("%s: not support mode: %d\n", __FUNCTION__, set_val));
		break;

	}
	bytes_written = snprintf(command, total_len, "OK");
error:
	return ((ret == 0) ? bytes_written : ret);
}
#endif 
