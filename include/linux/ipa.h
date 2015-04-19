/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IPA_H_
#define _IPA_H_

#include <linux/msm_ipa.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/msm-sps.h>
#include <linux/if_ether.h>

#define IPA_APPS_MAX_BW_IN_MBPS 200

enum ipa_nat_en_type {
	IPA_BYPASS_NAT,
	IPA_SRC_NAT,
	IPA_DST_NAT,
};

enum ipa_mode_type {
	IPA_BASIC,
	IPA_ENABLE_FRAMING_HDLC,
	IPA_ENABLE_DEFRAMING_HDLC,
	IPA_DMA,
};

enum ipa_aggr_en_type {
	IPA_BYPASS_AGGR,
	IPA_ENABLE_AGGR,
	IPA_ENABLE_DEAGGR,
};

enum ipa_aggr_type {
	IPA_MBIM_16 = 0,
	IPA_HDLC    = 1,
	IPA_TLP     = 2,
	IPA_RNDIS   = 3,
	IPA_GENERIC = 4,
	IPA_QCMAP   = 6,
};

enum ipa_aggr_mode {
	IPA_MBIM,
	IPA_QCNCM,
};

enum ipa_dp_evt_type {
	IPA_RECEIVE,
	IPA_WRITE_DONE,
};

enum hdr_total_len_or_pad_type {
	IPA_HDR_PAD = 0,
	IPA_HDR_TOTAL_LEN = 1,
};

struct ipa_ep_cfg_nat {
	enum ipa_nat_en_type nat_en;
};

struct ipa_ep_cfg_hdr {
	u32  hdr_len;
	u32  hdr_ofst_metadata_valid;
	u32  hdr_ofst_metadata;
	u32  hdr_additional_const_len;
	u32  hdr_ofst_pkt_size_valid;
	u32  hdr_ofst_pkt_size;
	u32  hdr_a5_mux;
	u32  hdr_remove_additional;
	u32  hdr_metadata_reg_valid;
};

struct ipa_ep_cfg_hdr_ext {
	u32 hdr_pad_to_alignment;
	u32 hdr_total_len_or_pad_offset;
	bool hdr_payload_len_inc_padding;
	enum hdr_total_len_or_pad_type hdr_total_len_or_pad;
	bool hdr_total_len_or_pad_valid;
	bool hdr_little_endian;
};

struct ipa_ep_cfg_mode {
	enum ipa_mode_type mode;
	enum ipa_client_type dst;
};

struct ipa_ep_cfg_aggr {
	enum ipa_aggr_en_type aggr_en;
	enum ipa_aggr_type aggr;
	u32 aggr_byte_limit;
	u32 aggr_time_limit;
	u32 aggr_pkt_limit;
};

struct ipa_ep_cfg_route {
	u32 rt_tbl_hdl;
};

struct ipa_ep_cfg_holb {
	u16 en;
	u32 tmr_val;
};

struct ipa_ep_cfg_deaggr {
	u32 deaggr_hdr_len;
	bool packet_offset_valid;
	u32 packet_offset_location;
	u32 max_packet_len;
};

enum ipa_cs_offload {
	IPA_DISABLE_CS_OFFLOAD,
	IPA_ENABLE_CS_OFFLOAD_UL,
	IPA_ENABLE_CS_OFFLOAD_DL,
	IPA_CS_RSVD
};

struct ipa_ep_cfg_cfg {
	bool frag_offload_en;
	enum ipa_cs_offload cs_offload_en;
	u8 cs_metadata_hdr_offset;
};

/**
 * struct ipa_ep_cfg_metadata_mask - Endpoint initialization hdr metadata mask
 * @metadata_mask: Mask specifying which metadata bits to write to
 *	IPA_ENDP_INIT_HDR_n.s HDR_OFST_METADATA. Only
 *	masked metadata bits (set to 1) will be written. Valid for Output
 *	Pipes only (IPA Producer)
 */
struct ipa_ep_cfg_metadata_mask {
	u32 metadata_mask;
};

struct ipa_ep_cfg_metadata {
	u32 qmap_id;
};

struct ipa_ep_cfg {
	struct ipa_ep_cfg_nat nat;
	struct ipa_ep_cfg_hdr hdr;
	struct ipa_ep_cfg_hdr_ext hdr_ext;
	struct ipa_ep_cfg_mode mode;
	struct ipa_ep_cfg_aggr aggr;
	struct ipa_ep_cfg_deaggr deaggr;
	struct ipa_ep_cfg_route route;
	struct ipa_ep_cfg_cfg cfg;
	struct ipa_ep_cfg_metadata_mask metadata_mask;
	struct ipa_ep_cfg_metadata meta;
};

struct ipa_ep_cfg_ctrl {
	bool ipa_ep_suspend;
	bool ipa_ep_delay;
};

#define IPA_NUM_OF_FIFO_DESC(x) (x/sizeof(struct sps_iovec))
typedef void (*ipa_notify_cb)(void *priv, enum ipa_dp_evt_type evt,
		       unsigned long data);

struct ipa_connect_params {
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_client_type client;
	unsigned long client_bam_hdl;
	u32 client_ep_idx;
	void *priv;
	ipa_notify_cb notify;
	u32 desc_fifo_sz;
	u32 data_fifo_sz;
	bool pipe_mem_preferred;
	struct sps_mem_buffer desc;
	struct sps_mem_buffer data;
	bool skip_ep_cfg;
	bool keep_ipa_awake;
};

struct ipa_sps_params {
	unsigned long ipa_bam_hdl;
	u32 ipa_ep_idx;
	struct sps_mem_buffer desc;
	struct sps_mem_buffer data;
};

struct ipa_tx_intf {
	u32 num_props;
	struct ipa_ioc_tx_intf_prop *prop;
};

struct ipa_rx_intf {
	u32 num_props;
	struct ipa_ioc_rx_intf_prop *prop;
};

struct ipa_ext_intf {
	bool excp_pipe_valid;
	enum ipa_client_type excp_pipe;
	u32 num_props;
	struct ipa_ioc_ext_intf_prop *prop;
};

struct ipa_sys_connect_params {
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_client_type client;
	u32 desc_fifo_sz;
	void *priv;
	ipa_notify_cb notify;
	bool skip_ep_cfg;
	bool keep_ipa_awake;
};

struct ipa_tx_meta {
	u8 mbim_stream_id;
	bool mbim_stream_id_valid;
	u8 pkt_init_dst_ep;
	bool pkt_init_dst_ep_valid;
	bool pkt_init_dst_ep_remote;
	dma_addr_t dma_address;
	bool dma_address_valid;
};

typedef void (*ipa_msg_free_fn)(void *buff, u32 len, u32 type);

typedef int (*ipa_msg_pull_fn)(void *buff, u32 len, u32 type);

enum ipa_voltage_level {
	IPA_VOLTAGE_UNSPECIFIED,
	IPA_VOLTAGE_SVS = IPA_VOLTAGE_UNSPECIFIED,
	IPA_VOLTAGE_NOMINAL,
	IPA_VOLTAGE_MAX,
};

enum ipa_rm_event {
	IPA_RM_RESOURCE_GRANTED,
	IPA_RM_RESOURCE_RELEASED
};

typedef void (*ipa_rm_notify_cb)(void *user_data,
		enum ipa_rm_event event,
		unsigned long data);
struct ipa_rm_register_params {
	void *user_data;
	ipa_rm_notify_cb notify_cb;
};

struct ipa_rm_create_params {
	enum ipa_rm_resource_name name;
	enum ipa_voltage_level floor_voltage;
	struct ipa_rm_register_params reg_params;
	int (*request_resource)(void);
	int (*release_resource)(void);
};

struct ipa_rm_perf_profile {
	u32 max_supported_bandwidth_mbps;
};

#define A2_MUX_HDR_NAME_V4_PREF "dmux_hdr_v4_"
#define A2_MUX_HDR_NAME_V6_PREF "dmux_hdr_v6_"

enum teth_tethering_mode {
	TETH_TETHERING_MODE_RMNET,
	TETH_TETHERING_MODE_MBIM,
	TETH_TETHERING_MODE_MAX,
};

struct teth_bridge_init_params {
	ipa_notify_cb usb_notify_cb;
	void *private_data;
	enum ipa_client_type client;
	bool skip_ep_cfg;
};

struct teth_bridge_connect_params {
	u32 ipa_usb_pipe_hdl;
	u32 usb_ipa_pipe_hdl;
	enum teth_tethering_mode tethering_mode;
	enum ipa_client_type client_type;
};

struct ipa_tx_data_desc {
	struct list_head link;
	void *priv;
	void *pyld_buffer;
	u16  pyld_len;
};

struct ipa_rx_data {
	struct sk_buff *skb;
	dma_addr_t dma_addr;
};

enum ipa_irq_type {
	IPA_BAD_SNOC_ACCESS_IRQ = 0,
	IPA_EOT_COAL_IRQ,
	IPA_UC_IRQ_0,
	IPA_UC_IRQ_1,
	IPA_UC_IRQ_2,
	IPA_UC_IRQ_3,
	IPA_UC_IN_Q_NOT_EMPTY_IRQ,
	IPA_UC_RX_CMD_Q_NOT_FULL_IRQ,
	IPA_UC_TX_CMD_Q_NOT_FULL_IRQ,
	IPA_UC_TO_PROC_ACK_Q_NOT_FULL_IRQ,
	IPA_PROC_TO_UC_ACK_Q_NOT_EMPTY_IRQ,
	IPA_RX_ERR_IRQ,
	IPA_DEAGGR_ERR_IRQ,
	IPA_TX_ERR_IRQ,
	IPA_STEP_MODE_IRQ,
	IPA_PROC_ERR_IRQ,
	IPA_TX_SUSPEND_IRQ = 16,
	IPA_TX_HOLB_DROP_IRQ = 17,

	IPA_IRQ_MAX
};

struct ipa_tx_suspend_irq_data {
	u32 endpoints;
};


typedef void (*ipa_irq_handler_t)(enum ipa_irq_type interrupt,
				void *private_data,
				void *interrupt_data);

struct IpaHwBamStats_t {
	u32 bamFifoFull;
	u32 bamFifoEmpty;
	u32 bamFifoUsageHigh;
	u32 bamFifoUsageLow;
} __packed;

struct IpaHwRingStats_t {
	u32 ringFull;
	u32 ringEmpty;
	u32 ringUsageHigh;
	u32 ringUsageLow;
} __packed;

struct IpaHwStatsWDIRxInfoData_t {
	u32 max_outstanding_pkts;
	u32 num_pkts_processed;
	u32 rx_ring_rp_value;
	struct IpaHwRingStats_t rx_ind_ring_stats;
	struct IpaHwBamStats_t bam_stats;
	u32 num_bam_int_handled;
	u32 num_db;
	u32 num_unexpected_db;
	u32 reserved1;
	u32 reserved2;
} __packed;

/**
 * struct IpaHwStatsWDITxInfoData_t  - Structure holding the WDI Tx channel
 * structures
 *
 * @num_pkts_processed : Number of packets processed - cumulative
 * @copy_engine_doorbell_value : latest value of doorbell written to copy engine
 * @num_db_fired : Number of DB from uC FW to Copy engine
 * @tx_comp_ring_stats : ring info
 * @bam_stats : BAM info
 * @num_db : Number of times the doorbell was rung
 * @num_unexpected_db : Number of unexpected doorbells
 * @num_bam_int_handled : Number of Bam Interrupts handled by FW
 * @num_bam_int_in_non_runnning_state : Number of Bam interrupts while not in
 * Running state
 * @num_qmb_int_handled : Number of QMB interrupts handled
*/
struct IpaHwStatsWDITxInfoData_t {
	u32 num_pkts_processed;
	u32 copy_engine_doorbell_value;
	u32 num_db_fired;
	struct IpaHwRingStats_t tx_comp_ring_stats;
	struct IpaHwBamStats_t bam_stats;
	u32 num_db;
	u32 num_unexpected_db;
	u32 num_bam_int_handled;
	u32 num_bam_int_in_non_runnning_state;
	u32 num_qmb_int_handled;
} __packed;

struct IpaHwStatsWDIInfoData_t {
	struct IpaHwStatsWDIRxInfoData_t rx_ch_stats;
	struct IpaHwStatsWDITxInfoData_t tx_ch_stats;
} __packed;


struct ipa_wdi_ul_params {
	phys_addr_t rdy_ring_base_pa;
	u32 rdy_ring_size;
	phys_addr_t rdy_ring_rp_pa;
};

struct ipa_wdi_dl_params {
	phys_addr_t comp_ring_base_pa;
	u32 comp_ring_size;
	phys_addr_t ce_ring_base_pa;
	phys_addr_t ce_door_bell_pa;
	u32 ce_ring_size;
	u32 num_tx_buffers;
};

struct ipa_wdi_in_params {
	struct ipa_sys_connect_params sys;
	union {
		struct ipa_wdi_ul_params ul;
		struct ipa_wdi_dl_params dl;
	} u;
};

struct ipa_wdi_out_params {
	phys_addr_t uc_door_bell_pa;
	u32 clnt_hdl;
};

struct odu_bridge_params {
	const char *netdev_name;
	void *priv;
	ipa_notify_cb tx_dp_notify;
	int (*send_dl_skb)(void *priv, struct sk_buff *skb);
	u8 device_ethaddr[ETH_ALEN];
	u32 ipa_desc_size;
};

#ifdef CONFIG_IPA

int ipa_connect(const struct ipa_connect_params *in, struct ipa_sps_params *sps,
		u32 *clnt_hdl);
int ipa_disconnect(u32 clnt_hdl);

int ipa_reset_endpoint(u32 clnt_hdl);

int ipa_cfg_ep(u32 clnt_hdl, const struct ipa_ep_cfg *ipa_ep_cfg);

int ipa_cfg_ep_nat(u32 clnt_hdl, const struct ipa_ep_cfg_nat *ipa_ep_cfg);

int ipa_cfg_ep_hdr(u32 clnt_hdl, const struct ipa_ep_cfg_hdr *ipa_ep_cfg);

int ipa_cfg_ep_hdr_ext(u32 clnt_hdl,
			const struct ipa_ep_cfg_hdr_ext *ipa_ep_cfg);

int ipa_cfg_ep_mode(u32 clnt_hdl, const struct ipa_ep_cfg_mode *ipa_ep_cfg);

int ipa_cfg_ep_aggr(u32 clnt_hdl, const struct ipa_ep_cfg_aggr *ipa_ep_cfg);

int ipa_cfg_ep_deaggr(u32 clnt_hdl,
		      const struct ipa_ep_cfg_deaggr *ipa_ep_cfg);

int ipa_cfg_ep_route(u32 clnt_hdl, const struct ipa_ep_cfg_route *ipa_ep_cfg);

int ipa_cfg_ep_holb(u32 clnt_hdl, const struct ipa_ep_cfg_holb *ipa_ep_cfg);

int ipa_cfg_ep_cfg(u32 clnt_hdl, const struct ipa_ep_cfg_cfg *ipa_ep_cfg);

int ipa_cfg_ep_metadata_mask(u32 clnt_hdl, const struct ipa_ep_cfg_metadata_mask
		*ipa_ep_cfg);

int ipa_cfg_ep_holb_by_client(enum ipa_client_type client,
				const struct ipa_ep_cfg_holb *ipa_ep_cfg);

int ipa_cfg_ep_ctrl(u32 clnt_hdl, const struct ipa_ep_cfg_ctrl *ep_ctrl);

int ipa_add_hdr(struct ipa_ioc_add_hdr *hdrs);

int ipa_del_hdr(struct ipa_ioc_del_hdr *hdls);

int ipa_commit_hdr(void);

int ipa_reset_hdr(void);

int ipa_get_hdr(struct ipa_ioc_get_hdr *lookup);

int ipa_put_hdr(u32 hdr_hdl);

int ipa_copy_hdr(struct ipa_ioc_copy_hdr *copy);

int ipa_add_hdr_proc_ctx(struct ipa_ioc_add_hdr_proc_ctx *proc_ctxs);

int ipa_del_hdr_proc_ctx(struct ipa_ioc_del_hdr_proc_ctx *hdls);

int ipa_add_rt_rule(struct ipa_ioc_add_rt_rule *rules);

int ipa_del_rt_rule(struct ipa_ioc_del_rt_rule *hdls);

int ipa_commit_rt(enum ipa_ip_type ip);

int ipa_reset_rt(enum ipa_ip_type ip);

int ipa_get_rt_tbl(struct ipa_ioc_get_rt_tbl *lookup);

int ipa_put_rt_tbl(u32 rt_tbl_hdl);

int ipa_query_rt_index(struct ipa_ioc_get_rt_tbl_indx *in);

int ipa_mdfy_rt_rule(struct ipa_ioc_mdfy_rt_rule *rules);

int ipa_add_flt_rule(struct ipa_ioc_add_flt_rule *rules);

int ipa_del_flt_rule(struct ipa_ioc_del_flt_rule *hdls);

int ipa_mdfy_flt_rule(struct ipa_ioc_mdfy_flt_rule *rules);

int ipa_commit_flt(enum ipa_ip_type ip);

int ipa_reset_flt(enum ipa_ip_type ip);

int allocate_nat_device(struct ipa_ioc_nat_alloc_mem *mem);

int ipa_nat_init_cmd(struct ipa_ioc_v4_nat_init *init);

int ipa_nat_dma_cmd(struct ipa_ioc_nat_dma_cmd *dma);

int ipa_nat_del_cmd(struct ipa_ioc_v4_nat_del *del);

int ipa_send_msg(struct ipa_msg_meta *meta, void *buff,
		  ipa_msg_free_fn callback);
int ipa_register_pull_msg(struct ipa_msg_meta *meta, ipa_msg_pull_fn callback);
int ipa_deregister_pull_msg(struct ipa_msg_meta *meta);

int ipa_register_intf(const char *name, const struct ipa_tx_intf *tx,
		       const struct ipa_rx_intf *rx);
int ipa_register_intf_ext(const char *name, const struct ipa_tx_intf *tx,
		       const struct ipa_rx_intf *rx,
		       const struct ipa_ext_intf *ext);
int ipa_deregister_intf(const char *name);

int ipa_set_aggr_mode(enum ipa_aggr_mode mode);

int ipa_set_qcncm_ndp_sig(char sig[3]);

int ipa_set_single_ndp_per_mbim(bool enable);

int ipa_tx_dp(enum ipa_client_type dst, struct sk_buff *skb,
		struct ipa_tx_meta *metadata);

int ipa_tx_dp_mul(enum ipa_client_type dst,
			struct ipa_tx_data_desc *data_desc);

void ipa_free_skb(struct ipa_rx_data *);

int ipa_setup_sys_pipe(struct ipa_sys_connect_params *sys_in, u32 *clnt_hdl);

int ipa_teardown_sys_pipe(u32 clnt_hdl);

int ipa_connect_wdi_pipe(struct ipa_wdi_in_params *in,
		struct ipa_wdi_out_params *out);
int ipa_disconnect_wdi_pipe(u32 clnt_hdl);
int ipa_enable_wdi_pipe(u32 clnt_hdl);
int ipa_disable_wdi_pipe(u32 clnt_hdl);
int ipa_resume_wdi_pipe(u32 clnt_hdl);
int ipa_suspend_wdi_pipe(u32 clnt_hdl);
int ipa_get_wdi_stats(struct IpaHwStatsWDIInfoData_t *stats);
u16 ipa_get_smem_restr_bytes(void);

int ipa_rm_create_resource(struct ipa_rm_create_params *create_params);

int ipa_rm_delete_resource(enum ipa_rm_resource_name resource_name);

int ipa_rm_register(enum ipa_rm_resource_name resource_name,
			struct ipa_rm_register_params *reg_params);

int ipa_rm_deregister(enum ipa_rm_resource_name resource_name,
			struct ipa_rm_register_params *reg_params);

int ipa_rm_set_perf_profile(enum ipa_rm_resource_name resource_name,
			struct ipa_rm_perf_profile *profile);

int ipa_rm_add_dependency(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name);

int ipa_rm_delete_dependency(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name);

int ipa_rm_request_resource(enum ipa_rm_resource_name resource_name);

int ipa_rm_release_resource(enum ipa_rm_resource_name resource_name);

int ipa_rm_notify_completion(enum ipa_rm_event event,
		enum ipa_rm_resource_name resource_name);

int ipa_rm_inactivity_timer_init(enum ipa_rm_resource_name resource_name,
				 unsigned long msecs);

int ipa_rm_inactivity_timer_destroy(enum ipa_rm_resource_name resource_name);

int ipa_rm_inactivity_timer_request_resource(
				enum ipa_rm_resource_name resource_name);

int ipa_rm_inactivity_timer_release_resource(
				enum ipa_rm_resource_name resource_name);

int teth_bridge_init(struct teth_bridge_init_params *params);

int teth_bridge_disconnect(enum ipa_client_type client);

int teth_bridge_connect(struct teth_bridge_connect_params *connect_params);


int odu_bridge_init(struct odu_bridge_params *params);

int odu_bridge_connect(void);

int odu_bridge_disconnect(void);

int odu_bridge_tx_dp(struct sk_buff *skb, struct ipa_tx_meta *metadata);

int odu_bridge_cleanup(void);


int ipa_write_qmap_id(struct ipa_ioc_write_qmapid *param_in);

int ipa_add_interrupt_handler(enum ipa_irq_type interrupt,
		ipa_irq_handler_t handler,
		bool deferred_flag,
		void *private_data);

int ipa_remove_interrupt_handler(enum ipa_irq_type interrupt);

void ipa_bam_reg_dump(void);

bool ipa_emb_ul_pipes_empty(void);

int ipa_get_ep_mapping(enum ipa_client_type client);

bool ipa_is_ready(void);

void ipa_proxy_clk_vote(void);
void ipa_proxy_clk_unvote(void);

enum ipa_hw_type ipa_get_hw_type(void);

bool ipa_is_client_handle_valid(u32 clnt_hdl);

enum ipa_client_type ipa_get_client_mapping(int pipe_idx);

enum ipa_rm_resource_name ipa_get_rm_resource_from_ep(int pipe_idx);

#else 

static inline int ipa_connect(const struct ipa_connect_params *in,
		struct ipa_sps_params *sps,	u32 *clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_disconnect(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_reset_endpoint(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_cfg_ep(u32 clnt_hdl,
		const struct ipa_ep_cfg *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_nat(u32 clnt_hdl,
		const struct ipa_ep_cfg_nat *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_hdr(u32 clnt_hdl,
		const struct ipa_ep_cfg_hdr *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_hdr_ext(u32 clnt_hdl,
		const struct ipa_ep_cfg_hdr_ext *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_mode(u32 clnt_hdl,
		const struct ipa_ep_cfg_mode *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_aggr(u32 clnt_hdl,
		const struct ipa_ep_cfg_aggr *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_deaggr(u32 clnt_hdl,
		const struct ipa_ep_cfg_deaggr *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_route(u32 clnt_hdl,
		const struct ipa_ep_cfg_route *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_holb(u32 clnt_hdl,
		const struct ipa_ep_cfg_holb *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_cfg(u32 clnt_hdl,
		const struct ipa_ep_cfg_cfg *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_metadata_mask(u32 clnt_hdl,
		const struct ipa_ep_cfg_metadata_mask *ipa_ep_cfg)
{
	return -EPERM;
}

static inline int ipa_cfg_ep_ctrl(u32 clnt_hdl,
			const struct ipa_ep_cfg_ctrl *ep_ctrl)
{
	return -EPERM;
}

static inline int ipa_add_hdr(struct ipa_ioc_add_hdr *hdrs)
{
	return -EPERM;
}

static inline int ipa_del_hdr(struct ipa_ioc_del_hdr *hdls)
{
	return -EPERM;
}

static inline int ipa_commit_hdr(void)
{
	return -EPERM;
}

static inline int ipa_reset_hdr(void)
{
	return -EPERM;
}

static inline int ipa_get_hdr(struct ipa_ioc_get_hdr *lookup)
{
	return -EPERM;
}

static inline int ipa_put_hdr(u32 hdr_hdl)
{
	return -EPERM;
}

static inline int ipa_copy_hdr(struct ipa_ioc_copy_hdr *copy)
{
	return -EPERM;
}

static inline int ipa_add_hdr_proc_ctx(
				struct ipa_ioc_add_hdr_proc_ctx *proc_ctxs)
{
	return -EPERM;
}

static inline int ipa_del_hdr_proc_ctx(struct ipa_ioc_del_hdr_proc_ctx *hdls)
{
	return -EPERM;
}
static inline int ipa_add_rt_rule(struct ipa_ioc_add_rt_rule *rules)
{
	return -EPERM;
}

static inline int ipa_del_rt_rule(struct ipa_ioc_del_rt_rule *hdls)
{
	return -EPERM;
}

static inline int ipa_commit_rt(enum ipa_ip_type ip)
{
	return -EPERM;
}

static inline int ipa_reset_rt(enum ipa_ip_type ip)
{
	return -EPERM;
}

static inline int ipa_get_rt_tbl(struct ipa_ioc_get_rt_tbl *lookup)
{
	return -EPERM;
}

static inline int ipa_put_rt_tbl(u32 rt_tbl_hdl)
{
	return -EPERM;
}

static inline int ipa_query_rt_index(struct ipa_ioc_get_rt_tbl_indx *in)
{
	return -EPERM;
}

static inline int ipa_mdfy_rt_rule(struct ipa_ioc_mdfy_rt_rule *rules)
{
	return -EPERM;
}

static inline int ipa_add_flt_rule(struct ipa_ioc_add_flt_rule *rules)
{
	return -EPERM;
}

static inline int ipa_del_flt_rule(struct ipa_ioc_del_flt_rule *hdls)
{
	return -EPERM;
}

static inline int ipa_mdfy_flt_rule(struct ipa_ioc_mdfy_flt_rule *rules)
{
	return -EPERM;
}

static inline int ipa_commit_flt(enum ipa_ip_type ip)
{
	return -EPERM;
}

static inline int ipa_reset_flt(enum ipa_ip_type ip)
{
	return -EPERM;
}

static inline int allocate_nat_device(struct ipa_ioc_nat_alloc_mem *mem)
{
	return -EPERM;
}


static inline int ipa_nat_init_cmd(struct ipa_ioc_v4_nat_init *init)
{
	return -EPERM;
}


static inline int ipa_nat_dma_cmd(struct ipa_ioc_nat_dma_cmd *dma)
{
	return -EPERM;
}


static inline int ipa_nat_del_cmd(struct ipa_ioc_v4_nat_del *del)
{
	return -EPERM;
}

static inline int ipa_send_msg(struct ipa_msg_meta *meta, void *buff,
		ipa_msg_free_fn callback)
{
	return -EPERM;
}

static inline int ipa_register_pull_msg(struct ipa_msg_meta *meta,
		ipa_msg_pull_fn callback)
{
	return -EPERM;
}

static inline int ipa_deregister_pull_msg(struct ipa_msg_meta *meta)
{
	return -EPERM;
}

static inline int ipa_register_intf(const char *name,
				     const struct ipa_tx_intf *tx,
				     const struct ipa_rx_intf *rx)
{
	return -EPERM;
}

static inline int ipa_register_intf_ext(const char *name,
		const struct ipa_tx_intf *tx,
		const struct ipa_rx_intf *rx,
		const struct ipa_ext_intf *ext)
{
	return -EPERM;
}

static inline int ipa_deregister_intf(const char *name)
{
	return -EPERM;
}

static inline int ipa_set_aggr_mode(enum ipa_aggr_mode mode)
{
	return -EPERM;
}

static inline int ipa_set_qcncm_ndp_sig(char sig[3])
{
	return -EPERM;
}

static inline int ipa_set_single_ndp_per_mbim(bool enable)
{
	return -EPERM;
}

static inline int ipa_tx_dp(enum ipa_client_type dst, struct sk_buff *skb,
		struct ipa_tx_meta *metadata)
{
	return -EPERM;
}

static inline int ipa_tx_dp_mul(
	enum ipa_client_type dst,
	struct ipa_tx_data_desc *data_desc)
{
	return -EPERM;
}

static inline void ipa_free_skb(struct ipa_rx_data *rx_in)
{
	return;
}

static inline u16 ipa_get_smem_restr_bytes(void)
{
	return -EPERM;
}

static inline int ipa_setup_sys_pipe(struct ipa_sys_connect_params *sys_in,
		u32 *clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_teardown_sys_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_connect_wdi_pipe(struct ipa_wdi_in_params *in,
		struct ipa_wdi_out_params *out)
{
	return -EPERM;
}

static inline int ipa_disconnect_wdi_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_enable_wdi_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_disable_wdi_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_resume_wdi_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_suspend_wdi_pipe(u32 clnt_hdl)
{
	return -EPERM;
}

static inline int ipa_rm_create_resource(
		struct ipa_rm_create_params *create_params)
{
	return -EPERM;
}

static inline int ipa_rm_delete_resource(
		enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_register(enum ipa_rm_resource_name resource_name,
			struct ipa_rm_register_params *reg_params)
{
	return -EPERM;
}

static inline int ipa_rm_set_perf_profile(
		enum ipa_rm_resource_name resource_name,
		struct ipa_rm_perf_profile *profile)
{
	return -EPERM;
}

static inline int ipa_rm_deregister(enum ipa_rm_resource_name resource_name,
			struct ipa_rm_register_params *reg_params)
{
	return -EPERM;
}

static inline int ipa_rm_add_dependency(
		enum ipa_rm_resource_name resource_name,
		enum ipa_rm_resource_name depends_on_name)
{
	return -EPERM;
}

static inline int ipa_rm_delete_dependency(
		enum ipa_rm_resource_name resource_name,
		enum ipa_rm_resource_name depends_on_name)
{
	return -EPERM;
}

static inline int ipa_rm_request_resource(
		enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_release_resource(
		enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_notify_completion(enum ipa_rm_event event,
		enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_inactivity_timer_init(
		enum ipa_rm_resource_name resource_name,
			unsigned long msecs)
{
	return -EPERM;
}

static inline int ipa_rm_inactivity_timer_destroy(
		enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_inactivity_timer_request_resource(
				enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int ipa_rm_inactivity_timer_release_resource(
				enum ipa_rm_resource_name resource_name)
{
	return -EPERM;
}

static inline int teth_bridge_init(struct teth_bridge_init_params *params)
{
	return -EPERM;
}

static inline int teth_bridge_disconnect(enum ipa_client_type client)
{
	return -EPERM;
}

static inline int teth_bridge_connect(struct teth_bridge_connect_params
				      *connect_params)
{
	return -EPERM;
}

static inline int odu_bridge_init(struct odu_bridge_params *params)
{
	return -EPERM;
}

static inline int odu_bridge_disconnect(void)
{
	return -EPERM;
}

static inline int odu_bridge_connect(void)
{
	return -EPERM;
}

static inline int odu_bridge_tx_dp(struct sk_buff *skb,
						struct ipa_tx_meta *metadata)
{
	return -EPERM;
}

static inline int odu_bridge_cleanup(void)
{
	return -EPERM;
}


static inline int ipa_write_qmap_id(struct ipa_ioc_write_qmapid *param_in)
{
	return -EPERM;
}

static inline int ipa_add_interrupt_handler(enum ipa_irq_type interrupt,
		ipa_irq_handler_t handler,
		bool deferred_flag,
		void *private_data)
{
	return -EPERM;
}

static inline int ipa_remove_interrupt_handler(enum ipa_irq_type interrupt)
{
	return -EPERM;
}

static inline void ipa_bam_reg_dump(void)
{
	return;
}

static inline bool ipa_emb_ul_pipes_empty(void)
{
	return false;
}

static inline int ipa_get_wdi_stats(struct IpaHwStatsWDIInfoData_t *stats)
{
	return -EPERM;
}

static inline int ipa_get_ep_mapping(enum ipa_client_type client)
{
	return -EPERM;
}

static inline bool ipa_is_ready(void)
{
	return false;
}

static inline void ipa_proxy_clk_vote(void)
{
}

static inline void ipa_proxy_clk_unvote(void)
{
}

static inline enum ipa_hw_type ipa_get_hw_type(void)
{
	return IPA_HW_None;
}

static inline bool ipa_is_client_handle_valid(u32 clnt_hdl)
{
	return -EINVAL;
}

static inline enum ipa_client_type ipa_get_client_mapping(int pipe_idx)
{
	return -EINVAL;
}

static inline enum ipa_rm_resource_name ipa_get_rm_resource_from_ep(
	int pipe_idx)
{
	return -EFAULT;
}

#endif 

#endif 
