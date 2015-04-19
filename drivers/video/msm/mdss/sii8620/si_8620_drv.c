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

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>

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

#define SLEEP_10MS 10

#ifdef USE_HW_TIMER
static int int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status);
#endif
static int int_2_isr(struct drv_hw_context *hw_context, uint8_t int_2_status);
static int int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status);
static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status);
static int int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status);
static int int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status);
static int int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status);
static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t intr_status);
static int hdcp2_isr(struct drv_hw_context *hw_context, uint8_t intr_status);
static int g2wb_err_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static int g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int);
static int mhl_cbus_err_isr(struct drv_hw_context *hw_context,
			    uint8_t cbus_err_int);
static int mhl3_block_isr(struct drv_hw_context *hw_context, uint8_t status);
static int coc_isr(struct drv_hw_context *hw_context, uint8_t coc_int_status);
static int tdm_isr(struct drv_hw_context *hw_context, uint8_t intr_status);
static int int_link_trn_isr(struct drv_hw_context *hw_context,
			    uint8_t intr_status);
static void enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num,
			uint8_t intr_mask);
static void switch_to_d3(struct drv_hw_context *hw_context,
			 bool do_interrupt_clear);
static void disconnect_mhl(struct drv_hw_context *hw_context,
			   bool do_interrupt_clear);
static void start_hdcp(struct drv_hw_context *hw_context);
static void start_hdcp_content_type(struct drv_hw_context *hw_context);
static void stop_video(struct drv_hw_context *hw_context);
static int get_device_rev(struct drv_hw_context *hw_context);
static void unmute_video(struct drv_hw_context *hw_context);
static int set_hdmi_params(struct mhl_dev_context *dev_context);
static void hsic_init(struct drv_hw_context *hw_context);

static void si_mhl_tx_drv_enable_emsc_block(struct drv_hw_context *hw_context);
static void disable_gen2_write_burst_rcv(struct drv_hw_context *hw_context);
static void disable_gen2_write_burst_xmit(struct drv_hw_context *hw_context);
static void si_mhl_tx_drv_start_gen2_write_burst(
	struct drv_hw_context *hw_context);

static void si_mhl_tx_drv_set_lowest_tmds_link_speed(
	struct mhl_dev_context *dev_context, uint32_t pixel_clock_frequency,
	uint8_t bits_per_pixel);

static int start_video(struct drv_hw_context *hw_context);

#define FIELD_RATE_MEASUREMENT_INTERVAL 50
#define LOCAL_BLK_RCV_BUFFER_SIZE 288

#define DDC_ABORT_THRESHOLD	10
static int ddc_abort_count;

#define MSC_ABORT_THRESHOLD	10
static int msc_abort_count;

struct intr_tbl {
	uint8_t aggr_stat_index;
	uint8_t aggr_stat_id_bit;
	uint8_t mask;
	uint16_t mask_addr;
	uint16_t stat_addr;
	int (*isr) (struct drv_hw_context *, uint8_t status);
	char *name;
};

enum l1_intr_stat_enums_t {
	FAST_INTR_STAT,
	L1_INTR_STAT_0,
	L1_INTR_STAT_1,
	L1_INTR_STAT_2,
	L1_INTR_STAT_3,
	L1_INTR_STAT_4,
	L1_INTR_STAT_5,
	
	NUM_AGGREGATED_INTR_REGS
};

struct intr_tbl g_intr_tbl[] = {
	{L1_INTR_STAT_2, 0x10, 0, REG_CBUS_DISC_INTR0_MASK,
	 REG_CBUS_DISC_INTR0, int_4_isr, "DISC"},
	{L1_INTR_STAT_1, 0x08, 0, REG_MDT_INT_1_MASK,
	 REG_MDT_INT_1, g2wb_err_isr, "G2WB"},
	{L1_INTR_STAT_1, 0x04, 0, REG_MDT_INT_0_MASK,
	 REG_MDT_INT_0, g2wb_isr, "G2WB"},
	{L1_INTR_STAT_5, 0x08, 0, REG_COC_INTR_MASK, REG_COC_INTR,
	 coc_isr, "COC"},
	{L1_INTR_STAT_4, 0x04, 0, REG_TRXINTMH, REG_TRXINTH,
	 tdm_isr, "TDM"},
	{L1_INTR_STAT_1, 0x01, 0, REG_CBUS_INT_0_MASK,
	 REG_CBUS_INT_0, mhl_cbus_isr, "MSC"},
	{L1_INTR_STAT_1, 0x02, 0, REG_CBUS_INT_1_MASK,
	 REG_CBUS_INT_1, mhl_cbus_err_isr, "MERR"},
	{L1_INTR_STAT_2, 0x40, 0, REG_EMSCINTRMASK, REG_EMSCINTR,
	 mhl3_block_isr, "BLOCK"},
	{L1_INTR_STAT_2, 0x80, 0, REG_EMSCINTRMASK1,
		REG_EMSCINTR1, int_link_trn_isr, "LTRN"},
	{L1_INTR_STAT_0, 0x20, 0, REG_INTR8_MASK, REG_INTR8,
	 int_8_isr, "INFR"},
	{L1_INTR_STAT_0, 0x80, 0, REG_TPI_INTR_EN,
	 REG_TPI_INTR_ST0, hdcp_isr, "HDCP"},
	{L1_INTR_STAT_3, 0x01, 0, REG_HDCP2X_INTR0_MASK,
	 REG_HDCP2X_INTR0, hdcp2_isr, "HDCP2"},
	{L1_INTR_STAT_0, 0x40, 0, REG_INTR9_MASK, REG_INTR9,
	 int_9_isr, "EDID"},
	{L1_INTR_STAT_0, 0x04, 0, REG_INTR3_MASK, REG_INTR3,
	 int_3_isr, "DDC"},
	{L1_INTR_STAT_0, 0x08, 0, REG_INTR5_MASK, REG_INTR5,
	 int_5_isr, "SCDT"},
	{L1_INTR_STAT_0, 0x02, 0, REG_INTR2_MASK, REG_INTR2,
	 int_2_isr, "INT2"},
#ifdef USE_HW_TIMER
	{L1_INTR_STAT_0, 0x01, 0, REG_INTR1_MASK, REG_INTR1,
	 int_1_isr, "TIMR"},
#endif
};

enum intr_nums_t {
	INTR_DISC,
	INTR_G2WB_ERR,
	INTR_G2WB,
	INTR_COC,
	INTR_TDM,
	INTR_MSC,
	INTR_MERR,
	INTR_BLOCK,
	INTR_LINK_TRAINING,
	INTR_INFR,
	INTR_HDCP,
	INTR_HDCP2,
	INTR_EDID,
	INTR_DDC,
	INTR_SCDT,
	INTR_USER,
#ifdef USE_HW_TIMER
	INTR_TIMR,
#endif
	MAX_INTR
};

#define BIT_RGND_READY_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT6
#define BIT_CBUS_MHL12_DISCON_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT5
#define BIT_CBUS_MHL3_DISCON_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT4
#define BIT_NOT_MHL_EST_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT3
#define BIT_MHL_EST_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT2
#define BIT_MHL3_EST_INT \
		BIT_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT1

#define BIT_RGND_READY_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK6
#define BIT_CBUS_MHL12_DISCON_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK5
#define BIT_CBUS_MHL3_DISCON_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK4
#define BIT_NOT_MHL_EST_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK3
#define BIT_MHL_EST_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK2
#define BIT_MHL3_EST_INT_MASK \
		BIT_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK1

#ifdef USE_HW_TIMER
#define BIT_HW_TIMER_POP		BIT_INTR1_STAT7
#endif

#define BIT_DDC_CMD_DONE	BIT_INTR3_STAT3

#define BIT_INTR_SCDT_CHANGE		BIT_INTR5_STAT0

#define BIT_CEA_NEW_VSI		BIT_INTR8_MASK2
#define BIT_CEA_NEW_AVI		BIT_INTR8_MASK1

#define BIT_VID_OVRRD_ENABLE_AUTO_LINK_MODE_UPDATE 0x08

#define BIT_MDT_RFIFO_DATA_RDY		BIT_MDT_INT_0_MDT_INT_0_0
#define BIT_MDT_IDLE_AFTER_HAWB_DISABLE	BIT_MDT_INT_0_MDT_INT_0_2
#define BIT_MDT_XFIFO_EMPTY		BIT_MDT_INT_0_MDT_INT_0_3

#define BIT_MDT_RCV_TIMEOUT		BIT_MDT_INT_1_MDT_INT_1_0
#define BIT_MDT_RCV_SM_ABORT_PKT_RCVD	BIT_MDT_INT_1_MDT_INT_1_1
#define BIT_MDT_RCV_SM_ERROR		BIT_MDT_INT_1_MDT_INT_1_2

#define BIT_MDT_XMIT_TIMEOUT		BIT_MDT_INT_1_MDT_INT_1_5
#define BIT_MDT_XMIT_SM_ABORT_PKT_RCVD	BIT_MDT_INT_1_MDT_INT_1_6
#define BIT_MDT_XMIT_SM_ERROR		BIT_MDT_INT_1_MDT_INT_1_7

#define BIT_CBUS_DDC_PEER_ABORT	BIT_DDC_ABORT_INT_DDC_ABORT_INT_STAT7

#define BIT_CBUS_MSC_MT_DONE_NACK \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK7
#define BIT_CBUS_MSC_MR_SET_INT \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK6
#define BIT_CBUS_MSC_MR_WRITE_BURST \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK5
#define BIT_CBUS_MSC_MR_MSC_MSG \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK4
#define BIT_CBUS_MSC_MR_WRITE_STAT \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK3
#define BIT_CBUS_HPD_CHG \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK2
#define BIT_CBUS_MSC_MT_DONE \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK1
#define BIT_CBUS_CNX_CHG \
		BIT_CBUS_INT_0_MASK_CBUS_INT_0_MASK0

#define BIT_CBUS_CMD_ABORT	BIT_CBUS_INT_1_MASK_CBUS_INT_1_MASK6
#define BIT_CBUS_MSC_ABORT_RCVD	BIT_CBUS_INT_1_MASK_CBUS_INT_1_MASK3
#define BIT_CBUS_DDC_ABORT	BIT_CBUS_INT_1_MASK_CBUS_INT_1_MASK2
#define BIT_CBUS_CEC_ABORT	BIT_CBUS_INT_1_MASK_CBUS_INT_1_MASK1

#define BIT_INTR9_EDID_ERROR		BIT_INTR9_STAT6
#define BIT_INTR9_EDID_DONE		BIT_INTR9_STAT5
#define BIT_INTR9_DEVCAP_DONE		BIT_INTR9_STAT4
#define BIT_INTR9_EDID_ERROR_MASK	BIT_INTR9_MASK6
#define BIT_INTR9_EDID_DONE_MASK	BIT_INTR9_MASK5
#define BIT_INTR9_DEVCAP_DONE_MASK	BIT_INTR9_MASK4

#define BIT_TDM_INTR_SYNC_DATA		BIT_TRXINTH_TRX_INTR8
#define BIT_TDM_INTR_SYNC_WAIT		BIT_TRXINTH_TRX_INTR9

#define BIT_TDM_INTR_SYNC_DATA_MASK	BIT_TRXINTMH_TRX_INTRMASK8
#define BIT_TDM_INTR_SYNC_WAIT_MASK	BIT_TRXINTMH_TRX_INTRMASK9

#define BIT_HDCP2_INTR_AUTH_DONE \
		BIT_HDCP2X_INTR0_HDCP2X_INTR0_STAT0
#define BIT_HDCP2_INTR_AUTH_FAIL \
		BIT_HDCP2X_INTR0_HDCP2X_INTR0_STAT1
#define BIT_HDCP2_INTR_RPTR_RCVID_CHANGE \
		BIT_HDCP2X_INTR0_HDCP2X_INTR0_STAT4

#define BIT_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR	0x04
#define BIT_TMDS_CSTAT_P3_AVIF_MANUAL_CLEAR_STROBE	0x08

#define BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT7
#define BIT_PAGE_CBUS_REG_MSC_MT_ABORT_INT_STAT5 \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT5
#define BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT3
#define BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT2
#define BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT1
#define BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL \
		BIT_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT0

#define	REG_CBUS_MHL_SCRPAD_BASE		0x40

#define REG_RX_HDMI_CTRL0_DEFVAL_DVI		0x14
#define REG_RX_HDMI_CTRL0_DEFVAL_HDMI		0x1C
#define REG_RX_HDMI_CTRL0_DEFVAL_HW_CTRL	0x10

#define	REG_RX_HDMI_CTRL2_DEFVAL_DVI		0x30
#define REG_RX_HDMI_CTRL2_DEFVAL_HDMI		0x38

#define TX_HW_RESET_PERIOD			10
#define TX_HW_RESET_DELAY			100

#define MHL_FLOW_FLAG_NONE			0x0000
#define MHL_FLOW_FLAG_MHL_IMPEDANCE		0x0001
#define MHL_FLOW_FLAG_MHL_ESTABLISHED		0x0002
#define MHL_FLOW_FLAG_DCAP_READY		0x0004
#define MHL_FLOW_FLAG_DCAP_CHANGE		0x0008
#define MHL_FLOW_FLAG_PATH_ENABLE		0x0010
#define MHL_FLOW_FLAG_RAP_CONTENT_ON		0x0020
#define MHL_FLOW_FLAG_CBUS_SET_HPD		0x0040
#define MHL_FLOW_FLAG_EDID_RTP			0x0080
#define MHL_FLOW_FLAG_INPUT_CLOCK_STABLE	0x0100
#define MHL_FLOW_FLAG_INFOFRAME_RECEIVED	0x0200
#define MHL_FLOW_FLAG_CP_AVAILABLE		0x0400
#define MHL_FLOW_FLAG_CP_AUTHENTICATED		0x0800

#define BIT_COC_PLL_LOCK_STATUS_CHANGE		0x01
#define BIT_COC_CALIBRATION_DONE		0x02

#define MSK_TDM_SYNCHRONIZED			0xC0
#define VAL_TDM_SYNCHRONIZED			0x80

#define BITS_COC_STAT_F_FSM_STATE_MASK		0x0F
#define BITS_COC_STAT_0_CALIBRATION_MASK	0x8F
#define BITS_COC_STAT_0_CALIBRATION_STATE_2	0x02
#define BITS_COC_STAT_0_PLL_LOCKED		0x80

#define VAL_M3_CTRL_MHL3_VALUE (BIT_M3_CTRL_SW_MHL3_SEL \
			| BIT_M3_CTRL_M3AV_EN \
			| BIT_M3_CTRL_ENC_TMDS \
			| BIT_M3_CTRL_MHL3_MASTER_EN)

#define VAL_M3_CTRL_PEER_VERSION_PENDING_VALUE \
	VAL_M3_CTRL_MHL3_VALUE

#define VAL_M3_CTRL_MHL1_2_VALUE (BIT_M3_CTRL_SW_MHL3_SEL | \
	BIT_M3_CTRL_ENC_TMDS)

#define BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_PP	0x08
#define BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_NORM	0x00

static char *rgnd_value_string[] = {
	"open",
	"2k",
	"1k",
	"short"
};

#define IN_MHL3_MODE(hw_context) \
	(hw_context->cbus_mode > CM_oCBUS_PEER_IS_MHL1_2)

static uint8_t colorSpaceTranslateInfoFrameToHw[] = {
	VAL_INPUT_FORMAT_RGB,
	VAL_INPUT_FORMAT_YCBCR422,
	VAL_INPUT_FORMAT_YCBCR444,
	VAL_INPUT_FORMAT_INTERNAL_RGB
};

static void program_ext_clock_regs(struct drv_hw_context *hw_context,
				   int crystal_khz)
{
	
	int reg_fb_div_ctl_main = 0x04;
	int reg_hdcp2x_tp1 = 0x5E;

	switch (crystal_khz) {
	case 38400:
		reg_fb_div_ctl_main = 0x0C;
		reg_hdcp2x_tp1 = 0xBC;
		break;
	case 30000:
		reg_fb_div_ctl_main = 0x06;
		reg_hdcp2x_tp1 = 0x92;
		break;
	case 24000:
		reg_fb_div_ctl_main = 0x05;
		reg_hdcp2x_tp1 = 0x75;
		break;
	case 20000:
		reg_fb_div_ctl_main = 0x04;
		reg_hdcp2x_tp1 = 0x62;
		break;
	case 19200:
	default:
		break;
	}
	mhl_tx_write_reg(hw_context, REG_DIV_CTL_MAIN,
							reg_fb_div_ctl_main);
	mhl_tx_write_reg(hw_context, REG_HDCP2X_TP1, reg_hdcp2x_tp1);
	MHL_TX_DBG_INFO("Set ext clock regs: 3F2 = %02X, 3B4 = %02X\n",
			reg_fb_div_ctl_main, reg_hdcp2x_tp1);
}

static uint8_t ok_to_proceed_with_ddc(struct drv_hw_context *hw_context)
{
	int cbus_status = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if (cbus_status < 0) {
		MHL_TX_DBG_INFO("I2C error: 0x%02x\n", cbus_status);
		return 0;
	}
	if (!(BIT_CBUS_STATUS_CBUS_HPD & cbus_status))
		return 0;
	if (!(BIT_CBUS_STATUS_CBUS_CONNECTED & cbus_status))
		return 0;
	return cbus_status & (BIT_CBUS_STATUS_CBUS_HPD |
			      BIT_CBUS_STATUS_CBUS_CONNECTED);
}

#if 0
static void enable_heartbeat(struct drv_hw_context *hw_context)
{
	switch (platform_get_flags() & PLATFORM_FLAG_HEARTBEAT_MASK) {
	case PLATFORM_VALUE_DISCONN_HEARTBEAT:
		MHL_TX_DBG_INFO
		    ("Disconnection on heartbeat failure is enabled\n");
		mhl_tx_modify_reg(hw_context, REG_DISC_CTRL1,
				  BIT_DISC_CTRL1_HB_ONLY,
				  BIT_DISC_CTRL1_HB_ONLY);
		
	case PLATFORM_VALUE_ISSUE_HEARTBEAT:
		
		MHL_TX_DBG_WARN("Heartbeat polling is enabled\n");
		mhl_tx_write_reg(hw_context, REG_MSC_HEARTBEAT_CONTROL,
				 0xA7);
		break;
	default:
		MHL_TX_DBG_INFO
		    ("Disconnection on heartbeat failure is disabled\n");
		MHL_TX_DBG_INFO("heartbeat entirely disabled for compliance\n");
		mhl_tx_modify_reg(hw_context, REG_DISC_CTRL1,
				  BIT_DISC_CTRL1_HB_ONLY, 0x00);
		mhl_tx_write_reg(hw_context, REG_MSC_HEARTBEAT_CONTROL,
				 0x27);
		break;
	}
}
#endif

static void disable_heartbeat(struct drv_hw_context *hw_context)
{
	
	mhl_tx_write_reg(hw_context, REG_MSC_HEARTBEAT_CONTROL, 0x27);
}

static void clear_auto_zone_for_mhl_3(struct drv_hw_context *hw_context)
{
	
	mhl_tx_write_reg(hw_context, REG_TX_ZONE_CTL1, 0x0);

	
	mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0,
			(VAL_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X |
			BIT_MHL_PLL_CTL0_CRYSTAL_CLK_SEL |
			BIT_MHL_PLL_CTL0_ZONE_MASK_OE));
}

static void set_auto_zone_for_mhl_1_2(struct drv_hw_context *hw_context)
{
	
	mhl_tx_write_reg(hw_context,
			 REG_TX_ZONE_CTL1,
			 VAL_TX_ZONE_CTL1_TX_ZONE_CTRL_MODE);

	
	mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0,
			(VAL_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X |
			BIT_MHL_PLL_CTL0_ZONE_MASK_OE));
}

int si_mhl_tx_drv_set_tdm_slot_allocation(struct drv_hw_context *hw_context,
					  uint8_t *vc_slot_counts,
					  bool program)
{
	int status = -EINVAL;
	uint16_t slot_total = 0;
	uint8_t idx;

	
	if (vc_slot_counts[VC_CBUS1] != 1)
		goto done;
	for (idx = 0; idx < VC_MAX; idx++) {
		slot_total += vc_slot_counts[idx];
		if (vc_slot_counts[idx] == 0)
			goto done;
	}

	switch (hw_context->cbus_mode) {
	case CM_eCBUS_S:
		if (slot_total != 25)
			goto done;
		break;
	case CM_eCBUS_D:
		if (slot_total != 200)
			goto done;
		break;
	default:
		goto done;
	}
	status = 0;

	if (program) {
		mhl_tx_write_reg(hw_context, REG_TTXSPINUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_E_MSC]);
		mhl_tx_write_reg(hw_context, REG_TTXHSICNUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_T_CBUS]);
	}

done:
	return status;
}

static int block_input_buffer_available(struct drv_hw_context *hw_context)
{
	uint16_t head, tail;
	head = hw_context->block_protocol.head;
	tail = hw_context->block_protocol.tail;
	if (head == (tail + 1)) {
		
		return 0;
	}
	return 1;
}

static int alloc_block_input_buffer(struct drv_hw_context *hw_context,
				    uint8_t **pbuffer)
{
	uint16_t head, tail;
	int index;
	head = hw_context->block_protocol.head;
	tail = hw_context->block_protocol.tail;

	if (!block_input_buffer_available(hw_context)) {
		
		return -1;
	}
	index = tail;

	if (++tail >= NUM_BLOCK_INPUT_BUFFERS)
		tail = 0;

	hw_context->block_protocol.tail = tail;

	*pbuffer = &hw_context->block_protocol.input_buffers[index][0];
	return index;
}

static void set_block_input_buffer_length(struct drv_hw_context *hw_context,
					  int block, int length)
{
	if ((block < 0) || (block > NUM_BLOCK_INPUT_BUFFERS))
		return;

	hw_context->block_protocol.input_buffer_lengths[block] = length;
}

int si_mhl_tx_drv_peek_block_input_buffer(struct mhl_dev_context *dev_context,
					  uint8_t **buffer, int *length)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	uint16_t head, tail, index;
	head = hw_context->block_protocol.head;
	tail = hw_context->block_protocol.tail;
	if (head == tail)
		return -1;

	index = head;

	*buffer = &hw_context->block_protocol.input_buffers[index][0];
	*length = hw_context->block_protocol.input_buffer_lengths[index];
	return 0;
}

void si_mhl_tx_drv_free_block_input_buffer(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	uint16_t head;
	head = hw_context->block_protocol.head;
	if (++head >= NUM_BLOCK_INPUT_BUFFERS)
		head = 0;

	hw_context->block_protocol.head = head;
}

#ifdef DUMP_ALL_REGS
void dump_all_registers(struct drv_hw_context *hw_context)
{
	uint8_t pages[] = {
		SA_TX_PAGE_0,
		SA_TX_PAGE_1,
		SA_TX_PAGE_2,
		SA_TX_PAGE_3,
		SA_TX_PAGE_4,
		SA_TX_CBUS,
		SA_TX_PAGE_6,
		SA_TX_PAGE_7,
		SA_TX_PAGE_8
	};
	int i;
	for (i = 0; i < sizeof(pages); ++i) {
		int j;
		for (j = 0; j < 256; ++j) {
			int dummy;
			dummy = mhl_tx_read_reg(hw_context, pages[i], j);
		}
	}
}
#endif
static int mhl3_block_isr(struct drv_hw_context *hw_context, uint8_t status)
{
	bool payload_encountered = false;

	if (BIT_EMSCINTR_EMSC_RFIFO_READ_ERR & status) {
		MHL_TX_DBG_ERR("%seMSC read error! status:%x%s\n",
			       ANSI_ESC_RED_TEXT, status, ANSI_ESC_RESET_TEXT);
		return 0;
	}
	if (BIT_EMSCINTR_SPI_DVLD & status) {
		if (block_input_buffer_available(hw_context)) {
			int block_index;
			uint8_t rfifo_byte_count[2];
#define EMSC_HEADER_SIZE (1 + STD_TRANSPORT_HDR_SIZE)
			uint16_t fifo_data_len;
			if (use_spi) {
				int spi_burst_stat =
				    mhl_tx_read_reg(hw_context,
						    REG_SPIBURSTSTAT);
				if (0 ==
				    (BIT_SPIBURSTSTAT_EMSC_NORMAL_MODE &
				     spi_burst_stat)) {
					return 0;
				}
			}
			mhl_tx_read_reg_block(hw_context,
					      REG_EMSCRFIFOBCNTL,
					      sizeof(rfifo_byte_count),
					      rfifo_byte_count);

			fifo_data_len =
			    ((uint16_t) rfifo_byte_count[1] << 8) | (uint16_t)
			    rfifo_byte_count[0];

			if (fifo_data_len > LOCAL_BLK_RCV_BUFFER_SIZE) {
				MHL_TX_DBG_ERR("%slength:0x%x too large%s\n",
					       ANSI_ESC_RED_TEXT,
					       fifo_data_len,
					       ANSI_ESC_RESET_TEXT)
				    return 0;
			}
			MHL_TX_DBG_INFO
			    ("EMSCINTR_SPI_DVLD intr, payload length: %d\n",
			     fifo_data_len);

			do {
				uint8_t *buffer = NULL;
				uint8_t header[EMSC_HEADER_SIZE];
				uint8_t length_remaining, count;
				struct SI_PACK_THIS_STRUCT
				    standard_transport_header_t *tport_hdr;

				if (fifo_data_len < EMSC_HEADER_SIZE) {
					MHL_TX_DBG_WARN("eMSC FIFO did not "
					     "contain enough data for a "
					     "EMSC_HEADER (%d < %ld\n",
					     fifo_data_len, EMSC_HEADER_SIZE);
					break;
				}
				if (use_spi) {
					mhl_tx_read_reg_block_spi_emsc
					    (hw_context, EMSC_HEADER_SIZE,
					     header);
				} else {
					mhl_tx_read_reg_block(hw_context,
						REG_EMSC_RCV_READ_PORT,
						EMSC_HEADER_SIZE,
						header);
				}
				MHL_TX_DBG_INFO("EMSCINTR_SPI_DVLD intr, "
				     "payload header: %02X %02X %02X\n",
				     header[0], header[1], header[2]);
				fifo_data_len -= EMSC_HEADER_SIZE;
				tport_hdr =
				    (struct SI_PACK_THIS_STRUCT
				     standard_transport_header_t *)&header[1];

				MHL_TX_DBG_INFO("EMSCINTR_SPI_DVLD intr, "
				     "tport_hdr->length_remaining: %d\n",
				     tport_hdr->length_remaining);

				if (fifo_data_len <
						tport_hdr->length_remaining) {
					MHL_TX_DBG_WARN("eMSC FIFO data count "
					     "not enough for STD header remain"
					     "ing byte count: (%d < %d\n",
					     fifo_data_len,
					     tport_hdr->length_remaining);
					break;
				}

				count = header[0];
				length_remaining = tport_hdr->length_remaining;
				MHL_TX_DBG_INFO(
							 "length_remaining: 0x"
							 "%02x count:0x%02x\n",
							 length_remaining,
							 count);
				    if (length_remaining > 0) {
					payload_encountered = true;
					block_index =
					    alloc_block_input_buffer(hw_context,
								     &buffer);

					if (use_spi) {
						mhl_tx_read_reg_block_spi_emsc
						    (hw_context,
						     length_remaining, buffer);
					} else {
						mhl_tx_read_reg_block
						 (hw_context,
						 REG_EMSC_RCV_READ_PORT,
						     length_remaining, buffer);
					}
#ifdef DUMP_ALL_REGS
					{
						PMHL_burst_id_t p_burst =
						    (PMHL_burst_id_t) buffer;
						if (burst_id_BITS_PER_PIXEL_FMT
						    == BURST_ID(*p_burst)) {
							dump_all_registers
							    (hw_context);
						}

					}
#endif
					set_block_input_buffer_length
					    (hw_context, block_index,
					     length_remaining);
					hw_context->block_protocol.
					    received_byte_count +=
					    (length_remaining +
					     STD_TRANSPORT_HDR_SIZE);
				}

				hw_context->block_protocol.
				    peer_blk_rx_buffer_avail +=
				    tport_hdr->rx_unload_ack;
				MHL_TX_DBG_WARN
				    ("RD PEER Buffer Available: %ld\n",
				     hw_context->block_protocol.
				     peer_blk_rx_buffer_avail);
				fifo_data_len -= length_remaining;
				if (!block_input_buffer_available(hw_context))
					break;
			} while (fifo_data_len > 0);
			if (payload_encountered) {
				hw_context->intr_info->flags |=
				    DRV_INTR_EMSC_INCOMING;
			}
		}
	}
	return 0;
}

static int coc_isr(struct drv_hw_context *hw_context, uint8_t coc_int_status)
{
	int ret_val = 0;

	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
				   struct mhl_dev_context, drv_context);

	if (BIT_COC_PLL_LOCK_STATUS_CHANGE & coc_int_status)
		MHL_TX_DBG_INFO("COC PLL lock status change\n");

	if (si_mhl_tx_drv_connection_is_mhl3(dev_context)) {

		if (BIT_COC_CALIBRATION_DONE & coc_int_status) {
			int calibration_stat, retry = 0;
			uint8_t calibrated_value;
			MHL_TX_DBG_INFO("Calibration done\n");

			calibration_stat = mhl_tx_read_reg(hw_context,
							REG_COC_STAT_0);
			calibration_stat &=
			    BITS_COC_STAT_0_CALIBRATION_MASK;
			calibrated_value =
			    BITS_COC_STAT_0_PLL_LOCKED |
			    BITS_COC_STAT_0_CALIBRATION_STATE_2;

			if (calibrated_value == calibration_stat) {
				
				mhl_tx_write_reg(hw_context,
						REG_COC_CTLB, 0x00);
				MHL_TX_DBG_ERR("CoC in calibrated state\n");
				retry = 0;
				enable_intr(hw_context, INTR_TDM,
					    BIT_TDM_INTR_SYNC_DATA_MASK |
					    BIT_TDM_INTR_SYNC_WAIT_MASK);
				hw_context->cbus_mode =
				    CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED;
				hw_context->intr_info->flags |=
				    DRV_INTR_COC_CAL;
			} else {
				MHL_TX_DBG_ERR("calibration state: 0x%02X\n",
					       calibration_stat);
			}

			if (retry) {
				uint8_t coc_ctl0;
				MHL_TX_DBG_ERR("Retry calibration\n");

				coc_ctl0 = mhl_tx_read_reg(hw_context,
						REG_COC_CTL0);
				mhl_tx_write_reg(hw_context,
						REG_COC_CTL0,
						coc_ctl0 & ~0x04);
				mhl_tx_write_reg(hw_context,
						REG_COC_CTL14, 0x00);
				mhl_tx_write_reg(hw_context,
						REG_COC_CTL15, 0x80);
				msleep(20);
				mhl_tx_write_reg(hw_context,
						REG_COC_CTL0, coc_ctl0);
			}
		}
	}

	return ret_val;
}

static int tdm_isr(struct drv_hw_context *hw_context, uint8_t intr_status)
{
	int ret_val = 0;
	uint8_t tdm_status;

	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
				   struct mhl_dev_context, drv_context);

	if (si_mhl_tx_drv_connection_is_mhl3(dev_context)) {

		if (BIT_TDM_INTR_SYNC_DATA & intr_status) {

			MHL_TX_DBG_INFO("TDM in SYNC_DATA state.\n");
			tdm_status =
			    mhl_tx_read_reg(hw_context, REG_TRXSTA2);

			if ((tdm_status & MSK_TDM_SYNCHRONIZED) ==
			    VAL_TDM_SYNCHRONIZED) {

				MHL_TX_DBG_ERR("TDM is synchronized\n");

				if (CM_eCBUS_S > hw_context->cbus_mode) {
					hw_context->cbus_mode = CM_eCBUS_S;
					hw_context->intr_info->flags |=
					    DRV_INTR_TDM_SYNC;
					hw_context->block_protocol.
					    received_byte_count = 0;

					si_mhl_tx_send_blk_rcv_buf_info
					    (dev_context);
#ifdef EARLY_HSIC
					hsic_init(hw_context);
#endif
				}
			} else {
				MHL_TX_DBG_ERR
				    ("TDM not synchronized, retrying\n");
				mhl_tx_write_reg(hw_context,
						REG_MHL_PLL_CTL2, 0x00);
				mhl_tx_write_reg(hw_context,
						REG_MHL_PLL_CTL2, 0x80);
			}
		}

		if (BIT_TDM_INTR_SYNC_WAIT & intr_status)
			MHL_TX_DBG_ERR("TDM in SYNC_WAIT state.\n");
	}

	return ret_val;
}

static int int_link_trn_isr(struct drv_hw_context *hw_context,
			    uint8_t intr_status)
{
	if (IN_MHL3_MODE(hw_context)) {
		MHL_TX_DBG_ERR("%sTODO: implement link training interrupt%s\n",
			       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		return 0;
	} else {
		return 0;
	}
}

void si_set_cbus_mode_leds(enum cbus_mode_e mode_sel)
{
	switch (mode_sel) {
	case CM_NO_CONNECTION:
		MHL_TX_DBG_INFO("CBUS MODE: NO CONNECTION\n");
		set_pin(0, X02_USB_LED15_AMBER, 1);
		set_pin(0, X02_USB_LED15_GREEN, 1);
		set_pin(0, LED_USB_MODE, 0);
		break;
	case CM_oCBUS_PEER_VERSION_PENDING:
		MHL_TX_DBG_INFO("CBUS MODE: VERSION PENDING\n");
		set_pin(0, X02_USB_LED15_AMBER, 0);
		set_pin(0, X02_USB_LED15_GREEN, 0);
		set_pin(0, LED_USB_MODE, 1);
		break;
	case CM_oCBUS_PEER_IS_MHL1_2:
		MHL_TX_DBG_INFO("CBUS MODE: PEER IS MHL1_2\n");
		set_pin(0, X02_USB_LED15_AMBER, 0);
		set_pin(0, X02_USB_LED15_GREEN, 1);
		set_pin(0, LED_USB_MODE, 1);
		break;
	case CM_oCBUS_PEER_IS_MHL3:
	case CM_eCBUS_S:
	case CM_eCBUS_D:
		MHL_TX_DBG_INFO("CBUS MODE: MHL3\n");
		set_pin(0, X02_USB_LED15_AMBER, 1);
		set_pin(0, X02_USB_LED15_GREEN, 0);
		set_pin(0, LED_USB_MODE, 1);
		break;
	default:
		MHL_TX_DBG_INFO("CBUS MODE: %02X\n", mode_sel);
		break;
	}
}

int si_mhl_tx_drv_switch_cbus_mode(struct drv_hw_context *hw_context,
				   enum cbus_mode_e mode_sel)
{
	int status = 1;
	uint8_t slot_total;

	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
				   struct mhl_dev_context, drv_context);

	MHL_TX_DBG_WARN("Switch cbus_mode from %x to %x\n",
		       hw_context->cbus_mode, mode_sel);

	if (hw_context->cbus_mode < CM_oCBUS_PEER_VERSION_PENDING)
		return -1;

	switch (mode_sel) {
	case CM_oCBUS_PEER_IS_MHL1_2:
		MHL_TX_DBG_ERR("Switch to MHL1/2 oCBUS mode\n");
#define  BIT_CBUS_MSC_COMPATIBILITY_CONTROL_ENABLE_XDEVCAP	0x80
		mhl_tx_write_reg(hw_context,
				 REG_CBUS_MSC_COMPATIBILITY_CONTROL,
				 0x02);
		mhl_tx_write_reg(hw_context, REG_M3_CTRL,
				 VAL_M3_CTRL_MHL1_2_VALUE);
		mhl_tx_write_reg(hw_context, REG_DPD,
				 BIT_DPD_PWRON_PLL |
				 BIT_DPD_PDNTX12 |
				 BIT_DPD_OSC_EN);
		enable_intr(hw_context, INTR_COC, 0);
		set_auto_zone_for_mhl_1_2(hw_context);

		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xBC);

		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL1, 0xBB);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL3, 0x48);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3F);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x2F);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL6, 0x2A);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL7, 0x08);
		hw_context->cbus_mode = CM_oCBUS_PEER_IS_MHL1_2;
		break;
	case CM_oCBUS_PEER_IS_MHL3:
		mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x40);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0x07);
		hw_context->cbus_mode = CM_oCBUS_PEER_IS_MHL3;
		break;

	case CM_eCBUS_S:
		si_mhl_tx_initialize_block_transport(dev_context);
		si_mhl_tx_drv_enable_emsc_block(hw_context);

		MHL_TX_DBG_WARN("hpd status: 0x%02x\n",
				mhl_tx_read_reg(hw_context,
						REG_CBUS_STATUS));
		hw_context->cbus_mode = CM_TRANSITIONAL_TO_eCBUS_S;

		mhl_tx_write_reg(hw_context, REG_TTXSPINUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_E_MSC]);
		slot_total = hw_context->tdm_virt_chan_slot_counts[VC_E_MSC];
		mhl_tx_write_reg(hw_context, REG_TTXHSICNUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_T_CBUS]);
		slot_total += hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS];
		mhl_tx_write_reg(hw_context, REG_TTXTOTNUMS, 24);

		
		mhl_tx_write_reg(hw_context, REG_PWD_SRST, 0xA0);

		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0xBC);

		
		mhl_tx_write_reg(hw_context, REG_PWD_SRST, 0x20);

		
		mhl_tx_write_reg(hw_context, REG_COC_CTLB, 0x01);

		mhl_tx_write_reg(hw_context, REG_COC_CTL14, 0x03);
		mhl_tx_write_reg(hw_context, REG_COC_CTL15, 0x80);

		msleep(50);

		mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x5C);

		mhl_tx_write_reg(hw_context, REG_COC_CTL14, 0x00);
		mhl_tx_write_reg(hw_context, REG_COC_CTL15, 0x80);

		mhl_tx_write_reg(hw_context, REG_CBUS3_CNVT, 0x85);

		break;

	case CM_eCBUS_D:
		hw_context->cbus_mode = CM_TRANSITIONAL_TO_eCBUS_D;

		si_mhl_tx_initialize_block_transport(dev_context);
		si_mhl_tx_drv_enable_emsc_block(hw_context);

		mhl_tx_write_reg(hw_context, REG_TTXSPINUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_E_MSC]);
		slot_total = hw_context->tdm_virt_chan_slot_counts[VC_E_MSC];
		mhl_tx_write_reg(hw_context, REG_TTXHSICNUMS,
				 hw_context->
				 tdm_virt_chan_slot_counts[VC_T_CBUS]);
		slot_total += hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS];
		mhl_tx_write_reg(hw_context, REG_TTXTOTNUMS, 199);

		break;

	default:
		MHL_TX_DBG_ERR("Invalid or unsupported CBUS mode specified\n");
		status = -EINVAL;
		break;
	}
	si_set_cbus_mode_leds(mode_sel);
	return status;
}

enum cbus_mode_e si_mhl_tx_drv_get_cbus_mode(struct mhl_dev_context
					     *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	return hw_context->cbus_mode;
}

char *si_mhl_tx_drv_get_cbus_mode_str(enum cbus_mode_e cbus_mode)
{
	char *cbus_mode_str;

	switch (cbus_mode) {
	case CM_NO_CONNECTION:
		cbus_mode_str = "not connected";
		break;
	case CM_oCBUS_PEER_VERSION_PENDING:
		cbus_mode_str = "oCBUS (MHL?)";
		break;
	case CM_oCBUS_PEER_IS_MHL1_2:
		cbus_mode_str = "oCBUS (MHL1/2)";
		break;
	case CM_oCBUS_PEER_IS_MHL3:
		cbus_mode_str = "oCBUS (MHL3)";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_S:
		cbus_mode_str = "-> to eCBUS_S";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
		cbus_mode_str = "-> to eCBUS_S (cal)";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_D:
		cbus_mode_str = "-> to eCBUS_D";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		cbus_mode_str = "-> to eCBUS_D (cal)";
		break;
	case CM_eCBUS_S:
		cbus_mode_str = "eCBUS_S";
		break;
	case CM_eCBUS_D:
		cbus_mode_str = "eCBUS_D";
		break;
	default:
		cbus_mode_str = "invalid";
		break;
	}

	return cbus_mode_str;
}

static void disable_gen2_write_burst_rcv(struct drv_hw_context *hw_context)
{
	if (hw_context->gen2_write_burst_rcv) {
		
		mhl_tx_write_reg(hw_context, REG_MDT_XMIT_CONTROL, 0);
		mhl_tx_write_reg(hw_context, REG_MDT_RCV_CONTROL, 0);
			
			
			
		MHL_TX_DBG_INFO("%sdisabled GEN2%s\n",
				ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_rcv = false;
	}
}

static void disable_gen2_write_burst_xmit(struct drv_hw_context *hw_context)
{
	if (hw_context->gen2_write_burst_xmit) {
		
		mhl_tx_write_reg(hw_context, REG_MDT_XMIT_CONTROL, 0);
		MHL_TX_DBG_INFO("%sdisabled GEN2%s\n",
				ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_xmit = false;
	}
}

static void enable_gen2_write_burst_rcv(struct drv_hw_context *hw_context)
{
	
	if (!hw_context->gen2_write_burst_rcv) {
		
		mhl_tx_write_reg(hw_context, REG_MDT_RCV_TIMEOUT, 100);
		mhl_tx_write_reg(hw_context, REG_MDT_RCV_CONTROL,
				 BIT_MDT_RCV_CONTROL_MDT_RCV_EN |
				 hw_context->delayed_hawb_enable_reg_val);

		MHL_TX_DBG_INFO("%senabled GEN2%s\n",
				ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_rcv = true;
	}
}

static void enable_gen2_write_burst_xmit(struct drv_hw_context *hw_context)
{
	
	if (!hw_context->gen2_write_burst_xmit) {
		mhl_tx_write_reg(hw_context, REG_MDT_XMIT_CONTROL,
			BIT_MDT_XMIT_CONTROL_MDT_XMIT_EN |
			BIT_MDT_XMIT_CONTROL_MDT_XMIT_FIXED_BURST_LEN);

		MHL_TX_DBG_INFO("%senabled GEN2%s\n",
				ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_xmit = true;
	}
}

#ifndef MANUAL_EDID_FETCH
static void freeze_MHL_connect(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_DISC_STAT1, 0x08);
	mhl_tx_modify_reg(hw_context, REG_DISC_CTRL5,
			  BIT_DISC_CTRL5_DSM_OVRIDE,
			  BIT_DISC_CTRL5_DSM_OVRIDE);
}

static void unfreeze_MHL_connect(struct drv_hw_context *hw_context)
{
	mhl_tx_modify_reg(hw_context, REG_DISC_CTRL5,
			  BIT_DISC_CTRL5_DSM_OVRIDE, 0);
}
#endif

void si_mhl_tx_drv_shut_down_HDCP2(struct drv_hw_context *hw_context)
{
	int ddcm_status;
	int count = 0;
	
	enable_intr(hw_context, INTR_HDCP2, 0x00);

	
	hw_context->hdcp2_started = false;
	mhl_tx_write_reg(hw_context, REG_HDCP2X_POLL_CS, 0x71);

	while (0 <=
	       (ddcm_status =
		mhl_tx_read_reg(hw_context, REG_HDCP2X_DDCM_STS))) {
		if (0 ==
		    (MSK_HDCP2X_DDCM_STS_HDCP2X_DDCM_CTL_CS_3_0 &
		     ddcm_status)) {
			break;
		}
		if (++count > 256)
			break;

		MHL_TX_DBG_WARN("shutting down HDCP\n");
	}

	
	mhl_tx_write_reg(hw_context, REG_HDCP2X_CTRL_0, 0x82);

	MHL_TX_DBG_WARN("HDCP2 Off; Last HDCP2X_DDCM Status %02X;\n",
			ddcm_status);

	mhl_tx_modify_reg(hw_context, REG_M3_P0CTRL,
			  BIT_M3_P0CTRL_MHL3_P0_HDCP_EN, 0x00);

	
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP2].stat_addr, 0xff);
}

static bool issue_edid_read_request(struct drv_hw_context *hw_context,
					   uint8_t block_number)
{
	if (ok_to_proceed_with_ddc(hw_context)) {
		int ddc_status;
		ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);
		if (BIT_DDC_STATUS_DDC_BUS_LOW & ddc_status) {
			int lm_ddc;
			lm_ddc = mhl_tx_read_reg(hw_context, REG_LM_DDC);
			
			mhl_tx_write_reg(hw_context, REG_LM_DDC,
					 lm_ddc |
					 VAL_LM_DDC_SW_TPI_EN_DISABLED);
			
			mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
					 ddc_status &
					 ~BIT_DDC_STATUS_DDC_BUS_LOW);

			
			mhl_tx_write_reg(hw_context, REG_LM_DDC, lm_ddc);
		}
		MHL_TX_DBG_INFO("\n\tRequesting EDID block:%d\n"
				"\tcurrentEdidRequestBlock:%d\n"
				"\tedidFifoBlockNumber:%d"
				"ddc_status:0x%02x\n",
				block_number,
				hw_context->current_edid_req_blk,
				hw_context->edid_fifo_block_number, ddc_status);
		
		mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
				 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
				 VAL_EDID_CTRL_DEVCAP_SELECT_EDID |
				 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
				 | ((block_number & 0x01) << 2)
				 | VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);

#ifndef MANUAL_EDID_FETCH
#define SWWA_BZ30759
#endif
#ifdef SWWA_BZ30759
		freeze_MHL_connect(hw_context);
#endif
		
		if (0 == block_number) {
			
			enable_intr(hw_context, INTR_EDID,
				    (BIT_INTR9_DEVCAP_DONE_MASK
				     | BIT_INTR9_EDID_DONE_MASK
				     | BIT_INTR9_EDID_ERROR));
			mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
				BIT_TPI_CBUS_START_GET_EDID_START_0);
		} else {
			uint8_t param = (1 << (block_number - 1));
			MHL_TX_DBG_INFO("EDID HW Assist: Programming "
					"Reg 0x%02X to 0x%02X\n",
					REG_EDID_START_EXT, param);
			mhl_tx_write_reg(hw_context, REG_EDID_START_EXT,
					 param);
		}

		return true;
	} else {
		MHL_TX_DBG_INFO("\n\tNo HPD for EDID block request:%d\n"
				"\tcurrentEdidRequestBlock:%d\n"
				"\tedidFifoBlockNumber:%d\n",
				block_number,
				hw_context->current_edid_req_blk,
				hw_context->edid_fifo_block_number);
		return false;
	}
}

void mhl_tx_drv_send_block(struct drv_hw_context *hw_context,
			   struct block_req *req)
{
	uint8_t count;

	count = req->sub_payload_size;
	MHL_TX_DBG_INFO(" req->sub_payload_size: %d req->count: %d\n", count,
			req->count);

	MHL_TX_DBG_WARN("total bytes to ack: %d\n",
			hw_context->block_protocol.received_byte_count);
	if (hw_context->block_protocol.received_byte_count >= 256) {

		req->payload->hdr_and_burst_id.tport_hdr.rx_unload_ack = 255;
		hw_context->block_protocol.received_byte_count -= 255;
	} else {
		req->payload->hdr_and_burst_id.tport_hdr.rx_unload_ack
		    = (uint8_t) hw_context->block_protocol.received_byte_count;
		hw_context->block_protocol.received_byte_count = 0;
	}

	MHL_TX_DBG_WARN("rx_unload_ack: %d\n",
			req->payload->
			hdr_and_burst_id.tport_hdr.rx_unload_ack);
	if (use_spi) {
		mhl_tx_write_block_spi_emsc(hw_context, req);
	} else {
		mhl_tx_write_reg_block(hw_context,
				       REG_EMSC_XMIT_WRITE_PORT,
				       req->count, &req->payload->as_bytes[0]);
	}
}

uint8_t si_mhl_tx_drv_get_pending_hawb_write_status(struct mhl_dev_context
						    *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	return hw_context->hawb_write_pending;
}


uint8_t si_mhl_tx_drv_hawb_xfifo_avail(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;

	return MSK_MDT_XFIFO_STAT_MDT_XFIFO_LEVEL_AVAIL
	    & mhl_tx_read_reg(hw_context, REG_MDT_XFIFO_STAT);
}

#ifdef MANUAL_EDID_FETCH
static uint8_t fetch_edid_block(struct drv_hw_context *hw_context,
				uint8_t *buffer, uint8_t block_num,
				bool trigger_on_last)
{
	int lm_ddc, ddc_cmd, ddc_status, ddc_address, ddc_limit, step, dout_cnt,
	    intr3_status, cbus_status;
	cbus_status = ok_to_proceed_with_ddc(hw_context);
	lm_ddc = mhl_tx_read_reg(hw_context, REG_LM_DDC);
	ddc_cmd = mhl_tx_read_reg(hw_context, REG_DDC_CMD);
	ddc_cmd &= ~MSK_DDC_CMD_DDC_CMD;
	
	enable_intr(hw_context, INTR_EDID, 0);

	hw_context->hdcp2_started = false;
	
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE);
	
	mhl_tx_write_reg(hw_context, REG_HDCP2X_POLL_CS, 0x71);

	
	mhl_tx_write_reg(hw_context, REG_HDCP2X_CTRL_0, 0x02);

	
	mhl_tx_write_reg(hw_context, REG_LM_DDC,
			 lm_ddc | VAL_LM_DDC_SW_TPI_EN_DISABLED);

	for (step = 0; step < 256; ++step) {
		ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);
		if (0 == (BIT_DDC_STATUS_DDC_I2C_IN_PROG & ddc_status))
			break;

		mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
				 BIT_DDC_STATUS_DDC_FIFO_EMPTY);
	}
	

	mhl_tx_write_reg(hw_context, REG_DDC_ADDR, 0xA0);
	step = 16;
	ddc_limit = (block_num << 7) + EDID_BLOCK_SIZE;
	for (ddc_address = block_num << 7; ddc_address < ddc_limit;
	     ddc_address += step) {
		ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);
		mhl_tx_write_reg(hw_context, REG_DDC_CMD,
				 ddc_cmd | VAL_DDC_CMD_DDC_CMD_ABORT);
		mhl_tx_write_reg(hw_context, REG_DDC_CMD,
				 ddc_cmd |
				 VAL_DDC_CMD_DDC_CMD_CLEAR_FIFO);
		mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
				 BIT_DDC_STATUS_DDC_FIFO_EMPTY);

		intr3_status = mhl_tx_read_reg(hw_context, REG_INTR3);
		mhl_tx_write_reg(hw_context, REG_INTR3, intr3_status);

		mhl_tx_write_reg(hw_context, REG_DDC_SEGM,
				 HIGH_BYTE_16(ddc_address));
		mhl_tx_write_reg(hw_context, REG_DDC_OFFSET,
				 LOW_BYTE_16(ddc_address));
		mhl_tx_write_reg(hw_context, REG_DDC_DIN_CNT1,
				 LOW_BYTE_16(step));
		mhl_tx_write_reg(hw_context, REG_DDC_DIN_CNT2,
				 HIGH_BYTE_16(step));

		mhl_tx_write_reg(hw_context, REG_DDC_CMD,
				 ddc_cmd |
				 VAL_DDC_CMD_ENH_DDC_READ_NO_ACK);

		do {
			intr3_status =
			    mhl_tx_read_reg(hw_context, REG_INTR3);
			cbus_status = ok_to_proceed_with_ddc(hw_context);
			if (BIT_DDC_CMD_DONE & intr3_status)
				break;
		} while (cbus_status);
		if (!cbus_status)
			break;

		ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);
		dout_cnt = mhl_tx_read_reg(hw_context, REG_DDC_DOUT_CNT);

		if (use_spi) {
			int k;
			for (k = 0; k < step; ++k) {
				buffer[(ddc_address + k) % EDID_BLOCK_SIZE] =
				    (uint8_t) mhl_tx_read_reg(hw_context,
							  REG_DDC_DATA);
			}
		} else {
			mhl_tx_read_reg_block(hw_context, REG_DDC_DATA,
					      step,
					      &buffer[ddc_address %
						      EDID_BLOCK_SIZE]);
		}

		if (0 == ddc_address) {
			struct EDID_block0_t *p_EDID_block_0 =
				(struct EDID_block0_t *) buffer;
			if (!si_mhl_tx_check_edid_header
			    (hw_context->intr_info->edid_parser_context,
			     p_EDID_block_0)) {
				int start = ddc_address % EDID_BLOCK_SIZE;
				
				MHL_TX_DBG_ERR("%02x %02x %02x %02x "
				     "%02x %02x %02x %02x\n",
				     buffer[start + 0], buffer[start + 1],
				     buffer[start + 2], buffer[start + 3],
				     buffer[start + 4], buffer[start + 5],
				     buffer[start + 6], buffer[start + 7]
				    );
				ddc_address -= step;
			}
		}

		if ((ddc_address + step) >= ddc_limit) {
			if (0 == block_num) {
				struct EDID_block0_t *p_EDID_block_0 =
				    (struct EDID_block0_t *) buffer;
				if (0 == p_EDID_block_0->extension_flag)
					trigger_on_last = true;
			}

			if (trigger_on_last) {
				enable_intr(hw_context, INTR_DDC,
					    BIT_DDC_CMD_DONE);
			} else {
				mhl_tx_write_reg(hw_context, REG_INTR3,
						 intr3_status);
			}
		} else {
			mhl_tx_write_reg(hw_context, REG_INTR3,
					 intr3_status);
		}
	}

	
	mhl_tx_write_reg(hw_context, REG_LM_DDC, lm_ddc);
	return cbus_status;
}
#endif
uint8_t si_mhl_tx_drv_send_cbus_command(struct drv_hw_context *hw_context,
					struct cbus_req *req)
{
	uint8_t ret_val = req->command;
	uint8_t block_write_buffer[3];
	int success;
	uint8_t cbus_status;

	switch (req->command) {
	case MHL_WRITE_BURST:
		break;
	default:
		disable_gen2_write_burst_rcv(hw_context);
		
		disable_gen2_write_burst_xmit(hw_context);
	}

	hw_context->current_cbus_req = *req;
	switch (req->command) {
	case MHL_SEND_3D_REQ_OR_FEAT_REQ:
		
		
		
		
		hw_context->delayed_hawb_enable_reg_val =
		    BIT_MDT_RCV_CONTROL_MDT_DELAY_RCV_EN;
		enable_gen2_write_burst_rcv(hw_context);
		hw_context->cbus1_state = CBUS1_MSC_PEND_DLY_RCV_EN;

		mhl_tx_write_reg(hw_context, REG_MSC_CMD_OR_OFFSET,
				 req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_1ST_TRANSMIT_DATA,
				 req->reg_data);
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
			BIT_MSC_COMMAND_START_MSC_WRITE_STAT_CMD);
		break;

	case MHL_SET_INT:
	case MHL_WRITE_STAT:
	case MHL_WRITE_XSTAT:
		MHL_TX_DBG_INFO("->reg: 0x%02x data: 0x%02x\n",
				req->reg, req->reg_data);
#ifdef WRITE_STAT_SET_INT_COALESCE
		mhl_tx_write_reg_block(hw_context, REG_MSC_CMD_OR_OFFSET,
				       2, &req->reg);
#else
		mhl_tx_write_reg(hw_context, REG_MSC_CMD_OR_OFFSET,
				 req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_1ST_TRANSMIT_DATA,
				 req->reg_data);
#endif
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
			BIT_MSC_COMMAND_START_MSC_WRITE_STAT_CMD);
		if (MHL_RCHANGE_INT == hw_context->current_cbus_req.reg) {

			if (MHL2_INT_3D_REQ & hw_context->current_cbus_req.
			    reg_data) {
				MHL_TX_DBG_WARN("3D_REQ sent\n");
			}

			if (MHL3_INT_FEAT_REQ & hw_context->current_cbus_req.
			    reg_data) {
				MHL_TX_DBG_WARN("FEAT_REQ sent\n");
			}

			if (MHL_INT_GRT_WRT & hw_context->current_cbus_req.
			    reg_data) {
				MHL_TX_DBG_WARN("GRT_WRT sent\n");
			}
		}
		break;

	case MHL_READ_DEVCAP:
		MHL_TX_DBG_WARN("Read DEVCAP\n");
		
		enable_intr(hw_context, INTR_EDID, BIT_INTR9_DEVCAP_DONE);

		mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
				 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
				 VAL_EDID_CTRL_DEVCAP_SELECT_DEVCAP |
				 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
				 | VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);

		
		mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
				 BIT_TPI_CBUS_START_GET_DEVCAP_START);
		break;
	case MHL_READ_DEVCAP_REG:
		MHL_TX_DBG_INFO("Trigger DEVCAP_REG Read\n");
		MHL_TX_DBG_INFO("Read DEVCAP (0x%02x)\n", req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_CMD_OR_OFFSET,
			req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
			BIT_MSC_COMMAND_START_MSC_READ_DEVCAP_CMD);
		break;

	case MHL_READ_XDEVCAP:
		MHL_TX_DBG_INFO("Trigger XDEVCAP Read\n");
		
		enable_intr(hw_context, INTR_EDID, BIT_INTR9_DEVCAP_DONE);

		mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
				 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
				 BIT_EDID_CTRL_XDEVCAP_EN |
				 VAL_EDID_CTRL_DEVCAP_SELECT_DEVCAP |
				 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
				 | VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);
		
		mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
				 BIT_TPI_CBUS_START_GET_DEVCAP_START);
		break;
	case MHL_READ_XDEVCAP_REG:
		MHL_TX_DBG_INFO("Read XDEVCAP_REG (0x%02x)\n", req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_CMD_OR_OFFSET,
				 req->reg);
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
			BIT_MSC_COMMAND_START_MSC_READ_DEVCAP_CMD);
		break;

	case MHL_READ_EDID_BLOCK:
		hw_context->current_edid_req_blk = 0;
#ifdef MANUAL_EDID_FETCH
#define HPD_AND_CONNECTED	(BIT_CBUS_STATUS_CBUS_HPD | \
				BIT_CBUS_STATUS_CBUS_CONNECTED)

		success = 1;
		cbus_status =
		    fetch_edid_block(hw_context, hw_context->edid_block,
				hw_context->current_edid_req_blk, false);
		if (cbus_status == (BIT_CBUS_STATUS_CBUS_HPD |
		     BIT_CBUS_STATUS_CBUS_CONNECTED)) {
			int num_extensions;

			num_extensions =
			    si_mhl_tx_get_num_cea_861_extensions
			    (hw_context->intr_info->edid_parser_context,
			     hw_context->current_edid_req_blk);
			if (num_extensions < 0) {
				success = 0;
			} else {
				for (hw_context->current_edid_req_blk = 1;
				     hw_context->current_edid_req_blk <=
				     num_extensions;
				     hw_context->current_edid_req_blk++) {
					cbus_status =
					    fetch_edid_block(hw_context,
						hw_context->edid_block,
						hw_context->
						    current_edid_req_blk,
						(hw_context->
						    current_edid_req_blk ==
						    num_extensions) ?
						true : false);
					if (cbus_status != HPD_AND_CONNECTED) {
						success = 0;
						break;
					}
					num_extensions =
					    si_mhl_tx_get_num_cea_861_extensions
					    (hw_context->intr_info->
					     edid_parser_context,
					     hw_context->current_edid_req_blk);
					if (num_extensions < 0) {
						MHL_TX_DBG_ERR(
						    "edid problem:%d\n",
						    num_extensions);
						success = 0;
						break;
					}
				}
			}
		}
#else
		success = issue_edid_read_request(hw_context,
				hw_context->current_edid_req_blk);
#endif
		ret_val = success ? ret_val : 0;
		break;

	case MHL_GET_STATE:
	case MHL_GET_VENDOR_ID:	
	case MHL_SET_HPD:	
	case MHL_CLR_HPD:	
	case MHL_GET_SC1_ERRORCODE:	
	case MHL_GET_DDC_ERRORCODE:	
	case MHL_GET_MSC_ERRORCODE:	
	case MHL_GET_SC3_ERRORCODE:	
		MHL_TX_DBG_INFO("Sending MSC command %02x, %02x, %02x\n",
				req->command, req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_MSC_CMD_OR_OFFSET,
				 req->command);
		mhl_tx_write_reg(hw_context, REG_MSC_1ST_TRANSMIT_DATA,
				 req->reg_data);
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
				 BIT_MSC_COMMAND_START_MSC_PEER_CMD);
		break;

	case MHL_MSC_MSG:
		MHL_TX_DBG_INFO("MHL_MSC_MSG sub cmd: 0x%02x data: 0x%02x\n",
				req->msg_data[0], req->msg_data[1]);
		block_write_buffer[0] = req->command;
		block_write_buffer[1] = req->msg_data[0];
		block_write_buffer[2] = req->msg_data[1];

		mhl_tx_write_reg_block(hw_context, REG_MSC_CMD_OR_OFFSET,
				       3, block_write_buffer);
		mhl_tx_write_reg(hw_context, REG_MSC_COMMAND_START,
				 BIT_MSC_COMMAND_START_MSC_MSC_MSG_CMD);
		break;

	case MHL_WRITE_BURST:
		MHL_TX_DBG_INFO
		    ("MHL_WRITE_BURST offset: 0x%02x length: 0x%02x\n",
		     req->burst_offset, req->length);
		hw_context->hawb_write_pending = true;
		enable_gen2_write_burst_xmit(hw_context);
		mhl_tx_write_reg_block(hw_context,
				       REG_MDT_XMIT_WRITE_PORT,
				       req->length, req->msg_data);
		ret_val =
		    (MSK_MDT_XFIFO_STAT_MDT_XFIFO_LEVEL_AVAIL &
		     mhl_tx_read_reg(hw_context, REG_MDT_XFIFO_STAT)
		    );
		break;

	default:
		MHL_TX_DBG_ERR("Unsupported command 0x%02x detected!\n",
			       req->command);
		ret_val = 0;
		break;
	}

	return ret_val;
}

void si_mhl_tx_drv_set_3d_mode(struct drv_hw_context *hw_context, bool do_3D,
	enum _3D_structure_e three_d_mode)
{
	if (do_3D) {
		if (tdsFramePacking == three_d_mode) {
			MHL_TX_DBG_INFO("using frame packing\n");

			mhl_tx_write_reg(hw_context, REG_VID_OVRRD,
				BIT_VID_OVRRD_PP_AUTO_DISABLE |
				VAL_VID_OVRRD_3DCONV_EN_FRAME_PACK);
		} else {
			MHL_TX_DBG_INFO("NOT using frame packing\n");
			mhl_tx_write_reg(hw_context, REG_VID_OVRRD,
					 BIT_VID_OVRRD_PP_AUTO_DISABLE);
		}
	} else {
		MHL_TX_DBG_INFO("NOT using frame packing\n");
		mhl_tx_write_reg(hw_context, REG_VID_OVRRD,
				 BIT_VID_OVRRD_PP_AUTO_DISABLE);
	}
}

struct SI_PACK_THIS_STRUCT si_incoming_hw_timing_t {
	uint8_t h_total_low;
	uint8_t h_total_high;
	uint8_t v_total_low;
	uint8_t v_total_high;
	uint8_t iadjust;
	uint8_t pol_detect;
	uint8_t columns_low;
	uint8_t columns_high;
	uint8_t rows_low;
	uint8_t rows_high;
	uint8_t field_rate_low;
	uint8_t field_rate_high;
};

uint16_t si_mhl_tx_drv_get_incoming_timing(struct drv_hw_context *hw_context,
	struct si_incoming_timing_t *p_timing)
{
	uint16_t ret_val;
	struct si_incoming_hw_timing_t hw_timing;

	ret_val =
	    mhl_tx_read_reg_block(hw_context, REG_H_RESL,
				  sizeof(hw_timing), (uint8_t *) &hw_timing);
	p_timing->h_total = (((uint16_t) hw_timing.h_total_high) << 8)
	    | (uint16_t) hw_timing.h_total_low;
	p_timing->v_total = (((uint16_t) hw_timing.v_total_high) << 8)
	    | (uint16_t) hw_timing.v_total_low;
	p_timing->columns = (((uint16_t) hw_timing.columns_high) << 8)
	    | (uint16_t) hw_timing.columns_low;
	p_timing->rows = (((uint16_t) hw_timing.rows_high) << 8)
	    | (uint16_t) hw_timing.rows_low;
	p_timing->field_rate = (((uint16_t) hw_timing.field_rate_high) << 8)
	    | (uint16_t) hw_timing.field_rate_low;
	return ret_val;
}

int si_mhl_tx_drv_get_aksv(struct drv_hw_context *hw_context, uint8_t * buffer)
{
	memcpy(buffer, hw_context->aksv, 5);
	return 0;
}

void si_mhl_tx_drv_skip_to_next_edid_block(struct drv_hw_context *hw_context)
{
	hw_context->edid_fifo_block_number++;
}

int si_mhl_tx_drv_get_edid_fifo_partial_block(struct drv_hw_context *hw_context,
					      uint8_t start, uint8_t length,
					      uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	offset = EDID_BLOCK_SIZE * (hw_context->edid_fifo_block_number & 0x01);
	offset += start;

	MHL_TX_DBG_INFO("%p %p\n", hw_context,
			edid_buf);
	if (EDID_BLOCK_SIZE == (offset + length))
		hw_context->edid_fifo_block_number++;

#ifdef MANUAL_EDID_FETCH
	memcpy(edid_buf, &hw_context->edid_block[start], length);
#else
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, offset);

	ret_val =
	    mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA,
				  length, edid_buf);
#endif

	DUMP_EDID_BLOCK(0, edid_buf, length);

	ret_val = ok_to_proceed_with_ddc(hw_context);
	if (!ret_val) {
		MHL_TX_DBG_INFO("No HPD ret_val:0x%02x\n", ret_val);
		return ne_NO_HPD;
	} else {
		MHL_TX_DBG_INFO("EDID block read complete. ret_val:0x%02x\n",
				ret_val);
		return ne_SUCCESS;
	}
}

int si_mhl_tx_drv_get_edid_fifo_next_block(struct drv_hw_context *hw_context,
					   uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	offset = EDID_BLOCK_SIZE * (hw_context->edid_fifo_block_number & 0x01);

	MHL_TX_DBG_INFO("%p %p\n", hw_context,
			edid_buf);
	hw_context->edid_fifo_block_number++;

#ifdef MANUAL_EDID_FETCH
	memcpy(edid_buf, hw_context->edid_block, EDID_BLOCK_SIZE);
#else
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, offset);

	ret_val =
	    mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA,
				  EDID_BLOCK_SIZE, edid_buf);
#endif

	DUMP_EDID_BLOCK(0, edid_buf, EDID_BLOCK_SIZE);

	ret_val = ok_to_proceed_with_ddc(hw_context);
	if (!ret_val) {
		MHL_TX_DBG_INFO("No HPD ret_val:0x%02x\n", ret_val);
		return ne_NO_HPD;
	} else {
		MHL_TX_DBG_ERR("EDID block read complete. ret_val:0x%02x\n",
				ret_val);
		return ne_SUCCESS;
	}
}

int si_mhl_tx_drv_get_scratch_pad(struct drv_hw_context *hw_context,
				  uint8_t start_reg, uint8_t *data,
				  uint8_t length)
{
	if ((start_reg + length) > (int)MHL_SCRATCHPAD_SIZE)
		return -1;

	memcpy(data, &hw_context->write_burst_data[start_reg], length);

	return 0;
}

static bool packed_pixel_available(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context;
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	if (hw_context->cbus_mode >= CM_eCBUS_S) {
		if ((MHL_DEV_VID_LINK_SUPP_16BPP & DEVCAP_VAL_VID_LINK_MODE) &&
		    (dev_context->dev_cap_cache.mdc.vid_link_mode &
		     MHL_DEV_VID_LINK_SUPP_16BPP)) {

			return true;
		}
	} else {
		if ((MHL_DEV_VID_LINK_SUPP_PPIXEL & DEVCAP_VAL_VID_LINK_MODE) &&
		    (dev_context->dev_cap_cache.mdc.vid_link_mode &
		     MHL_DEV_VID_LINK_SUPP_PPIXEL)) {

			return true;
		}
	}
	return false;
}

#define SIZE_AVI_INFOFRAME 14
static uint8_t calculate_avi_info_frame_checksum(
	union hw_avi_payload_t *payload)
{
	uint8_t checksum;

	checksum = 0x82 + 0x02 + 0x0D;	
	return calculate_generic_checksum(payload->ifData, checksum,
					  SIZE_AVI_INFOFRAME);
}

static int is_valid_avi_info_frame(struct mhl_dev_context *dev_context,
				   struct avi_info_frame_t *avif)
{
	uint8_t checksum;

	checksum =
	    calculate_generic_checksum((uint8_t *) avif, 0, sizeof(*avif));
	if (0 != checksum) {
		MHL_TX_DBG_ERR("AVI info frame checksum is: 0x%02x "
			       "should be 0\n", checksum);
		return 0;

	} else if (0x82 != avif->header.type_code) {
		MHL_TX_DBG_ERR("Invalid AVI type code: 0x%02x\n",
			       avif->header.type_code);
		return 0;

	} else if (0x02 != avif->header.version_number) {
		MHL_TX_DBG_ERR("Invalid AVI version: 0x%02x\n",
			       avif->header.version_number);
		return 0;

	} else if (0x0D != avif->header.length) {
		return 0;

	} else {
		return 1;
	}
}

static int is_valid_hdmi_vsif(struct mhl_dev_context *dev_context,
			      struct vendor_specific_info_frame_t *vsif)
{
	uint8_t checksum;

	checksum = calculate_generic_checksum((uint8_t *) vsif, 0,
					      sizeof(vsif->header) +
					      vsif->header.length);
	if (0 != checksum) {
		MHL_TX_DBG_WARN("VSIF info frame checksum is: 0x%02x "
				"should be 0\n", checksum);
		checksum = calculate_generic_checksum((uint8_t *) vsif, 0,
						      sizeof(vsif->header) +
						      vsif->header.length +
						      sizeof(vsif->payLoad.
							     checksum));
		if (0 != checksum) {
			MHL_TX_DBG_ERR("VSIF info frame checksum (adjusted "
				       "for checksum itself) is: 0x%02x "
				       "should be 0\n", checksum);
			return 0;

		}
	}
	if (0x81 != vsif->header.type_code) {
		MHL_TX_DBG_ERR("Invalid VSIF type code: 0x%02x\n",
			       vsif->header.type_code);
		return 0;

	} else if (0x01 != vsif->header.version_number) {
		MHL_TX_DBG_ERR("Invalid VSIF version: 0x%02x\n",
			       vsif->header.version_number);
		return 0;

	} else {
		return 1;
	}
}

static void print_vic_modes_impl(struct drv_hw_context *hw_context,
	uint8_t vic, const char *function, int iLine)
{
	int i;
	struct vic_name {
		uint8_t vic;
		char name[15];
	} vic_name_table[] = {
		{2, "480p"},
		{4, "720p60"},
		{5, "1080i60"},
		{6, "480i"},
		{16, "1080p60"},
		{17, "576p50"},
		{19, "720p50"},
		{20, "1080i50"},
		{21, "576i50"},
		{31, "1080p50"},
		{32, "1080p24"},
		{33, "1080p25"},
		{34, "1080p30"},
		{63, "1080p120"},
		{64, "1080p100"},
		{98, "2160p24-SMPTE"},
		{93, "2160p24"},
		{94, "2160p25"},
		{95, "2160p30"},
		 
		{0, ""}
	};

#define	NUM_VIC_NAMES (sizeof(vic_name_table)/sizeof(vic_name_table[0]))
	
	for (i = 0; i < (NUM_VIC_NAMES - 1); i++) {
		if (vic == vic_name_table[i].vic)
			break;
	}
	if (vic) {
		MHL_TX_PROXY_DBG_PRINT(0, function, iLine,
			"VIC = %s%d%s (%s)\n", ANSI_ESC_GREEN_TEXT, vic,
			ANSI_ESC_RESET_TEXT, vic_name_table[i].name);
	} else {
		MHL_TX_PROXY_DBG_PRINT(0, function, iLine,
			"VIC:%s%d%s\n", ANSI_ESC_YELLOW_TEXT, vic,
			ANSI_ESC_RESET_TEXT);
	}
}
#define print_vic_modes(ctx, vic) \
	print_vic_modes_impl(ctx, vic, __func__, __LINE__)

#define DUMP_INFO_FRAMES
#ifdef DUMP_INFO_FRAMES
static void dump_avif_vsif_impl(struct drv_hw_context *hw_context,
				const char *function, int line_num)
{
	if (debug_level >= DBG_MSG_LEVEL_WARN) {
		struct vendor_specific_info_frame_t vsif;
		struct avi_info_frame_t avif;
		int i;
		unsigned char *c;
		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
			hw_context->rx_hdmi_ctrl2_defval |
			VAL_RX_HDMI_CTRL2_VSI_MON_SEL_VSI);

		mhl_tx_read_reg_block(hw_context,
			REG_RX_HDMI_MON_PKT_HEADER1, sizeof(vsif),
			(uint8_t *) &vsif);

		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
			hw_context->rx_hdmi_ctrl2_defval |
			VAL_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);

		mhl_tx_read_reg_block(hw_context,
			REG_RX_HDMI_MON_PKT_HEADER1, sizeof(avif),
			(uint8_t *) &avif);

		MHL_TX_DBG_INFO("VSIF:");
		for (i = 0, c = (unsigned char *)&vsif; i < sizeof(vsif);
			++i, ++c) {
			printk(KERN_CONT " %02x", *c);
		}
		printk("\n");
		MHL_TX_DBG_INFO("AVIF:");
		for (i = 0, c = (unsigned char *)&avif; i < sizeof(avif);
			++i, ++c) {
			printk(KERN_CONT " %02x", *c);
		}
		printk(KERN_CONT "\n");
	}
}

#define DUMP_AVIF_VSIF(hw_context) \
	dump_avif_vsif_impl(hw_context, __func__, __LINE__);
#else
#define DUMP_AVIF_VSIF(hw_context)	
#endif

enum timing_info_basis_e {
	use_avi_vic,
	use_hdmi_vic,
	use_mhl3_sequence_index,
	use_hardware_totals
};

static bool process_hdmi_vsif(struct drv_hw_context *hw_context,
	struct avi_info_frame_data_byte_4_t *p_input_video_code,
	enum timing_info_basis_e *p_timing_info_basis,
	enum mhl_vid_fmt_e *p_vid_fmt, uint32_t *p_threeDPixelClockRatio,
	uint8_t *p_fp_3d_mode, enum mhl_3d_fmt_type_e *p_3d_fmt_type)
{
	uint8_t input_VIC = (uint8_t) (*p_input_video_code).VIC;
	MHL_TX_DBG_WARN("valid HDMI VSIF\n");
	print_vic_modes(hw_context, input_VIC);
	if (0 == input_VIC) {
		if (hvfExtendedResolutionFormatPresent ==
		    hw_context->current_vs_info_frame.payLoad.pb4.
		    HDMI_Video_Format) {
			uint8_t vic =
			    hw_context->current_vs_info_frame.payLoad.pb5.
			    HDMI_VIC;
			
			MHL_TX_DBG_ERR("HDMI extended resolution %d\n", vic);
			*p_timing_info_basis = use_hdmi_vic;
		} else {
#ifdef PC_MODE_VIDEO_TIMING_SUPPORT
			MHL_TX_DBG_WARN("AVI VIC is zero!!!\n");
			*p_timing_info_basis = use_hardware_totals;
#else
			MHL_TX_DBG_ERR("AVI VIC is zero!!!\n");
			return false;
#endif
		}
	}

	if (hvf3DFormatIndicationPresent ==
	    hw_context->current_vs_info_frame.payLoad.pb4.HDMI_Video_Format) {

		*p_vid_fmt = mhl_vid_fmt_3d_fmt_present;

		MHL_TX_DBG_INFO("VSIF indicates 3D\n");
		switch (hw_context->current_vs_info_frame.payLoad.pb5.
			ThreeDStructure.threeDStructure) {
		case tdsFramePacking:
			MHL_TX_DBG_INFO("mhl_tx: tdsFramePacking\n");
			*p_threeDPixelClockRatio = 2;
			*p_fp_3d_mode |=
			    VAL_VID_OVRRD_3DCONV_EN_FRAME_PACK;
			*p_3d_fmt_type = MHL_3D_FMT_TYPE_FS;
			break;
		case tdsTopAndBottom:
			*p_3d_fmt_type = MHL_3D_FMT_TYPE_TB;
			break;
		case tdsSideBySide:
			*p_3d_fmt_type = MHL_3D_FMT_TYPE_LR;
			break;
		default:
			break;
		}
	}

	return true;
}


static uint32_t find_pixel_clock_from_closest_match_timing(
	struct mhl_dev_context *dev_context,
	struct si_incoming_timing_t *p_timing)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	uint32_t pixels_per_second = 0;
	uint32_t pixel_clock_frequency = 0;
	uint32_t pixels_per_line, lines_per_field, fields_per_second;
	uint32_t lines_per_second;
	int ret_val;

	pixel_clock_frequency = 0;

	
	
	ret_val = si_mhl_tx_drv_get_incoming_timing(hw_context, p_timing);

	pixels_per_line = (uint32_t) p_timing->h_total;
	lines_per_field = (uint32_t) p_timing->v_total;
	fields_per_second = (uint32_t) p_timing->field_rate;

	lines_per_second = lines_per_field * fields_per_second;
	pixels_per_second = pixels_per_line * lines_per_second;

	
	if (pixels_per_second) {
		p_timing->calculated_pixel_clock = 0;
		pixel_clock_frequency =
		    si_mhl_tx_find_timings_from_totals(
			dev_context->edid_parser_context, p_timing);

		MHL_TX_DBG_WARN
		    ("{%d,%d,%dx%d-%dHz, %s%d.%d%s MHz, %d.%d MHz}\n",
		     p_timing->h_total, p_timing->v_total, p_timing->columns,
		     p_timing->rows, p_timing->field_rate, ANSI_ESC_YELLOW_TEXT,
		     p_timing->calculated_pixel_clock / 1000000,
		     p_timing->calculated_pixel_clock % 1000000,
		     ANSI_ESC_RESET_TEXT, pixels_per_second / 1000000,
		     pixels_per_second % 1000000);

	}

	if (0 == pixel_clock_frequency) {
		
		MHL_TX_DBG_ERR
		    ("%s{%4d,%4d,%5d,%4d,%3d,%d,{0,0},\"%dx%d-%d\"}%s\n",
		     ANSI_ESC_YELLOW_TEXT, p_timing->h_total, p_timing->v_total,
		     p_timing->columns, p_timing->rows, p_timing->field_rate,
		     pixels_per_second, p_timing->columns, p_timing->rows,
		     p_timing->field_rate, ANSI_ESC_RESET_TEXT);
	} else {
		MHL_TX_DBG_ERR("%s%dx%d-%dHz@%d.%dMHz%s\n", ANSI_ESC_GREEN_TEXT,
			       p_timing->columns, p_timing->rows,
			       p_timing->field_rate,
			       pixel_clock_frequency / 1000000,
			       pixel_clock_frequency % 1000000,
			       ANSI_ESC_RESET_TEXT);
	}
	return pixel_clock_frequency;
}

static int set_hdmi_params(struct mhl_dev_context *dev_context)
{
	uint32_t pixel_clock_frequency;
	uint8_t tpi_output = 0;
	uint32_t threeDPixelClockRatio;
	uint8_t packedPixelNeeded = 0;
	uint8_t fp_3d_mode;
	enum AviColorSpace_e input_clr_spc = acsRGB;
	uint8_t output_clr_spc = acsRGB;
	struct avi_info_frame_data_byte_4_t input_video_code;
	struct avi_info_frame_data_byte_4_t output_video_code;
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	enum timing_info_basis_e timing_info_basis = use_avi_vic;
	
	enum mhl_vid_fmt_e vid_fmt = mhl_vid_fmt_no_additional;
	enum mhl_3d_fmt_type_e _3d_fmt_type = MHL_3D_FMT_TYPE_FS;
	enum mhl_sep_audio_e sep_aud = mhl_sep_audio_not_available;
	enum mhl_hev_fmt_e hev_fmt = mhl_hev_fmt_no_additional;
	uint16_t hev_fmt_type = 0;
	uint32_t delay_sync = 0;
	enum mhl_av_delay_dir_e delay_dir = mhl_av_delay_dir_audio_earlier;

	
	input_video_code =
	    hw_context->current_avi_info_frame.payLoad.hwPayLoad.
	    namedIfData.ifData_u.bitFields.VIC;
	threeDPixelClockRatio = 1;
	fp_3d_mode =
	    REG_VID_OVRRD_DEFVAL | BIT_VID_OVRRD_M1080P_OVRRD;

#define ENABLE_FP	VAL_VID_OVRRD_3DCONV_EN_FRAME_PACK
	
	if (hw_context->valid_vsif) {
		if (hw_context->callbacks.hpd_driven_high) {
			switch (hw_context->hpd_high_callback_status) {
			case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON:
			case HH_FMT_HDMI_VSIF_MHL3:
			case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON_NOT_RPT:
			case HH_FMT_HDMI_VSIF_MHL3_NOT_RPT:
				MHL_TX_DBG_WARN("MHL3 VSIF from callback\n");
				switch (hw_context->
					vsif_mhl3_or_hdmi_from_callback.
					mhl3_vsif.pb4 & PB4_MASK_MHL_VID_FMT) {
				case mhl_vid_fmt_no_additional:
					break;
				case mhl_vid_fmt_3d_fmt_present:
					_3d_fmt_type =
					    hw_context->
					    vsif_mhl3_or_hdmi_from_callback.
					    mhl3_vsif.
					    pb4 & PB4_MASK_MHL_3D_FMT_TYPE;
					switch (_3d_fmt_type) {
					case MHL_3D_FMT_TYPE_TB:
					case MHL_3D_FMT_TYPE_LR:
					case MHL_3D_FMT_TYPE_TBLR:
						break;
					case MHL_3D_FMT_TYPE_FS:
					case MHL_3D_FMT_TYPE_FS_TB:
					case MHL_3D_FMT_TYPE_FS_LR:
						threeDPixelClockRatio = 2;
						fp_3d_mode |= ENABLE_FP;
						break;
					}
					break;
				case mhl_vid_fmt_multi_view:
					break;
				case mhl_vid_fmt_dual_3d:
					break;
				}
				switch (hw_context->
					vsif_mhl3_or_hdmi_from_callback.
					mhl3_vsif.pb6 & PB6_MASK_MHL_HEV_FMT) {
				case mhl_hev_fmt_hev_present:
					timing_info_basis =
					    use_mhl3_sequence_index;
					break;
				case mhl_hev_fmt_no_additional:
				case mhl_hev_fmt_reserved_2:
				case mhl_hev_fmt_reserved_3:
					
					break;
				}
				break;
			case HH_FMT_HDMI_VSIF_HDMI:
			case HH_FMT_HDMI_VSIF_HDMI_HDCP_ON:
			case HH_FMT_HDMI_VSIF_HDMI_NOT_RPT:
			case HH_FMT_HDMI_VSIF_HDMI_HDCP_ON_NOT_RPT:
				MHL_TX_DBG_WARN
				    ("HDMI VSIF from callback\n");
				if (false ==
				    process_hdmi_vsif(hw_context,
						      &input_video_code,
						      &timing_info_basis,
						      &vid_fmt,
						      &threeDPixelClockRatio,
						      &fp_3d_mode,
						      &_3d_fmt_type))
					return false;
				break;
			default:
				MHL_TX_DBG_WARN
				    ("HDMI VSIF from data islands\n");
				if (false ==
				    process_hdmi_vsif(hw_context,
						      &input_video_code,
						      &timing_info_basis,
						      &vid_fmt,
						      &threeDPixelClockRatio,
						      &fp_3d_mode,
						      &_3d_fmt_type)) {
					return false;
				}
				break;
			}
		} else {
			MHL_TX_DBG_WARN("HDMI VSIF from data islands\n");
			if (false ==
			    process_hdmi_vsif(hw_context, &input_video_code,
					      &timing_info_basis, &vid_fmt,
					      &threeDPixelClockRatio,
					      &fp_3d_mode, &_3d_fmt_type)) {
				return false;
			}
		}
	} else {		
		if (0 == input_video_code.VIC) {
			DUMP_AVIF_VSIF(hw_context)
#ifdef PC_MODE_VIDEO_TIMING_SUPPORT
			    timing_info_basis = use_hardware_totals;
#else
			    MHL_TX_DBG_ERR
			    ("no VSIF and AVI VIC (offset 0x%x) is zero!!! "
			     "trying HTOTAL/VTOTAL\n",
			     (size_t) &hw_context->current_avi_info_frame.
			     payLoad.hwPayLoad.namedIfData.ifData_u.bitFields.
			     VIC -
			     (size_t) &hw_context->current_avi_info_frame);
			
			return false;
#endif
		} else {
			print_vic_modes(hw_context,
					(uint8_t) input_video_code.VIC);
		}
	}
	mhl_tx_write_reg(hw_context, REG_VID_OVRRD, fp_3d_mode);
#ifndef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
	
	hw_context->valid_vsif = 0;
#ifdef NEVER_USED
	hw_context->valid_avif = 0;
#endif
#endif

	
	hw_context->outgoingAviPayLoad =
	    hw_context->current_avi_info_frame.payLoad.hwPayLoad;
	switch (timing_info_basis) {
	case use_avi_vic:

		
		pixel_clock_frequency =
		    si_edid_find_pixel_clock_from_AVI_VIC(dev_context->
							  edid_parser_context,
							  input_video_code.VIC);
		output_video_code = input_video_code;
		break;
	case use_hdmi_vic:
		output_video_code.VIC =
		    hw_context->current_vs_info_frame.payLoad.pb5.HDMI_VIC;
		pixel_clock_frequency =
		    si_edid_find_pixel_clock_from_HDMI_VIC(dev_context->
							   edid_parser_context,
							   output_video_code.
							   VIC);
		output_video_code.VIC =
		    si_edid_map_hdmi_vic_to_mhl3_vic(dev_context->
						     edid_parser_context,
						     output_video_code.VIC);
		print_vic_modes(hw_context, output_video_code.VIC);
		break;
	case use_mhl3_sequence_index:
		output_video_code.VIC = 0;
		pixel_clock_frequency =
		    si_edid_find_pixel_clock_from_HEV_DTD(dev_context->
			edid_parser_context,
			hw_context->vsif_mhl3_or_hdmi_from_callback.
						mhl3_vsif.mhl_hev_fmt_type);
		break;
	case use_hardware_totals:
		output_video_code = input_video_code;
		{
			struct si_incoming_timing_t timing;
			
			memset((void *)&timing, 0, sizeof(timing));
			pixel_clock_frequency =
			    find_pixel_clock_from_closest_match_timing
			    (dev_context, &timing);
			if (0 == pixel_clock_frequency) {
				MHL_TX_DBG_WARN(
					"%sVIC==0 and totals not supported%s\n",
					ANSI_ESC_YELLOW_TEXT,
					ANSI_ESC_RESET_TEXT);
				mhl_tx_start_timer(dev_context,
					hw_context->
					input_field_rate_measurement_timer,
					FIELD_RATE_MEASUREMENT_INTERVAL);
				return false;
			} else {
				MHL_TX_DBG_INFO("MHL3 vic:%d\n",
						timing.mhl3_vic);
				output_video_code.VIC = timing.mhl3_vic;
				print_vic_modes(hw_context,
					output_video_code.VIC);
			}
		}
		break;
	default:
		MHL_TX_DBG_ERR("%s'shouldn't get here, the following assignment"
			       " statement exists to avoid a compiler warning%s\n",
			       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		pixel_clock_frequency = 0;
		break;
	}

	mhl_tx_stop_timer(dev_context,
			  hw_context->input_field_rate_measurement_timer);
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.VIC =
	    output_video_code;
	
	input_clr_spc =
	    hw_context->current_avi_info_frame.payLoad.hwPayLoad.
	    namedIfData.ifData_u.bitFields.pb1.colorSpace;

	MHL_TX_DBG_INFO("input_clr_spc = %02X infoData[0]:%02X\n",
			input_clr_spc,
			hw_context->current_avi_info_frame.payLoad.hwPayLoad.
			namedIfData.ifData_u.infoFrameData[0]);

	pixel_clock_frequency *= threeDPixelClockRatio;
	MHL_TX_DBG_INFO("pixel clock:%u\n", pixel_clock_frequency);

	if (qualify_pixel_clock_for_mhl(dev_context->edid_parser_context,
					pixel_clock_frequency, 24)) {
		MHL_TX_DBG_INFO("OK for 24 bit pixels\n");
	} else {
		
		if (si_edid_sink_supports_YCbCr422
		    (dev_context->edid_parser_context)) {
			MHL_TX_DBG_INFO("Sink supports YCbCr422\n");

			if (qualify_pixel_clock_for_mhl
			    (dev_context->edid_parser_context,
			     pixel_clock_frequency, 16)) {
				
				packedPixelNeeded = 1;
			} else {
				MHL_TX_DBG_ERR("unsupported video mode."
					"pixel clock too high %s\n",
					si_peer_supports_packed_pixel
					(dev_context) ? "" :
					"(peer does not support packed pixel)."
					);
				return false;
			}
		} else {
			MHL_TX_DBG_ERR("unsupported video mode."
				       "Sink doesn't support 4:2:2.\n");
			return false;
		}
	}

	
	output_clr_spc = input_clr_spc;
#define MHL3_PACKED_PIXEL_MODE	VAL_M3_P0CTRL_MHL3_P0_PIXEL_MODE_PACKED
	switch (hw_context->pp_16bpp_override) {
	case pp_16bpp_automatic:
	case pp_16bpp_override_24bpp:
		break;
	case pp_16bpp_override_16bpp:
		packedPixelNeeded = 1;
		break;
	}

	if (packedPixelNeeded) {
		if (packed_pixel_available(dev_context)) {

			dev_context->link_mode =
			    MHL_STATUS_PATH_ENABLED |
			    MHL_STATUS_CLK_MODE_PACKED_PIXEL;
			
			output_clr_spc = BIT_EDID_FIELD_FORMAT_YCbCr422;

#if 0
			tpi_output |= BIT_TPI_OUTPUT_CSCMODE709;
#if 0
			if (hw_context->
				current_avi_info_frame.payLoad.hwPayLoad.
				namedIfData.ifData_u.bitFields.pb3.
				RGBQuantizationRange) {
				
			} else if (acsRGB == hw_context->current_avi_info_frame.
				payLoad.hwPayLoad.namedIfData.ifData_u.
				bitFields.pb1.colorSpace) {
				tpi_output |= 0x04; 
				hw_context->outgoingAviPayLoad.namedIfData.
					ifData_u.bitFields.pb3.
					RGBQuantizationRange = 2;
			}
#endif

			
			hw_context->outgoingAviPayLoad.namedIfData.
				ifData_u.bitFields.
				colorimetryAspectRatio.Colorimetry = 3;
			
			hw_context->outgoingAviPayLoad.namedIfData.
				ifData_u.bitFields.pb3.ExtendedColorimetry = 1;
#endif

			if (IN_MHL3_MODE(hw_context)) {
				MHL_TX_DBG_INFO("setting 16BPP mode\n");
				mhl_tx_modify_reg(hw_context,
					REG_M3_P0CTRL,
					BIT_M3_P0CTRL_MHL3_P0_PIXEL_MODE,
					MHL3_PACKED_PIXEL_MODE);
			} else {
				MHL_TX_DBG_INFO("setting packed pixel mode\n");
				mhl_tx_write_reg(hw_context,
					REG_VID_MODE,
					VAL_VID_MODE_M1080P_ENABLE);
				mhl_tx_write_reg(hw_context,
						 REG_MHL_TOP_CTL, 0x41);

				mhl_tx_write_reg(hw_context,
						 REG_MHLTX_CTL6, 0x60);
			}

		} else {
			MHL_TX_DBG_ERR
			    ("unsupported video mode. Packed Pixel not "
			     "available on sink. Sink's link mode = 0x%02x\n",
			     dev_context->dev_cap_cache.mdc.vid_link_mode);
			return false;
		}
	} else {

		dev_context->link_mode =
			MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_NORMAL;

		if (IN_MHL3_MODE(hw_context)) {
			MHL_TX_DBG_INFO("setting 24BPP Mode\n");
			mhl_tx_modify_reg(hw_context, REG_M3_P0CTRL,
				BIT_M3_P0CTRL_MHL3_P0_PIXEL_MODE,
				VAL_M3_P0CTRL_MHL3_P0_PIXEL_MODE_NORMAL);
		} else {
			MHL_TX_DBG_INFO(
				"normal Mode, Packed Pixel mode disabled\n");
			mhl_tx_write_reg(hw_context, REG_VID_MODE,
				VAL_VID_MODE_M1080P_DISABLE);
			mhl_tx_write_reg(hw_context, REG_MHL_TOP_CTL,
				0x01);
			mhl_tx_write_reg(hw_context, REG_MHLTX_CTL6,
				0xA0);
		}
	}

	
	mhl_tx_write_reg(hw_context, REG_TPI_INPUT,
		colorSpaceTranslateInfoFrameToHw[input_clr_spc]);

	
	tpi_output |= colorSpaceTranslateInfoFrameToHw[output_clr_spc];

	mhl_tx_write_reg(hw_context, REG_TPI_OUTPUT, tpi_output);

	if (IN_MHL3_MODE(hw_context)) {
		struct MHL_bits_per_pixel_fmt_data_t bpp_fmt;
		struct MHL_bits_per_pixel_fmt_data_t *p_buffer;
		size_t xfer_size;
		
		xfer_size = sizeof(bpp_fmt) - sizeof(p_buffer->descriptors) +
			sizeof(p_buffer->descriptors[0]);
		p_buffer =
			si_mhl_tx_get_sub_payload_buffer(dev_context,
			xfer_size);

		if (p_buffer) {
			p_buffer->header.burst_id.low =
				LOW_BYTE_16(burst_id_BITS_PER_PIXEL_FMT);
			p_buffer->header.burst_id.high =
				HIGH_BYTE_16(burst_id_BITS_PER_PIXEL_FMT);
			p_buffer->header.checksum = 0;
			p_buffer->header.total_entries = 1;
			p_buffer->header.sequence_index = 1;
			p_buffer->num_entries_this_burst = 1;
			p_buffer->descriptors[0].stream_id = 0;
			switch (dev_context->
				link_mode & MHL_STATUS_CLK_MODE_MASK) {
			case MHL_STATUS_CLK_MODE_PACKED_PIXEL:
				p_buffer->descriptors[0].stream_pixel_format =
					VIEW_PIX_FMT_16BPP;
				break;
			case MHL_STATUS_CLK_MODE_NORMAL:
				p_buffer->descriptors[0].stream_pixel_format =
					VIEW_PIX_FMT_24BPP;
				break;
			}
			p_buffer->header.checksum =
				calculate_generic_checksum(p_buffer, 0,
				xfer_size);
			si_mhl_tx_push_block_transactions(dev_context);
		}
		
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xF0);

		
		hw_context->outgoing_mhl3_vsif.header.type_code =
		    MHL3_VSIF_TYPE;
		hw_context->outgoing_mhl3_vsif.header.version_number =
		    MHL3_VSIF_VERSION;
		hw_context->outgoing_mhl3_vsif.header.length =
		    sizeof(hw_context->outgoing_mhl3_vsif);
		hw_context->outgoing_mhl3_vsif.checksum = 0;
		hw_context->outgoing_mhl3_vsif.iee_oui[0] =
		    (uint8_t) (IEEE_OUI_MHL3 & 0xFF);
		hw_context->outgoing_mhl3_vsif.iee_oui[1] =
		    (uint8_t) ((IEEE_OUI_MHL3 >> 8) & 0xFF);
		hw_context->outgoing_mhl3_vsif.iee_oui[2] =
		    (uint8_t) ((IEEE_OUI_MHL3 >> 16) & 0xFF);
		hw_context->outgoing_mhl3_vsif.pb4 =
		    MHL3_VSIF_PB4(vid_fmt, _3d_fmt_type, sep_aud);
		hw_context->outgoing_mhl3_vsif.pb5_reserved = 0;
		hw_context->outgoing_mhl3_vsif.pb6 = MHL3_VSIF_PB6(hev_fmt);
		hw_context->outgoing_mhl3_vsif.mhl_hev_fmt_type.high =
		    HIGH_BYTE_16(hev_fmt_type);
		hw_context->outgoing_mhl3_vsif.mhl_hev_fmt_type.low =
		    LOW_BYTE_16(hev_fmt_type);
		hw_context->outgoing_mhl3_vsif.pb9 =
		    MHL3_VSIF_PB9(delay_sync, delay_dir);
		hw_context->outgoing_mhl3_vsif.av_delay_sync.high =
		    HIGH_BYTE_16(delay_sync);
		hw_context->outgoing_mhl3_vsif.av_delay_sync.low =
		    LOW_BYTE_16(delay_sync);

		hw_context->outgoing_mhl3_vsif.checksum =
			calculate_generic_checksum(
				&hw_context->outgoing_mhl3_vsif,
				0, hw_context->outgoing_mhl3_vsif.header.
				length);

		switch (dev_context->link_mode & MHL_STATUS_CLK_MODE_MASK) {
		case MHL_STATUS_CLK_MODE_PACKED_PIXEL:
			si_mhl_tx_drv_set_lowest_tmds_link_speed(dev_context,
				pixel_clock_frequency, 16);
			break;
		case MHL_STATUS_CLK_MODE_NORMAL:
			si_mhl_tx_drv_set_lowest_tmds_link_speed(dev_context,
				pixel_clock_frequency, 24);
			break;
		}
	} else {
		si_mhl_tx_set_status(dev_context, false,
			MHL_STATUS_REG_LINK_MODE, dev_context->link_mode);
	}
	hw_context->outgoingAviPayLoad.namedIfData.checksum = 0;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.pb1.
		colorSpace = output_clr_spc;
	hw_context->outgoingAviPayLoad.ifData[1] &= 0x7F; 
	hw_context->outgoingAviPayLoad.ifData[4] &= 0x7F; 
	hw_context->outgoingAviPayLoad.namedIfData.checksum =
		calculate_avi_info_frame_checksum(
			&hw_context->outgoingAviPayLoad);

	return true;
}


void process_info_frame_change(struct drv_hw_context *hw_context,
	struct vendor_specific_info_frame_t *hdmi_vsif,
	struct avi_info_frame_t *avif)
{
	bool mode_change = false;
	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context, struct mhl_dev_context,
				   drv_context);
	if (NULL != hdmi_vsif) {
		if (is_valid_hdmi_vsif(dev_context, hdmi_vsif)) {
			hw_context->current_vs_info_frame = *hdmi_vsif;
			hw_context->valid_vsif = 1;
			mode_change = true;
		}
	}
	if (NULL != avif) {
		if (is_valid_avi_info_frame(dev_context, avif)) {
			hw_context->current_avi_info_frame = *avif;
#ifdef NEVER_USED
			hw_context->valid_avif = 1;
#endif
			mode_change = true;
		}
	}
	if (mode_change) {
		int cstat_p3;
		int bits_of_interest;
		cstat_p3 =
		    mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);
		bits_of_interest =
		    cstat_p3 & (BIT_TMDS_CSTAT_P3_SCDT |
				BIT_TMDS_CSTAT_P3_CKDT);

		if ((BIT_TMDS_CSTAT_P3_SCDT |
		     VAL_TMDS_CSTAT_P3_CKDT_DETECTED)
		    == bits_of_interest) {
			start_video(hw_context);
		}
	}
}

#define dump_edid_fifo(hw_context, block_number)	

#define RX_DPD_BITS (BIT_DPD_PDNRX12 \
					  | BIT_DPD_PDIDCK_N \
					  | BIT_DPD_PD_MHL_CLK_N)
static int init_rx_regs(struct drv_hw_context *hw_context)
{
	
	mhl_tx_modify_reg(hw_context, REG_DPD, RX_DPD_BITS, RX_DPD_BITS);

	
	mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL3, 0x00);

	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_0, 0xFF);
	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1, 0xFF);

	mhl_tx_write_reg(hw_context, REG_ALICE0_BW_I2C, 0x06);

	mhl_tx_modify_reg(hw_context, REG_RX_HDMI_CLR_BUFFER,
			  BIT_RX_HDMI_CLR_BUFFER_VSI_CLR_EN,
			  VAL_RX_HDMI_CLR_BUFFER_VSI_CLR_EN_CLEAR);

	return 0;
}

#ifdef USE_HW_TIMER
static void start_hw_timer(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Start HW Timer.\n");
	mhl_tx_write_reg(hw_context, REG_SYS_CTRL3,
			 BIT_SYS_CTRL3_SYS_CNTR);
}

static void stop_hw_timer(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Stop HW Timer.\n");
	mhl_tx_write_reg(hw_context, REG_SYS_CTRL3, 0x00);
}

static void setup_hw_timer(struct drv_hw_context *hw_context, uint16_t time_ms)
{
	MHL_TX_DBG_INFO("Setup HW Timer for %dms.\n", time_ms);

	
	time_ms &= 0x7FFF;

	
	time_ms <<= 1;

	mhl_tx_write_reg(hw_context, REG_SYS_CNTR_0,
			 (uint8_t) (time_ms & 0x00FF));
	mhl_tx_write_reg(hw_context, REG_SYS_CNTR_1,
			 (uint8_t) ((time_ms >> 8) & 0x00FF));
}
#endif

int si_mhl_tx_drv_set_display_mode(struct mhl_dev_context *dev_context,
	enum hpd_high_callback_status status)
{
	struct drv_hw_context *hw_context;
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	hw_context->hpd_high_callback_status = status;
#ifdef NEVER_USED
	hw_context->valid_avif = true;
#endif
	
	if (status >= 0) {
		switch (status) {
		case HH_FMT_DVI_HDCP_ON:
		case HH_FMT_DVI:
		case HH_FMT_DVI_HDCP_ON_NOT_RPT:
		case HH_FMT_DVI_NOT_RPT:
#ifdef NEVER_USED
			hw_context->valid_avif = false;
#endif
			start_video(hw_context);
			break;
		case HH_FMT_HDMI_VSIF_NONE_HDCP_ON:
		case HH_FMT_HDMI_VSIF_NONE:
		case HH_FMT_HDMI_VSIF_NONE_HDCP_ON_NOT_RPT:
		case HH_FMT_HDMI_VSIF_NONE_NOT_RPT:
			process_info_frame_change(hw_context, NULL,
						  &hw_context->
						  avif_or_dtd_from_callback.
						  avif);
			break;

		case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON:
		case HH_FMT_HDMI_VSIF_MHL3:
		case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON_NOT_RPT:
		case HH_FMT_HDMI_VSIF_MHL3_NOT_RPT:
#ifdef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
			hw_context->valid_vsif = true;
#endif
			
			process_info_frame_change(hw_context, NULL,
						  &hw_context->
						  avif_or_dtd_from_callback.
						  avif);
			break;
		case HH_FMT_HDMI_VSIF_HDMI_HDCP_ON:
		case HH_FMT_HDMI_VSIF_HDMI:
		case HH_FMT_HDMI_VSIF_HDMI_HDCP_ON_NOT_RPT:
		case HH_FMT_HDMI_VSIF_HDMI_NOT_RPT:
#ifdef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
			hw_context->valid_vsif = true;
#endif
			process_info_frame_change(hw_context,
				&hw_context->vsif_mhl3_or_hdmi_from_callback.
					hdmi_vsif,
				&hw_context->avif_or_dtd_from_callback.
					avif);
			break;
		default:
			
			break;
		}
	}
	return 0;
}

#define BIT_0_DISABLED	0x01
#define BIT_0_HIGH		0x02
#define BIT_1_DISABLED	0x04
#define BIT_1_HIGH		0x08

#define BITS_GPIO_01_HPD_HIGH	(BIT_0_HIGH | BIT_1_HIGH)
#define BITS_GPIO_01_HPD_LOW	0

#define BITS_HPD_CTRL_OPEN_DRAIN_HIGH (BITS_GPIO_01_HPD_HIGH | 0x70)
#define BITS_HPD_CTRL_PUSH_PULL_HIGH (BITS_GPIO_01_HPD_HIGH | 0x30)

#define BITS_HPD_CTRL_OPEN_DRAIN_LOW (BITS_GPIO_01_HPD_LOW | 0x50)
#define BITS_HPD_CTRL_PUSH_PULL_LOW (BITS_GPIO_01_HPD_LOW | 0x10)
static int drive_hpd_high(struct drv_hw_context *hw_context, uint8_t * edid,
			  uint16_t length)
{
	enum hpd_control_mode mode;
	int ret_val = -1;
	int cstat_p3;

	mode = platform_get_hpd_control_mode();

	
	cstat_p3 = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);

	
	cstat_p3 |= BIT_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR;
#ifdef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
	cstat_p3 |= BIT_TMDS_CSTAT_P3_AVIF_MANUAL_CLEAR_STROBE;
#endif
	mhl_tx_write_reg(hw_context, REG_TMDS_CSTAT_P3, cstat_p3);

	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val =
		    mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				     BITS_HPD_CTRL_OPEN_DRAIN_HIGH);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val =
		    mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				     BITS_HPD_CTRL_PUSH_PULL_HIGH);

	if (edid) {
		if (hw_context->callbacks.hpd_driven_high) {
			int status;
			struct mhl_dev_context *dev_context =
			    container_of((void *)hw_context,
					 struct mhl_dev_context, drv_context);

			struct edid_3d_data_t *p_edid_data =
			    dev_context->edid_parser_context;

			status =
			    hw_context->callbacks.hpd_driven_high(
					hw_context->callbacks.context,
					edid,
					length,
					p_edid_data->hev_dtd_list,
					p_edid_data->hev_dtd_info.num_items,
					p_edid_data->hev_vic_list,
					p_edid_data->hev_vic_info.num_items,
					p_edid_data->_3d_dtd_list,
					p_edid_data->_3d_dtd_info.num_items,
					p_edid_data->_3d_vic_list,
					p_edid_data->_3d_vic_info.num_items,
					&hw_context->avif_or_dtd_from_callback,
					sizeof(hw_context->
						avif_or_dtd_from_callback),
					&hw_context->
						vsif_mhl3_or_hdmi_from_callback,
					sizeof(hw_context->
						vsif_mhl3_or_hdmi_from_callback)
					);
			si_mhl_tx_drv_set_display_mode(dev_context, status);
		}
	}
	if (ret_val >= 0)
		return cstat_p3;

	return ret_val;
}

static int drive_hpd_low(struct drv_hw_context *hw_context)
{
	enum hpd_control_mode mode;
	int ret_val = -1;

	ddc_abort_count = 0;
	mode = platform_get_hpd_control_mode();

	mhl_tx_modify_reg(hw_context, REG_EDID_CTRL,
			  BIT_EDID_CTRL_EDID_PRIME_VALID,
			  VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE);

	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val =
		    mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				     BITS_HPD_CTRL_OPEN_DRAIN_LOW);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val =
		    mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				     BITS_HPD_CTRL_PUSH_PULL_LOW);

	enable_intr(hw_context, INTR_INFR, 0x00);
	hw_context->hpd_high_callback_status = HH_VIDEO_NOT_RDY;
	if (hw_context->callbacks.hpd_driven_low) {
		hw_context->callbacks.hpd_driven_low(hw_context->callbacks.
						     context);
	}
	return ret_val;
}

#ifdef SWWA_BZ30759
void edid_hw_sm_clean_up(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_PAGE_8_HDCP1X_LB_BIST,
			 BIT_PAGE_8_HDCP1X_LB_BIST_HDCP1X_LB_BIST_EN);
	mhl_tx_write_reg(hw_context, REG_DDC_MANUAL,
			 BIT_DDC_MANUAL_MAN_DDC);
	mhl_tx_write_reg(hw_context, REG_INTR9, 0xFF);

	
	enable_intr(hw_context, INTR_EDID, 0);

	
	mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
			 BIT_TPI_CBUS_START_GET_EDID_START_0);

	mhl_tx_write_reg(hw_context, REG_INTR9, 0xFF);
	mhl_tx_write_reg(hw_context, REG_DDC_MANUAL, 0x00);
	mhl_tx_write_reg(hw_context, REG_PAGE_8_HDCP1X_LB_BIST, 0x00);

}
#endif

int si_mhl_tx_drv_set_upstream_edid(struct drv_hw_context *hw_context,
				    uint8_t *edid, uint16_t length)
{
	uint8_t reg_val;

	if (MHL_SEND_3D_REQ_OR_FEAT_REQ ==
		hw_context->current_cbus_req.command) {
		struct mhl_dev_context *dev_context;

		dev_context = container_of((void *)hw_context,
					  struct mhl_dev_context, drv_context);
		MHL_TX_DBG_WARN("3D_REQ or FEAT_REQ completed\n");
		hw_context->current_cbus_req.command = 0x00;
		si_mhl_tx_msc_command_done(dev_context, 0x00);
	}

	if (!ok_to_proceed_with_ddc(hw_context))
		return -1;

	MHL_TX_DBG_INFO("presenting EDID upstream\n");

	init_rx_regs(hw_context);

#ifdef SWWA_BZ30759
	edid_hw_sm_clean_up(hw_context);
#endif
	
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
			 VAL_EDID_CTRL_DEVCAP_SELECT_EDID |
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE |
			 VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);

	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);
	mhl_tx_write_reg_block(hw_context, REG_EDID_FIFO_WR_DATA, length,
			       edid);

	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_PRIME_VALID_ENABLE |
			 VAL_EDID_CTRL_DEVCAP_SELECT_EDID |
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE |
			 VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);

	
	enable_intr(hw_context, INTR_SCDT, BIT_INTR_SCDT_CHANGE);

	
	enable_intr(hw_context, INTR_EDID, 0);

#ifndef EARLY_HSIC
	if (IN_MHL3_MODE(hw_context)) {
		hsic_init(hw_context);
	}
#endif
	MHL_TX_DBG_ERR("Expose EDID\n");

	
	reg_val = drive_hpd_high(hw_context, edid, length);

	
	if (BIT_TMDS_CSTAT_P3_SCDT & reg_val) {
		MHL_TX_DBG_INFO("SCDT status is already HIGH. "
		     "Simulate int_5: %s0x%02x%s\n",
		     ANSI_ESC_YELLOW_TEXT,
		     mhl_tx_read_reg(hw_context, REG_RX_HDMI_CTRL0),
		     ANSI_ESC_RESET_TEXT);
		int_5_isr(hw_context, BIT_INTR_SCDT_CHANGE);
	}
	return 0;
}

static void hsic_init(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Initialize USB Tunneling\n");

	
	mhl_tx_write_reg(hw_context, REG_FCGC, 0x03);

	mhl_tx_modify_reg(hw_context, REG_HRXCTRL3,
			  BIT_HRXCTRL3_HRX_STAY_RESET |
			  BIT_HRXCTRL3_STATUS_EN,
			  BIT_HRXCTRL3_HRX_STAY_RESET |
			  BIT_HRXCTRL3_STATUS_EN);

	
	mhl_tx_modify_reg(hw_context, REG_TTXNUMB,
			  MSK_TTXNUMB_TTX_NUMBPS_2_0, 4);

	
	mhl_tx_modify_reg(hw_context, REG_TRXCTRL, 0x08, 0x08);

	
	mhl_tx_modify_reg(hw_context, REG_HTXCTRL, 0x02, 0x00);

	
	mhl_tx_modify_reg(hw_context, REG_KEEPER, 0x03, 0x00);

	mhl_tx_write_reg(hw_context, REG_TDMLLCTL, 0x00);

	
	mhl_tx_write_reg(hw_context, REG_UTSRST,
			 BIT_UTSRST_HRX_SRST |
			 BIT_UTSRST_HTX_SRST |
			 BIT_UTSRST_KEEPER_SRST |
			 BIT_UTSRST_FC_SRST);
	mhl_tx_write_reg(hw_context, REG_UTSRST,
			 BIT_UTSRST_HRX_SRST |
			 BIT_UTSRST_HTX_SRST);

	mhl_tx_write_reg(hw_context, REG_HRXINTL, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_HRXINTH, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_TTXINTL, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_TTXINTH, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_TRXINTL, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_TRXINTH, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_HTXINTL, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_HTXINTH, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR0, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR1, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR2, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR3, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR4, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR5, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR6, 0xFF);	
	mhl_tx_write_reg(hw_context, REG_FCINTR7, 0xFF);	
}

#define MHL_LOGICAL_DEVICE_MAP (MHL_DEV_LD_AUDIO | MHL_DEV_LD_VIDEO | \
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_GUI)
#define DEVCAP_REG(x) (REG_MHL_DEVCAP_0 | DEVCAP_OFFSET_##x)
#define XDEVCAP_REG(x) (REG_MHL_EXTDEVCAP_0 | \
		XDEVCAP_OFFSET(XDEVCAP_ADDR_##x))

uint8_t dev_cap_values[16] = {
	DEVCAP_VAL_DEV_STATE,
	DEVCAP_VAL_MHL_VERSION,
	DEVCAP_VAL_DEV_CAT,
	DEVCAP_VAL_ADOPTER_ID_H,
	DEVCAP_VAL_ADOPTER_ID_L,
	DEVCAP_VAL_VID_LINK_MODE,
	DEVCAP_VAL_AUD_LINK_MODE,
	DEVCAP_VAL_VIDEO_TYPE,
	DEVCAP_VAL_LOG_DEV_MAP,
	DEVCAP_VAL_BANDWIDTH,
	DEVCAP_VAL_FEATURE_FLAG,
	0,
	0,
	DEVCAP_VAL_SCRATCHPAD_SIZE,
	DEVCAP_VAL_INT_STAT_SIZE,
	DEVCAP_VAL_RESERVED
};

uint8_t xdev_cap_values[4] = {
	XDEVCAP_VAL_ECBUS_SPEEDS,
	XDEVCAP_VAL_TMDS_SPEEDS,
	XDEVCAP_VAL_DEV_ROLES,
	XDEVCAP_VAL_LOG_DEV_MAPX
};

static void peer_specific_init(struct drv_hw_context *hw_context)
{
	if (IN_MHL3_MODE(hw_context)) {
		
		mhl_tx_write_reg(hw_context, REG_SYS_CTRL1,
			BIT_SYS_CTRL1_BLOCK_DDC_BY_HPD);
	} else {
		
		enable_intr(hw_context, INTR_HDCP2, 0x00);
		enable_intr(hw_context, INTR_LINK_TRAINING, 0x00);
		mhl_tx_write_reg(hw_context, REG_HDCP2X_INTR0, 0xFF);
		mhl_tx_write_reg(hw_context, REG_INTR1, 0xFF);

		mhl_tx_write_reg(hw_context, REG_SYS_CTRL1,
			BIT_SYS_CTRL1_BLOCK_DDC_BY_HPD |
			BIT_SYS_CTRL1_TX_CONTROL_HDMI);
	}
}

static int init_regs(struct drv_hw_context *hw_context)
{
	int ret_val = 0;

	MHL_TX_DBG_INFO("called\n");
	peer_specific_init(hw_context);

	
	hw_context->video_ready = false;
	hw_context->video_path = 1;

	hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_DVI;
	
	drive_hpd_low(hw_context);
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE);

	
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL9,
			 BIT_DISC_CTRL9_WAKE_DRVFLT |
			 BIT_DISC_CTRL9_WAKE_PULSE_BYPASS);

	mhl_tx_write_reg(hw_context, REG_TMDS0_CCTRL1, 0x90);

	
	mhl_tx_write_reg(hw_context, REG_TMDS_CLK_EN, 0x01);

	
	mhl_tx_write_reg(hw_context, REG_TMDS_CH_EN, 0x11);
	mhl_tx_write_reg(hw_context, REG_BGR_BIAS, 0x87);

	mhl_tx_write_reg(hw_context, REG_ALICE0_ZONE_CTRL, 0xE8);
	mhl_tx_write_reg(hw_context, REG_ALICE0_MODE_CTRL, 0x04);

	
	ret_val = mhl_tx_read_reg(hw_context, REG_LM_DDC);
	ret_val &= ~BIT_LM_DDC_SW_TPI_EN;
	ret_val |= VAL_LM_DDC_SW_TPI_EN_ENABLED;
	mhl_tx_write_reg(hw_context, REG_LM_DDC, ret_val);


	mhl_tx_write_reg(hw_context, REG_TPI_HW_OPT3, 0x76);

	mhl_tx_write_reg(hw_context, REG_TMDS_CCTRL,
			 BIT_TMDS_CCTRL_TMDS_OE);

	
	mhl_tx_write_reg(hw_context, REG_TPI_DTD_B2, 79);

	

	dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_L] =
	    (uint8_t) hw_context->chip_device_id;
	dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_H] =
	    (uint8_t) (hw_context->chip_device_id >> 8);

	
	mhl_tx_write_reg_block(hw_context, DEVCAP_REG(DEV_STATE),
			       ARRAY_SIZE(dev_cap_values), dev_cap_values);

	xdev_cap_values[XDEVCAP_OFFSET(XDEVCAP_ADDR_ECBUS_SPEEDS)] =
	    MHL_XDC_ECBUS_S_075 | MHL_XDC_ECBUS_S_8BIT;
	if (si_mhl_tx_drv_support_e_cbus_d(hw_context))
		xdev_cap_values[XDEVCAP_OFFSET(XDEVCAP_ADDR_ECBUS_SPEEDS)] |=
		    MHL_XDC_ECBUS_D_150 | MHL_XDC_ECBUS_D_8BIT;

	mhl_tx_write_reg_block(hw_context, XDEVCAP_REG(ECBUS_SPEEDS),
			       ARRAY_SIZE(xdev_cap_values), xdev_cap_values);

	mhl_tx_write_reg(hw_context, REG_MDT_XMIT_TIMEOUT, 100);

	
	mhl_tx_write_reg(hw_context, REG_MDT_XMIT_CONTROL, 0x03);

	
	mhl_tx_write_reg(hw_context, REG_MDT_XFIFO_STAT, 0x00);

	mhl_tx_write_reg(hw_context, REG_MDT_RCV_TIMEOUT, 100);

	
	mhl_tx_write_reg(hw_context, REG_CBUS_LINK_CONTROL_8, 0x1D);

	si_mhl_tx_drv_start_gen2_write_burst(hw_context);

	mhl_tx_write_reg(hw_context, REG_BIST_CTRL, 0x00);

	
	mhl_tx_write_reg(hw_context, REG_COC_CTL1, 0x10);
	mhl_tx_write_reg(hw_context, REG_COC_CTL2, 0x18);
	mhl_tx_write_reg(hw_context, REG_COC_CTLF, 0x07);
	mhl_tx_write_reg(hw_context, REG_COC_CTL11, 0xF8);
	mhl_tx_write_reg(hw_context, REG_COC_CTL17, 0x61);
	mhl_tx_write_reg(hw_context, REG_COC_CTL18, 0x46);
	mhl_tx_write_reg(hw_context, REG_COC_CTL19, 0x15);
	mhl_tx_write_reg(hw_context, REG_COC_CTL1A, 0x01);
	mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL3,
					BIT_MHL_COC_CTL3_COC_AECHO_EN);
	mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL4, 0x2D);
	mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL5, 0xF9);

	disable_heartbeat(hw_context);
	disable_gen2_write_burst_rcv(hw_context);
	disable_gen2_write_burst_xmit(hw_context);

	return ret_val;
}

static void si_mhl_tx_drv_start_gen2_write_burst(struct drv_hw_context
						 *hw_context)
{
	enable_intr(hw_context, INTR_G2WB_ERR,
		    BIT_MDT_RCV_TIMEOUT | BIT_MDT_RCV_SM_ABORT_PKT_RCVD |
		    BIT_MDT_RCV_SM_ERROR | BIT_MDT_XMIT_TIMEOUT |
		    BIT_MDT_XMIT_SM_ABORT_PKT_RCVD | BIT_MDT_XMIT_SM_ERROR);
	enable_intr(hw_context, INTR_G2WB,
		    BIT_MDT_XFIFO_EMPTY | BIT_MDT_IDLE_AFTER_HAWB_DISABLE |
		    BIT_MDT_RFIFO_DATA_RDY);

	hw_context->cbus1_state = CBUS1_IDLE_RCV_ENABLED;
	hw_context->delayed_hawb_enable_reg_val = 0;
	enable_gen2_write_burst_rcv(hw_context);
}

void si_mhl_tx_drv_disable_video_path(struct drv_hw_context *hw_context)
{
	
	if (hw_context->video_ready
	    && (0 ==
		(VAL_TPI_SC_TPI_AV_MUTE_MUTED &
		 mhl_tx_read_reg(hw_context, REG_TPI_SC)))) {

		
		stop_video(hw_context);
		hw_context->video_path = 0;
	}
}

void si_mhl_tx_drv_enable_video_path(struct drv_hw_context *hw_context)
{
	uint8_t mask =
	    (BIT_TPI_SC_REG_TMDS_OE | BIT_TPI_SC_TPI_AV_MUTE);
	uint8_t reg;

	if (hw_context->video_ready && (0 == hw_context->video_path)) {
		
		hw_context->video_path = 1;

		reg = mhl_tx_read_reg(hw_context, REG_TPI_SC);

		if (mask == (mask & reg))
			start_video(hw_context);
	}
}

void si_mhl_tx_drv_content_off(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("RAP CONTENT_OFF video %sready\n",
			hw_context->video_ready ? "" : "NOT ");
	
	if (hw_context->video_ready
	    && (0 ==
		(VAL_TPI_SC_TPI_AV_MUTE_MUTED &
		 mhl_tx_read_reg(hw_context, REG_TPI_SC)))) {

		MHL_TX_DBG_INFO("RAP CONTENT_OFF\n");
		
		stop_video(hw_context);
	}
}

void si_mhl_tx_drv_content_on(struct drv_hw_context *hw_context)
{
	uint8_t mask =
	    (BIT_TPI_SC_REG_TMDS_OE | BIT_TPI_SC_TPI_AV_MUTE);
	uint8_t reg;

	if (hw_context->video_ready) {

		reg = mhl_tx_read_reg(hw_context, REG_TPI_SC);

		if (mask == (mask & reg))
			start_video(hw_context);
	}
}

static void unmute_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("AV unmuted\n");

	if (!IN_MHL3_MODE(hw_context)) {
		if (hdcp_content_type == 0) {
			if (si_edid_sink_is_hdmi
			    (hw_context->intr_info->edid_parser_context)) {
				mhl_tx_write_reg(hw_context, REG_TPI_SC,
				    VAL_TPI_SC_TPI_OUTPUT_MODE_0_HDMI);
			} else {
				mhl_tx_write_reg(hw_context, REG_TPI_SC,
				    VAL_TPI_SC_TPI_OUTPUT_MODE_0_DVI);
			}
		} else {
			MHL_TX_DBG_INFO
			    ("HDCP Content Type 1, AV remain muted\n");
		}
	}

	
	hw_context->video_ready = 1;
}

#define OUTPUT_OFF_HDMI	(VAL_TPI_SC_REG_TMDS_OE_POWER_DOWN | \
			VAL_TPI_SC_TPI_AV_MUTE_MUTED | \
			VAL_TPI_SC_TPI_OUTPUT_MODE_0_HDMI)

#define OUTPUT_OFF_DVI	(VAL_TPI_SC_REG_TMDS_OE_POWER_DOWN | \
			VAL_TPI_SC_TPI_AV_MUTE_MUTED | \
			VAL_TPI_SC_TPI_OUTPUT_MODE_0_DVI)

static void stop_video(struct drv_hw_context *hw_context)
{
	if (IN_MHL3_MODE(hw_context)) {

		MHL_TX_DBG_WARN("for >= MHL3.0\n");

		
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xC0);

		
		mhl_tx_write_reg(hw_context, REG_M3_CTRL,
				 VAL_M3_CTRL_MHL3_VALUE |
				 BIT_M3_CTRL_H2M_SWRST);

		si_mhl_tx_drv_shut_down_HDCP2(hw_context);

	} else {

		
		MHL_TX_DBG_WARN(" for MHL1/2.x\n");

		
		enable_intr(hw_context, INTR_HDCP, 0x00);
		enable_intr(hw_context, INTR_HDCP2, 0x00);

		
		mhl_tx_write_reg(hw_context, REG_TPI_COPP_DATA2, 0);

		
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_addr,
			0xFF);

		MHL_TX_DBG_WARN("hw_context = %p; ->intr_info = %p\n",
				hw_context, hw_context->intr_info);

		if (NULL != hw_context->intr_info) {
			if (NULL !=
				hw_context->intr_info->edid_parser_context) {
				if (si_edid_sink_is_hdmi(
				    hw_context->intr_info->
				    edid_parser_context)) {

					mhl_tx_write_reg(hw_context,
						 REG_TPI_SC,
						 OUTPUT_OFF_HDMI);
				} else {
					mhl_tx_write_reg(hw_context,
						 REG_TPI_SC,
						 OUTPUT_OFF_DVI);
				}
			}
		}
	}
}

void si_mhl_tx_drv_start_cp(struct drv_hw_context *hw_context)
{
	if (hw_context->hdcp2_started) {
		MHL_TX_DBG_ERR("%salready started%s\n", ANSI_ESC_RED_TEXT,
		ANSI_ESC_RESET_TEXT);
		si_mhl_tx_drv_shut_down_HDCP2(hw_context);
	} else {
		start_hdcp(hw_context);
	}
}

static void start_hdcp(struct drv_hw_context *hw_context)
{
	struct mhl_dev_context *dev_context;
	dev_context = container_of((void *)hw_context, struct mhl_dev_context, drv_context);

	if (dev_context->hdcp_status == 0) {
		MHL_TX_DBG_ERR("Disable HDCP 2.2 by command \n");
		unmute_video(hw_context);
		return;
	}

	if (IN_MHL3_MODE(hw_context)) {
		MHL_TX_DBG_ERR("Start HDCP 2.2\n");

		start_hdcp_content_type(hw_context);

		
		enable_intr(hw_context, INTR_HDCP2, BIT_HDCP2_INTR_AUTH_DONE |
			    BIT_HDCP2_INTR_AUTH_FAIL |
			    BIT_HDCP2_INTR_RPTR_RCVID_CHANGE);

		
		mhl_tx_modify_reg(hw_context, REG_M3_P0CTRL,
				  BIT_M3_P0CTRL_MHL3_P0_HDCP_EN,
				  BIT_M3_P0CTRL_MHL3_P0_HDCP_EN);

		
		mhl_tx_write_reg(hw_context, REG_HDCP2X_CTRL_0, 0x83);

		
		hw_context->hdcp2_started = true;
		mhl_tx_write_reg(hw_context, REG_HDCP2X_POLL_CS, 0x70);

		mhl_tx_write_reg(hw_context, REG_HDCP2X_RPT_SMNG_K, 1);

		mhl_tx_write_reg(hw_context, REG_HDCP2X_CTRL_1, 0x01);
		mhl_tx_write_reg(hw_context, REG_HDCP2X_CTRL_1, 0x00);
	} else {
		
		stop_video(hw_context);
		MHL_TX_DBG_ERR("Start HDCP 1.x\n");
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_addr,
			0xFF);

		if (ok_to_proceed_with_ddc(hw_context)) {
			
			enable_intr(hw_context, INTR_HDCP,
			    (BIT_TPI_INTR_ST0_TPI_AUTH_CHNGE_STAT |
			    BIT_TPI_INTR_ST0_TPI_COPP_CHNGE_STAT |
#ifdef KSV_FIFO_RDY_INTERRUPT
			    BIT_TPI_INTR_ST0_KSV_FIFO_FIRST_STAT |
#endif
			    BIT_TPI_INTR_ST0_READ_BKSV_BCAPS_DONE_STAT |
			    BIT_TPI_INTR_ST0_READ_BKSV_ERR_STAT));

			msleep(250);
			mhl_tx_modify_reg(hw_context, REG_TPI_SC,
					  BIT_TPI_SC_REG_TMDS_OE,
					  VAL_TPI_SC_REG_TMDS_OE_ACTIVE);
		}
	}
}

static void start_hdcp_content_type(struct drv_hw_context *hw_context)
{
	uint8_t misc_ctrl;
	uint8_t index;
	uint8_t msg[2] = { 0x01, 0x00 };	

	if (hdcp_content_type == 1)
		msg[1] = 0x01;

	MHL_TX_DBG_INFO("HDCP Content Type = %d\n", msg[1]);

	misc_ctrl = mhl_tx_read_reg(hw_context, REG_HDCP2X_MISC_CTRL);

	mhl_tx_write_reg(hw_context, REG_HDCP2X_MISC_CTRL, misc_ctrl |
			 BIT_HDCP2X_MISC_CTRL_HDCP2X_RPT_SMNG_WR_START);
	mhl_tx_write_reg(hw_context, REG_HDCP2X_MISC_CTRL, misc_ctrl);

	
	for (index = 0; index < 2; index++) {
		mhl_tx_write_reg(hw_context, REG_HDCP2X_RPT_SMNG_IN,
				 msg[index]);

		mhl_tx_write_reg(hw_context, REG_HDCP2X_MISC_CTRL,
			misc_ctrl |
			BIT_HDCP2X_MISC_CTRL_HDCP2X_RPT_SMNG_WR);
		mhl_tx_write_reg(hw_context, REG_HDCP2X_MISC_CTRL,
				 misc_ctrl);
	}
}

static int start_video(struct drv_hw_context *hw_context)
{
	struct mhl_dev_context *dev_context;
	dev_context = get_mhl_device_context(hw_context);
	if (!IN_MHL3_MODE(hw_context)) {
		stop_video(hw_context);
	}

	if ((0 == hw_context->video_path)
	    || (!ok_to_proceed_with_ddc(hw_context))
	    || (false == dev_context->misc_flags.flags.rap_content_on)
	    ) {
		return false;
	}

	if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
				 hw_context->rx_hdmi_ctrl2_defval =
				 REG_RX_HDMI_CTRL2_DEFVAL_HDMI |
				 VAL_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);
		if (false == set_hdmi_params(dev_context)) {
			
			return false;
		}

		if (IN_MHL3_MODE(hw_context)) {
			
			mhl_tx_write_reg(hw_context, REG_M3_CTRL,
					 VAL_M3_CTRL_MHL3_VALUE |
					 BIT_M3_CTRL_H2M_SWRST);

			
			mhl_tx_write_reg(hw_context, REG_M3_CTRL,
					 VAL_M3_CTRL_MHL3_VALUE);
		} else {
			set_auto_zone_for_mhl_1_2(hw_context);

			start_hdcp(hw_context);
		}
		mhl_tx_write_reg_block(hw_context, REG_TPI_AVI_CHSUM,
				       sizeof(hw_context->outgoingAviPayLoad.
					      ifData),
				       (uint8_t *) &hw_context->
				       outgoingAviPayLoad.ifData);
		mhl_tx_write_reg(hw_context, REG_PKT_FILTER_0, 0xA5);
		/* Value written does not match comment above per values below
		 * BIT_PKT_FILTER_0_DROP_CEA_GAMUT_PKT  (0x80)
		 * BIT_PKT_FILTER_0_DROP_MPEG_PKT       (0x20)
		 * BIT_PKT_FILTER_0_DROP_AVI_PKT        (0x04)
		 * BIT_PKT_FILTER_0_DROP_GCP_PKT        (0x01)
		 */

		if (IN_MHL3_MODE(hw_context)) {
			enum info_sel_e {
				info_sel_avi =
				    0, info_sel_spd, info_sel_audio,
				    info_sel_mpeg, info_sel_generic,
				    info_sel_generic2, info_sel_vsi,
				    info_sel_reserved
			};
			uint8_t vsif_buffer[31];
			mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1,
				BIT_PKT_FILTER_1_VSI_OVERRIDE_DIS |
				BIT_PKT_FILTER_1_DROP_GEN_PKT |
				BIT_PKT_FILTER_1_DROP_VSIF_PKT);

			mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL,
					 BIT_TPI_INFO_FSEL_TPI_INFO_EN |
					 BIT_TPI_INFO_FSEL_TPI_INFO_RPT |
					 info_sel_vsi);

			memset(vsif_buffer, 0, sizeof(vsif_buffer));
			if (hw_context->callbacks.hpd_driven_high) {
				switch (hw_context->hpd_high_callback_status) {
				case HH_FMT_HDMI_VSIF_MHL3:
				case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON:
				case HH_FMT_HDMI_VSIF_MHL3_NOT_RPT:
				case HH_FMT_HDMI_VSIF_MHL3_HDCP_ON_NOT_RPT:
					memcpy(vsif_buffer,
					       (uint8_t *) &hw_context->
					       vsif_mhl3_or_hdmi_from_callback.
					       mhl3_vsif,
					       sizeof(hw_context->
						vsif_mhl3_or_hdmi_from_callback.
						mhl3_vsif));
					break;
				default:
					memcpy(vsif_buffer,
					       (uint8_t *) &hw_context->
					       outgoing_mhl3_vsif,
					       sizeof(hw_context->
						      outgoing_mhl3_vsif));
				}
			} else {
				memcpy(vsif_buffer,
				       (uint8_t *) &hw_context->
				       outgoing_mhl3_vsif,
				       sizeof(hw_context->outgoing_mhl3_vsif));
			}
			mhl_tx_write_reg_block(hw_context,
					       REG_TPI_INFO_B0,
					       sizeof(vsif_buffer),
					       (uint8_t *) vsif_buffer);
		} else {
			mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1,
					 BIT_PKT_FILTER_1_DROP_GEN_PKT);
		}
	} else {
		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
				 hw_context->rx_hdmi_ctrl2_defval =
				 REG_RX_HDMI_CTRL2_DEFVAL_DVI |
				 VAL_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);

		MHL_TX_DBG_ERR("DVI - Start HDCP\n");
		start_hdcp(hw_context);
	}

	return true;
}

static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t intr_status)
{
	uint8_t query_data;

	query_data = mhl_tx_read_reg(hw_context, REG_TPI_COPP_DATA1);
	MHL_TX_DBG_INFO("R3D= %02x R29= %02x\n", intr_status, query_data);

	if (BIT_TPI_INTR_ST0_READ_BKSV_BCAPS_DONE_STAT & intr_status) {

		msleep(200);

		if (BIT_TPI_COPP_DATA1_COPP_PROTYPE & query_data) {
			if (BIT_TPI_COPP_DATA1_COPP_HDCP_REP &
			    query_data) {
#ifdef KSV_FIFO_RDY_INTERRUPT
				
				mhl_tx_write_reg(hw_context,
					REG_TPI_COPP_DATA2,
					BIT_TPI_COPP_DATA2_KSV_FORWARD |
					VAL_TPI_COPP_PROTLEVEL_MAX);
#else
				msleep(SLEEP_10MS);
				
				mhl_tx_write_reg(hw_context,
					REG_TPI_COPP_DATA2,
					VAL_TPI_COPP_PROTLEVEL_MAX);
#endif
			} else {
				
				mhl_tx_write_reg(hw_context,
					REG_TPI_COPP_DATA2,
					VAL_TPI_COPP_PROTLEVEL_MAX);
			}

		}
#ifdef KSV_FIFO_RDY_INTERRUPT
	} else if (BIT_TPI_INTR_ST0_KSV_FIFO_FIRST_STAT & intr_status) {
		
		int ksv_fifo_stat;
		int cbus_connected_state;
		
		do {
			int ksv_fifo_bytes;
			int dummy =
			    mhl_tx_read_reg(hw_context,
					    REG_TPI_DS_BCAPS);
			ksv_fifo_stat =
			    mhl_tx_read_reg(hw_context,
					    REG_TPI_KSV_FIFO_STAT);
			ksv_fifo_bytes =
			    ksv_fifo_stat &
			    MSK_TPI_KSV_FIFO_STAT_KSV_FIFO_BYTES;
			if (ksv_fifo_bytes) {
				int ksv;
				ksv =
				    mhl_tx_read_reg(hw_context,
						REG_TPI_KSV_FIFO_FORW);
			}
			if (BIT_TPI_KSV_FIFO_STAT_KSV_FIFO_LAST &
			    ksv_fifo_stat) {
				break;
			}
			cbus_connected_state =
			    mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
			cbus_connected_state &=
			    (BIT_CBUS_STATUS_CBUS_CONNECTED |
			     BIT_CBUS_STATUS_CBUS_HPD);
		} while ((BIT_CBUS_STATUS_CBUS_CONNECTED |
			  BIT_CBUS_STATUS_CBUS_HPD) ==
			 cbus_connected_state);

#endif
	} else if (BIT_TPI_INTR_ST0_READ_BKSV_ERR_STAT & intr_status) {
		MHL_TX_DBG_WARN("BKSV ERROR - Start HDCP\n");
		start_hdcp(hw_context);
	} else if (BIT_TPI_INTR_ST0_TPI_COPP_CHNGE_STAT & intr_status) {
		int link_status;

		link_status =
		    query_data & MSK_TPI_COPP_DATA1_COPP_LINK_STATUS;

		switch (link_status) {
		case VAL_TPI_COPP_LINK_STATUS_NORMAL:
			break;

		case VAL_TPI_COPP_LINK_STATUS_LINK_LOST:
			MHL_TX_DBG_ERR("LINK LOST - Start HDCP\n");
			start_hdcp(hw_context);
			break;
		case VAL_TPI_COPP_LINK_STATUS_RENEGOTIATION_REQ:
			MHL_TX_DBG_INFO("tpi BSTATUS2: 0x%x\n",
					mhl_tx_read_reg(hw_context,
							REG_TPI_BSTATUS2)
			    );
			mhl_tx_modify_reg(hw_context, REG_TPI_SC,
					  BIT_TPI_SC_TPI_AV_MUTE,
					  VAL_TPI_SC_TPI_AV_MUTE_MUTED);

			mhl_tx_write_reg(hw_context, REG_TPI_COPP_DATA2,
					 0);
			break;
		case VAL_TPI_COPP_LINK_STATUS_LINK_SUSPENDED:
			MHL_TX_DBG_ERR("LINK suspended - Start HDCP\n");
			start_hdcp(hw_context);
			break;
		}
	} else if (BIT_TPI_INTR_ST0_TPI_AUTH_CHNGE_STAT & intr_status) {
		uint8_t new_link_prot_level;

		new_link_prot_level = (uint8_t)
		    (query_data & (BIT_TPI_COPP_DATA1_COPP_GPROT |
				   BIT_TPI_COPP_DATA1_COPP_LPROT));

		switch (new_link_prot_level) {
		case (VAL_TPI_COPP_GPROT_NONE | VAL_TPI_COPP_LPROT_NONE):
			MHL_TX_DBG_ERR("?PROT_NONE - Start HDCP\n");
			start_hdcp(hw_context);
			break;

		case VAL_TPI_COPP_GPROT_SECURE:
			break;
		case (VAL_TPI_COPP_GPROT_SECURE | VAL_TPI_COPP_LPROT_SECURE):
		case VAL_TPI_COPP_LPROT_SECURE:
			MHL_TX_DBG_ERR("HDCP 1.x Authentication Done\n");
			unmute_video(hw_context);
			break;
		}
	}
	return 0;
}

static int hdcp2_isr(struct drv_hw_context *hw_context, uint8_t intr_status)
{
	uint8_t rcvr_info[3];
	uint8_t rcvr_id_list[5];

	if (intr_status & BIT_HDCP2_INTR_AUTH_DONE) {
		MHL_TX_DBG_ERR("HDCP2.2 Authentication Done.\n");

		
	}

	if (intr_status & BIT_HDCP2_INTR_AUTH_FAIL) {
		uint8_t ro_gp0;
		uint8_t ro_auth[2];

		

		ro_gp0 = mhl_tx_read_reg(hw_context, REG_HDCP2X_GP_OUT0);
		mhl_tx_read_reg_block(hw_context, REG_HDCP2X_AUTH_STAT,
				      sizeof(ro_auth), ro_auth);

		MHL_TX_DBG_ERR("HDCP2.2 Authentication Failed\n"
			"\tgp0 %02X, status %02X %02X\n",
			ro_gp0, ro_auth[0], ro_auth[1]);
	}

	if (intr_status & BIT_HDCP2_INTR_RPTR_RCVID_CHANGE) {
		MHL_TX_DBG_ERR("HDCP2.2 RCV_ID Changed.\n");

		
		mhl_tx_read_reg_block(hw_context,
				      REG_HDCP2X_RPT_RCVID_OUT,
				      sizeof(rcvr_info), rcvr_info);
		mhl_tx_read_reg_block(hw_context,
				      REG_HDCP2X_RPT_RCVR_ID0,
				      sizeof(rcvr_id_list), rcvr_id_list);
	}

	mhl_tx_write_reg(hw_context, REG_HDCP2X_INTR0, intr_status);

	return intr_status;
}

static int int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status)
{
	struct vendor_specific_info_frame_t vsif;
	struct avi_info_frame_t avif;

	memset(&vsif, 0, sizeof(vsif));
	
	mhl_tx_write_reg(hw_context, REG_INTR8, intr_8_status);

	if (hw_context->hpd_high_callback_status >= 0) {

		hw_context->hpd_high_callback_status = HH_VIDEO_NOT_RDY;
		return intr_8_status;
	}

	
	if ((BIT_CEA_NEW_VSI | BIT_CEA_NEW_AVI) & intr_8_status) {
		MHL_TX_DBG_WARN("got NEW_VSI\n");

		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
				 hw_context->
				 rx_hdmi_ctrl2_defval |
				 VAL_RX_HDMI_CTRL2_VSI_MON_SEL_VSI);

		mhl_tx_read_reg_block(hw_context,
				      REG_RX_HDMI_MON_PKT_HEADER1,
				      sizeof(vsif),
				      (uint8_t *) &vsif);
		MHL_TX_DBG_WARN
		    ("Got vsif: {%02x %02x %02x} %02x %02x%02x%02x %02x %02x\n",
		     vsif.header.type_code, vsif.header.version_number,
		     vsif.header.length, vsif.payLoad.checksum,
		     vsif.payLoad.IEEERegistrationIdentifier[0],
		     vsif.payLoad.IEEERegistrationIdentifier[1],
		     vsif.payLoad.IEEERegistrationIdentifier[2],
		     *((uint8_t *) &vsif.payLoad.pb4),
		     vsif.payLoad.pb5.HDMI_VIC);
#ifdef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
		hw_context->valid_vsif = false;
#endif
	}

	if (BIT_CEA_NEW_AVI & intr_8_status) {

		MHL_TX_DBG_WARN("got NEW_AVIF\n");

		

		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2,
				 hw_context->
				 rx_hdmi_ctrl2_defval |
				 VAL_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);

		mhl_tx_read_reg_block(hw_context,
				      REG_RX_HDMI_MON_PKT_HEADER1,
				      sizeof(avif),
				      (uint8_t *) &avif);

		if (0 == avif.header.type_code) {
			MHL_TX_DBG_ERR(
				"bogus AVIF: avif.header.type_code is 0\n");
			return intr_8_status;
		}
	}

	switch (intr_8_status &
		(BIT_CEA_NEW_VSI | BIT_CEA_NEW_AVI)) {
	case BIT_CEA_NEW_VSI:
		process_info_frame_change(hw_context, &vsif, NULL);
		break;
	case BIT_CEA_NEW_AVI:
		process_info_frame_change(hw_context, NULL, &avif);
		break;
	case (BIT_CEA_NEW_VSI | BIT_CEA_NEW_AVI):
		process_info_frame_change(hw_context, &vsif, &avif);
		break;
	}

	return intr_8_status;
}

static int int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status)
{
	if (BIT_DDC_CMD_DONE & int_3_status) {
		
		hw_context->intr_info->flags |= DRV_INTR_MSC_DONE;
		hw_context->intr_info->msc_done_data = 0;
		
		enable_intr(hw_context, INTR_DDC, 0);
	}
	return 0;
}

static int int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status)
{
	if (int_9_status) {
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_EDID].stat_addr,
				 int_9_status);
		if (BIT_INTR9_DEVCAP_DONE & int_9_status) {
			hw_context->intr_info->flags |= DRV_INTR_MSC_DONE;
			hw_context->intr_info->msc_done_data = 0;
		}

		if (BIT_INTR9_EDID_DONE & int_9_status) {
			int ddcStatus;
			ddcStatus =
			    mhl_tx_read_reg(hw_context, REG_DDC_STATUS);
#ifdef SWWA_BZ30759
			unfreeze_MHL_connect(hw_context);
#endif
			if (BIT_DDC_STATUS_DDC_NO_ACK & ddcStatus) {

				
				hw_context->current_edid_req_blk = 0;
				if (!issue_edid_read_request(hw_context,
				     hw_context->current_edid_req_blk)) {
					hw_context->intr_info->flags |=
					    DRV_INTR_MSC_DONE;
					hw_context->intr_info->msc_done_data =
					    1;
					MHL_TX_DBG_ERR
					    ("%sedid request problem%s\n",
					     ANSI_ESC_RED_TEXT,
					     ANSI_ESC_RESET_TEXT);
				}
			} else {
				int num_extensions;

				MHL_TX_DBG_INFO("EDID block read complete\n");
				num_extensions =
				    si_mhl_tx_get_num_cea_861_extensions
				    (hw_context->intr_info->edid_parser_context,
				     hw_context->current_edid_req_blk);
				if (num_extensions < 0) {
					MHL_TX_DBG_ERR("edid problem:%d\n",
						       num_extensions);
					if (ne_NO_HPD == num_extensions) {
						
						hw_context->intr_info->flags |=
						    DRV_INTR_MSC_DONE;
						hw_context->intr_info->
						    msc_done_data = 1;
						MHL_TX_DBG_ERR("%shpd low%s\n",
							  ANSI_ESC_RED_TEXT,
							  ANSI_ESC_RESET_TEXT);
					} else {
						hw_context->
						    current_edid_req_blk = 0;
						if (!issue_edid_read_request(
							hw_context,
							hw_context->
							current_edid_req_blk)) {
							hw_context->intr_info->
							    flags |=
							    DRV_INTR_MSC_DONE;
							hw_context->intr_info->
							    msc_done_data = 1;
							MHL_TX_DBG_ERR("%sedid"
							 " request problem%s\n",
							 ANSI_ESC_RED_TEXT,
							 ANSI_ESC_RESET_TEXT);
						}
					}
				} else if (hw_context->current_edid_req_blk <
					   num_extensions) {
					
					if (!issue_edid_read_request(
						hw_context, ++hw_context->
						current_edid_req_blk)) {
						hw_context->intr_info->flags |=
						    DRV_INTR_MSC_DONE;
						hw_context->intr_info->
						    msc_done_data = 1;
						MHL_TX_DBG_ERR("%sedid request"
							" problem%s\n",
							ANSI_ESC_RED_TEXT,
							ANSI_ESC_RESET_TEXT);
					}
				} else {
					hw_context->intr_info->flags |=
					    DRV_INTR_MSC_DONE;
					hw_context->intr_info->msc_done_data =
					    0;
				}
			}
		}

		if (BIT_INTR9_EDID_ERROR & int_9_status) {
#ifdef SWWA_BZ30759
			unfreeze_MHL_connect(hw_context);
#endif

			MHL_TX_DBG_INFO("EDID read error, retrying\n");

			hw_context->edid_fifo_block_number = 0;
			hw_context->current_edid_req_blk = 0;

			if (ok_to_proceed_with_ddc(hw_context)) {

				
				hw_context->current_edid_req_blk = 0;
				if (!issue_edid_read_request(hw_context,
				     hw_context->current_edid_req_blk)) {
					
					hw_context->intr_info->flags |=
					    DRV_INTR_MSC_DONE;
					hw_context->intr_info->msc_done_data =
					    1;
				}
			} else {
				
				hw_context->intr_info->flags |=
				    DRV_INTR_MSC_DONE;
				hw_context->intr_info->msc_done_data = 1;
			}
		}
	}
	return int_9_status;
}

void si_mhl_tx_read_devcap_fifo(struct drv_hw_context *hw_context,
				union MHLDevCap_u *dev_cap_buf)
{
	MHL_TX_DBG_INFO("called\n");

	
	enable_intr(hw_context, INTR_EDID,
		    (BIT_INTR9_DEVCAP_DONE_MASK
		     | BIT_INTR9_EDID_DONE_MASK | BIT_INTR9_EDID_ERROR));

	
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
			 VAL_EDID_CTRL_DEVCAP_SELECT_DEVCAP |
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE |
			 VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);

	
	mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA,
			      DEVCAP_SIZE, dev_cap_buf->devcap_cache);
	MHL_TX_DBG_INFO("%sgot DEVCAP%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
}

void si_mhl_tx_read_xdevcap_fifo(struct drv_hw_context *hw_context,
	union MHLXDevCap_u *xdev_cap_buf)
{
	MHL_TX_DBG_INFO("called\n");

	
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL,
			 VAL_EDID_CTRL_EDID_PRIME_VALID_DISABLE |
			 BIT_EDID_CTRL_XDEVCAP_EN |
			 VAL_EDID_CTRL_DEVCAP_SELECT_DEVCAP |
			 VAL_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE |
			 VAL_EDID_CTRL_EDID_MODE_EN_ENABLE);
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);

	
	mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA,
			      XDEVCAP_OFFSET(XDEVCAP_LIMIT),
			      xdev_cap_buf->xdevcap_cache);

	MHL_TX_DBG_INFO("%sgot XDEVCAP%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
}

static int mhl_cbus_err_isr(struct drv_hw_context *hw_context,
			    uint8_t cbus_err_int)
{
	int ret_val = 0;
	uint8_t msc_abort_reason = 0;

#ifdef PRINT_DDC_ABORTS
	if (cbus_err_int & BIT_CBUS_DDC_ABORT) {
		uint8_t ddc_abort_reason = 0;

		ddc_abort_reason = mhl_tx_read_reg(hw_context,
						   REG_DDC_ABORT_INT);

		MHL_TX_DBG_INFO("CBUS DDC ABORT. Reason = %02X\n",
				ddc_abort_reason);

		if (DDC_ABORT_THRESHOLD < ++ddc_abort_count) {
			ddc_abort_count = 0;
		} else if (MHL_READ_EDID_BLOCK ==
			   hw_context->current_cbus_req.command) {
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			
			hw_context->intr_info->msc_done_data = 1;
		}
	}
#endif
	if (cbus_err_int & BIT_CBUS_MSC_ABORT_RCVD) {
		hw_context->intr_info->flags |= DRV_INTR_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context,
						   REG_MSC_MR_ABORT_INT);

		++msc_abort_count;

		MHL_TX_DBG_ERR("#%d: ABORT during MSC RCV. Reason = %02X\n",
			       msc_abort_count, msc_abort_reason);
	}

	if (cbus_err_int & BIT_CBUS_CMD_ABORT) {
		hw_context->intr_info->flags |= DRV_INTR_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context,
						   REG_MSC_MT_ABORT_INT);

		MHL_TX_DBG_ERR("CBUS ABORT during MSC SEND. Reason = %02X\n",
			       msc_abort_reason);

		mhl_tx_write_reg(hw_context, REG_MSC_MT_ABORT_INT,
				 msc_abort_reason);
	}
	if (msc_abort_reason) {
		if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & msc_abort_reason)
			MHL_TX_DBG_ERR("Retry threshold exceeded\n");
		if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & msc_abort_reason)
			MHL_TX_DBG_ERR("Protocol Error\n");
		if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & msc_abort_reason)
			MHL_TX_DBG_ERR("Translation layer timeout\n");
		if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & msc_abort_reason)
			MHL_TX_DBG_ERR("Undefined opcode\n");
		if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT &
		    msc_abort_reason)
			MHL_TX_DBG_ERR("MSC Peer sent an ABORT\n");
	}
	return ret_val;
}

static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int)
{

	switch (hw_context->cbus_mode) {
	case CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
	case CM_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		MHL_TX_DBG_ERR("%sCBUS1 message received cbus_int:0x%02x "
			"hpd status: 0x%02x%s\n",
			ANSI_ESC_YELLOW_TEXT,
			cbus_int,
			mhl_tx_read_reg(hw_context, REG_CBUS_STATUS),
			ANSI_ESC_RESET_TEXT);
		return -1;
	default:
		break;
	}

	if (CBUS1_IDLE_RCV_PEND == hw_context->cbus1_state) {
		return -1;
	}

	if (cbus_int & ~BIT_CBUS_HPD_CHG) {
		mhl_tx_write_reg(hw_context, REG_CBUS_INT_0, cbus_int &
				 ~BIT_CBUS_HPD_CHG);
	}

	if (BIT_CBUS_HPD_CHG & cbus_int) {
		uint8_t cbus_status;
		uint8_t status;

		
		cbus_status =
		    mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
		status = cbus_status & BIT_CBUS_STATUS_CBUS_HPD;

		if (BIT_CBUS_STATUS_CBUS_HPD &
		    (hw_context->cbus_status ^ cbus_status)) {
			mhl_tx_write_reg(hw_context, REG_CBUS_INT_0,
				 BIT_CBUS_INT_0_CBUS_INT_0_STAT2);
			MHL_TX_DBG_INFO("HPD change\n");
		} else {
			MHL_TX_DBG_ERR("missed HPD change\n");

			
			status ^= BIT_CBUS_STATUS_CBUS_HPD;
			cbus_status ^= BIT_CBUS_STATUS_CBUS_HPD;
		}

		MHL_TX_DBG_INFO("DS HPD changed to %02X\n", status);

		hw_context->intr_info->flags |= DRV_INTR_HPD_CHANGE;
		hw_context->intr_info->hpd_status = status;

		if (0 == status) {
			struct mhl_dev_context *dev_context;
			dev_context = get_mhl_device_context(hw_context);
			mhl_tx_stop_timer(dev_context,
					  hw_context->
					  input_field_rate_measurement_timer);
			MHL_TX_DBG_ERR("%sgot CLR_HPD%s\n", ANSI_ESC_GREEN_TEXT,
				       ANSI_ESC_RESET_TEXT);
			
			if (MHL_SEND_3D_REQ_OR_FEAT_REQ ==
			    hw_context->current_cbus_req.command) {

				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |=
				    DRV_INTR_MSC_DONE;
				hw_context->intr_info->msc_done_data = 1;
			} else if (MHL_READ_EDID_BLOCK ==
				   hw_context->current_cbus_req.command) {
				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |=
				    DRV_INTR_MSC_DONE;
				
				hw_context->intr_info->msc_done_data = 1;
			}
			drive_hpd_low(hw_context);
			hw_context->current_edid_req_blk = 0;

			
			hw_context->video_ready = false;
			hw_context->video_path = 1;

			si_edid_reset(dev_context->edid_parser_context);

			
			stop_video(hw_context);
		} else {
			MHL_TX_DBG_ERR("%sGot SET_HPD%s\n", ANSI_ESC_GREEN_TEXT,
				       ANSI_ESC_RESET_TEXT);
		}

		hw_context->cbus_status = cbus_status;
	}

	if (BIT_CBUS_MSC_MT_DONE_NACK & cbus_int) {
		MHL_TX_DBG_ERR("%sGot MSC_MT_DONE_NACK%s\n", ANSI_ESC_RED_TEXT,
			ANSI_ESC_RESET_TEXT);
		hw_context->intr_info->flags |= DRV_INTR_MSC_NAK;
	}

	if (BIT_CBUS_MSC_MR_WRITE_STAT & cbus_int) {
		
		mhl_tx_read_reg_block(hw_context, REG_MHL_STAT_0,
			ARRAY_SIZE(hw_context->intr_info->dev_status.
				write_stat),
			hw_context->intr_info->dev_status.write_stat);
		
		mhl_tx_read_reg_block(hw_context, REG_MHL_EXTSTAT_0,
			ARRAY_SIZE(hw_context->intr_info->dev_status.
				write_xstat),
			hw_context->intr_info->dev_status.write_xstat);
		MHL_TX_DBG_WARN
		    ("%sGot WRITE_STAT: "
		     "%02x %02x %02x : %02x %02x %02x %02x%s\n",
		     ANSI_ESC_GREEN_TEXT,
		     hw_context->intr_info->dev_status.write_stat[0],
		     hw_context->intr_info->dev_status.write_stat[1],
		     hw_context->intr_info->dev_status.write_stat[2],
		     hw_context->intr_info->dev_status.write_xstat[0],
		     hw_context->intr_info->dev_status.write_xstat[1],
		     hw_context->intr_info->dev_status.write_xstat[2],
		     hw_context->intr_info->dev_status.write_xstat[3],
		     ANSI_ESC_RESET_TEXT);
		if (hw_context->intr_info->dev_status.write_stat[2] >= 0x30) {
			hw_context->mhl_peer_version_stat =
			    hw_context->intr_info->dev_status.write_stat[2];
		}
#ifdef FORCE_OCBUS_FOR_ECTS
		{
			
			if (force_ocbus_for_ects) {
				hw_context->mhl_peer_version_stat =
				    hw_context->intr_info->dev_status.
				    write_stat[2] = 0x20;
			}
		}
#endif
		if (MHL_STATUS_DCAP_RDY & hw_context->intr_info->dev_status.
		    write_stat[0]) {
			MHL_TX_DBG_INFO("DCAP_RDY in effect\n");

			switch (hw_context->cbus_mode) {
			case CM_oCBUS_PEER_VERSION_PENDING:
				if (hw_context->mhl_peer_version_stat >= 0x30) {
					if (MHL_STATUS_XDEVCAPP_SUPP &
					    hw_context->intr_info->dev_status.
					    write_stat[0]) {
						MHL_TX_DBG_ERR
						    ("%sdownstream device "
						     "supports MHL3.0+%s\n",
						     ANSI_ESC_GREEN_TEXT,
						     ANSI_ESC_RESET_TEXT);
						hw_context->cbus_mode =
						    CM_oCBUS_PEER_IS_MHL3;

						mhl_tx_write_reg(hw_context,
						 REG_M3_CTRL,
						 VAL_M3_CTRL_MHL3_VALUE);

					}
				}
				break;
			default:
				break;
			}

			if (CM_oCBUS_PEER_VERSION_PENDING ==
			    hw_context->cbus_mode) {
				hw_context->cbus_mode = CM_oCBUS_PEER_IS_MHL1_2;
				si_mhl_tx_drv_switch_cbus_mode(hw_context,
						CM_oCBUS_PEER_IS_MHL1_2);
			}
			peer_specific_init(hw_context);

			
			enable_intr(hw_context, INTR_EDID,
				    (BIT_INTR9_DEVCAP_DONE_MASK
				     | BIT_INTR9_EDID_DONE_MASK
				     | BIT_INTR9_EDID_ERROR));
		}
		hw_context->intr_info->flags |= DRV_INTR_WRITE_STAT;
	}

	if ((BIT_CBUS_MSC_MR_MSC_MSG & cbus_int)) {
		hw_context->intr_info->flags |= DRV_INTR_MSC_RECVD;
		mhl_tx_read_reg_block(hw_context,
			REG_MSC_MR_MSC_MSG_RCVD_1ST_DATA,
			ARRAY_SIZE(hw_context->intr_info->msc_msg),
			hw_context->intr_info->msc_msg);

		MHL_TX_DBG_INFO("MSC MSG: %02X %02X\n",
			hw_context->intr_info->msc_msg[0],
			hw_context->intr_info->msc_msg[1]);
	}

	if (BIT_CBUS_MSC_MR_SET_INT & cbus_int) {
		MHL_TX_DBG_WARN("MHL INTR Received\n");

		hw_context->intr_info->flags |= DRV_INTR_SET_INT;
		mhl_tx_read_reg_block(hw_context, REG_MHL_INT_0,
			ARRAY_SIZE(hw_context->intr_info->int_msg),
			hw_context->intr_info->int_msg);
		
		mhl_tx_write_reg_block(hw_context, REG_MHL_INT_0,
			ARRAY_SIZE(hw_context->intr_info->int_msg),
			hw_context->intr_info->int_msg);

		if (MHL_INT_EDID_CHG & hw_context->intr_info->int_msg[1]) {
			int reg_val;

			MHL_TX_DBG_INFO("%sgot EDID_CHG%s\n",
					ANSI_ESC_GREEN_TEXT,
					ANSI_ESC_RESET_TEXT);

			mhl_tx_modify_reg(hw_context, REG_TPI_INFO_FSEL,
					  BIT_TPI_INFO_FSEL_TPI_INFO_RPT,
					  0);

			
			drive_hpd_low(hw_context);
			stop_video(hw_context);

			
			reg_val =
			    mhl_tx_read_reg(hw_context, REG_LM_DDC);
			MHL_TX_DBG_INFO("REG_LM_DDC:%02x\n", reg_val);
			reg_val &= ~BIT_LM_DDC_SW_TPI_EN;
			reg_val |= VAL_LM_DDC_SW_TPI_EN_DISABLED;
			mhl_tx_write_reg(hw_context, REG_LM_DDC,
					 reg_val);

			
			MHL_TX_DBG_INFO("REG_LM_DDC:%02x\n", reg_val);
			reg_val &= ~BIT_LM_DDC_SW_TPI_EN;
			reg_val |= VAL_LM_DDC_SW_TPI_EN_ENABLED;
			mhl_tx_write_reg(hw_context, REG_LM_DDC,
					 reg_val);

			mhl_tx_write_reg(hw_context,
				g_intr_tbl[INTR_HDCP].stat_addr, 0xFF);
		} else if (MHL_INT_DSCR_CHG &
			hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got DSCR_CHG\n");
			if (hw_context->gen2_write_burst_rcv) {
				MHL_TX_DBG_INFO("Ignored DSCR_CHG "
					"since MDT is enabled\n");
			} else {
				mhl_tx_read_reg_block(hw_context,
					REG_MHL_SCRPAD_0,
						ARRAY_SIZE(hw_context->
							write_burst_data),
						hw_context->
							write_burst_data);
			}
		} else if (MHL_INT_DCAP_CHG &
			hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got DCAP_CHG\n");
		} else if (MHL_INT_REQ_WRT &
			hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got REQ_WRT\n");
		}
	}
	if (BIT_CBUS_MSC_MT_DONE & cbus_int) {
		bool completed = true;
		MHL_TX_DBG_INFO("MSC_REQ_DONE,0x%02x(0x%02x,0x%02x)\n",
				hw_context->current_cbus_req.command,
				hw_context->current_cbus_req.reg,
				hw_context->current_cbus_req.reg_data);

		if (MHL_SET_INT == hw_context->current_cbus_req.command) {
			if (MHL_RCHANGE_INT ==
				hw_context->current_cbus_req.reg) {
				if (MHL2_INT_3D_REQ & hw_context->
				    current_cbus_req.reg_data) {
					MHL_TX_DBG_WARN("3D_REQ complete\n");
					hw_context->cbus1_state =
						CBUS1_IDLE_RCV_ENABLED;
				}
				if (MHL_INT_GRT_WRT & hw_context->
				    current_cbus_req.reg_data) {
					MHL_TX_DBG_WARN("GRT_WRT complete\n");
					hw_context->cbus1_state =
						CBUS1_IDLE_RCV_ENABLED;
				}
			}
		} else if (MHL_SEND_3D_REQ_OR_FEAT_REQ ==
			   hw_context->current_cbus_req.command) {
			completed = false;
			hw_context->cbus1_state = CBUS1_IDLE_RCV_ENABLED;
		}
		if (completed) {
			hw_context->current_cbus_req.command = 0x00;
			hw_context->intr_info->flags |= DRV_INTR_MSC_DONE;
			hw_context->intr_info->msc_done_data =
			    mhl_tx_read_reg(hw_context,
					    REG_MSC_MT_RCVD_DATA0);
		}
	}
	return -1;
}

static int int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status)
{
	int ret_val = 0;

	if (int_5_status & BIT_INTR_SCDT_CHANGE) {
		uint8_t temp;
		temp = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);

		if (BIT_TMDS_CSTAT_P3_SCDT & temp) {
			MHL_TX_DBG_WARN("%sGot SCDT HIGH%s\n",
					ANSI_ESC_GREEN_TEXT,
					ANSI_ESC_RESET_TEXT);
			enable_intr(hw_context, INTR_INFR,
				    BIT_CEA_NEW_AVI |
				    BIT_CEA_NEW_VSI);

			if (IN_MHL3_MODE(hw_context) ||
			    si_edid_sink_is_hdmi(hw_context->intr_info->
						 edid_parser_context)) {

				mhl_tx_write_reg(hw_context, REG_TPI_SC,
				    VAL_TPI_SC_TPI_OUTPUT_MODE_0_HDMI);

				enable_intr(hw_context, INTR_INFR,
					    BIT_CEA_NEW_AVI |
					    BIT_CEA_NEW_VSI);
			} else {
				if (hw_context->hpd_high_callback_status >= 0) {

					hw_context->hpd_high_callback_status =
					    HH_VIDEO_NOT_RDY;
					return ret_val;
				}
				start_video(hw_context);
			}
			MHL_TX_DBG_INFO("actual_mode: %s0x%02x%s\n",
				ANSI_ESC_YELLOW_TEXT,
				mhl_tx_read_reg(hw_context,
					REG_RX_HDMI_CTRL0),
				ANSI_ESC_RESET_TEXT);
		} else {
			struct mhl_dev_context *dev_context;
			dev_context = get_mhl_device_context(hw_context);
			mhl_tx_stop_timer(dev_context,
					  hw_context->
					  input_field_rate_measurement_timer);
			MHL_TX_DBG_WARN("%sGot SCDT LOW%s\n",
					ANSI_ESC_GREEN_TEXT,
					ANSI_ESC_RESET_TEXT);

			
			mhl_tx_write_reg(hw_context, REG_TMDS_CSTAT_P3,
			    BIT_TMDS_CSTAT_P3_AVIF_MANUAL_CLEAR_STROBE |
			    BIT_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR);

#ifndef MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
			hw_context->valid_vsif = 0;
#ifdef NEVER_USED
			hw_context->valid_avif = 0;
#endif
			memset(&hw_context->current_vs_info_frame, 0,
			       sizeof(hw_context->current_vs_info_frame));
			memset(&hw_context->current_avi_info_frame, 0,
			       sizeof(hw_context->current_avi_info_frame));
#endif

			stop_video(hw_context);
			enable_intr(hw_context, INTR_INFR, 0);
		}
	}
	return ret_val;
}

#ifdef USE_HW_TIMER
static int int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status)
{
	if (int_1_status & BIT_HW_TIMER_POP) {
		MHL_TX_DBG_INFO("Timer Pop\n");

		
	}

	mhl_tx_write_reg(hw_context, REG_INTR1, 0x80);

	return 0;
}
#endif
#define BIT_0072_SW_INTR 0x04
static int int_2_isr(struct drv_hw_context *hw_context, uint8_t int_2_status)
{
	if (BIT_0072_SW_INTR & int_2_status) {
		MHL_TX_DBG_ERR("enabling RGND\n");
		
		enable_intr(hw_context, INTR_DISC, BIT_RGND_READY_INT);
		wait_for_user_intr = 0;
	}
	return 0;
}

int get_device_id(struct drv_hw_context *hw_context)
{
	int ret_val;
	uint16_t number;

	ret_val = mhl_tx_read_reg(hw_context, REG_DEV_IDH);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error 0x%x\n", ret_val);
		return ret_val;
	}
	number = ret_val << 8;

	ret_val = mhl_tx_read_reg(hw_context, REG_DEV_IDL);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error 0x%x\n", ret_val);
		return ret_val;
	}
	ret_val |= number;

	return ret_val;
}

static int get_device_rev(struct drv_hw_context *hw_context)
{
	int ret_val;

	ret_val = mhl_tx_read_reg(hw_context, REG_DEV_REV);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error\n");
		ret_val = -1;
	}

	return ret_val;
}

static void clear_and_disable_on_disconnect(struct drv_hw_context *hw_context)
{
	uint8_t intr_num;
	struct mhl_dev_context *dev_context;
	dev_context = get_mhl_device_context(hw_context);
	mhl_tx_stop_timer(dev_context,
			  hw_context->input_field_rate_measurement_timer);
	
	for (intr_num = 0; intr_num < MAX_INTR; intr_num++) {
		if (INTR_DISC == intr_num) {
			
			mhl_tx_write_reg(hw_context,
					 g_intr_tbl[INTR_DISC].stat_addr,
					 ~BIT_RGND_READY_INT);
			if (wait_for_user_intr) {
				
				enable_intr(hw_context, INTR_DISC, 0);
			} else {
				
				enable_intr(hw_context, INTR_DISC,
					    BIT_RGND_READY_INT);
			}
		} else {
			
			mhl_tx_write_reg(hw_context,
				g_intr_tbl[intr_num].stat_addr, 0xFF);
			if (INTR_USER == intr_num) {
				if (wait_for_user_intr) {
					
					enable_intr(hw_context, INTR_USER,
						    BIT_0072_SW_INTR);
				} else {
					enable_intr(hw_context, INTR_USER,
						    0x00);
				}
			} else {
				enable_intr(hw_context, intr_num, 0x00);
			}
		}
	}
}

static void switch_to_d3(struct drv_hw_context *hw_context,
			 bool do_interrupt_clear)
{
	if (do_interrupt_clear)
		clear_and_disable_on_disconnect(hw_context);

	mhl_tx_write_reg(hw_context, REG_MHL_CBUS_CTL0,
			 (VAL_MHL_CBUS_CTL0_CBUS_DRV_SEL_STRONG |
			  VAL_MHL_CBUS_CTL0_CBUS_RGND_VBIAS_734));

	mhl_tx_write_reg(hw_context, REG_MHL_CBUS_CTL1,
			 VAL_MHL_CBUS_CTL1_1115_OHM);
	mhl_tx_write_reg(hw_context, REG_DPD, BIT_DPD_PWRON_PLL
			 | BIT_DPD_PDNTX12 | BIT_DPD_OSC_EN);
}

static void cbus_reset(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_WARN("Perform CBUS reset to clean MHL STAT values\n");
	mhl_tx_write_reg(hw_context, REG_PWD_SRST,
		BIT_PWD_SRST_CBUS_RST |
		BIT_PWD_SRST_CBUS_RST_SW_EN);
	mhl_tx_write_reg(hw_context, REG_PWD_SRST,
		BIT_PWD_SRST_CBUS_RST_SW_EN);

#ifdef SWWA_BZ30759
	
	
	
	
	platform_mhl_tx_hw_reset(TX_HW_RESET_PERIOD, TX_HW_RESET_DELAY);
	
	mhl_tx_write_reg(hw_context, REG_DPD, BIT_DPD_PWRON_PLL
			 | BIT_DPD_PDNTX12 | BIT_DPD_OSC_EN);

	
	program_ext_clock_regs(hw_context, crystal_khz);

	
	
	
	
#endif
}

static void disconnect_mhl(struct drv_hw_context *hw_context,
			   bool do_interrupt_clear)
{
	disable_gen2_write_burst_rcv(hw_context);
	disable_gen2_write_burst_xmit(hw_context);

	stop_video(hw_context);
	MHL_TX_DBG_WARN("STOP_VIDEO DONE\n");

	mhl_tx_vbus_control(VBUS_OFF);
	msleep(50);
	cbus_reset(hw_context);
	clear_auto_zone_for_mhl_3(hw_context);

	mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x40);
	mhl_tx_write_reg(hw_context, REG_CBUS3_CNVT, 0x84);
	mhl_tx_write_reg(hw_context, REG_COC_CTL14, 0x00);
	mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x40);
	mhl_tx_write_reg(hw_context, REG_HRXCTRL3, 0x07);

	mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0,
		(VAL_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X |
		BIT_MHL_PLL_CTL0_CRYSTAL_CLK_SEL |
		BIT_MHL_PLL_CTL0_ZONE_MASK_OE));

	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xC0);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL1, 0xBB);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL3, 0x48);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3F);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x2F);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL6, 0x2A);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL7, 0x08);

	hw_context->cbus_mode = CM_NO_CONNECTION;
	hw_context->mhl_peer_version_stat = 0;
	si_set_cbus_mode_leds(CM_NO_CONNECTION);

	
	drive_hpd_low(hw_context);

	mhl_tx_write_reg(hw_context, REG_M3_CTRL,
			 VAL_M3_CTRL_PEER_VERSION_PENDING_VALUE);

	mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0x07);

	mhl_tx_write_reg(hw_context, REG_DISC_CTRL4, 0x10);
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL8, 0x00);

	
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL9,
			 BIT_DISC_CTRL9_WAKE_DRVFLT |
			 BIT_DISC_CTRL9_WAKE_PULSE_BYPASS);

	
	disable_heartbeat(hw_context);

	
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL1, 0x25);

	
	enable_hdmi(false);

	if (do_interrupt_clear)
		clear_and_disable_on_disconnect(hw_context);

	
	hw_context->cbus_status = 0;
	hw_context->current_cbus_req.command = 0x00;
	hw_context->hawb_write_pending = false;
	hw_context->cbus1_state = CBUS1_IDLE_RCV_DISABLED;
}

static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status)
{
	int ret_val = 0;

	if ((BIT_CBUS_MHL12_DISCON_INT & int_4_status) ||
	    (BIT_CBUS_MHL3_DISCON_INT & int_4_status) ||
	    (BIT_NOT_MHL_EST_INT & int_4_status)) {
		MHL_TX_DBG_ERR("Got CBUS_DIS. MHL disconnection\n");
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL4, 0x10);

		
		hw_context->intr_info->flags |= DRV_INTR_DISCONNECT;
		if (BIT_CBUS_MHL12_DISCON_INT & int_4_status) {
			disconnect_mhl(hw_context, true);
			switch_to_d3(hw_context, false);
		} else {	
			disconnect_mhl(hw_context, false);
			switch_to_d3(hw_context, true);
		}
		ret_val = 0xFF;	

		if (hw_context->current_cbus_req.command) {
			hw_context->current_cbus_req.command = 0x00;
			hw_context->intr_info->flags |= DRV_INTR_MSC_DONE;
			hw_context->intr_info->msc_done_data = 0;
		}

	} else if (int_4_status & BIT_RGND_READY_INT) {
		int disc_stat2;

		disc_stat2 =
		    mhl_tx_read_reg(hw_context,
				 REG_DISC_STAT2)	&
				 MSK_DISC_STAT2_RGND;
		MHL_TX_DBG_ERR("Cable (RGND) impedance measured (%s)\n",
			       rgnd_value_string[disc_stat2]);

		if (VAL_RGND_1K == disc_stat2) {
			MHL_TX_DBG_WARN("Cable impedance = 1k (MHL Device)\n");

			
			enable_hdmi(true);

			mhl_tx_write_reg(hw_context, REG_DISC_CTRL9,
				 BIT_DISC_CTRL9_WAKE_DRVFLT |
				 BIT_DISC_CTRL9_DISC_PULSE_PROCEED);

#ifdef ENABLE_VBUS_SENSE 
			msleep(MHL_T_src_vbus_cbus_stable_min);
			if (get_config(hw_context, XO3_SINK_VBUS_SENSE)) {
				MHL_TX_DBG_WARN("%ssink drives VBUS%s\n",
						ANSI_ESC_GREEN_TEXT,
						ANSI_ESC_RESET_TEXT);
				mhl_tx_vbus_control(VBUS_OFF);
			} else {
				MHL_TX_DBG_WARN("%ssource drives VBUS%s\n",
						ANSI_ESC_YELLOW_TEXT,
						ANSI_ESC_RESET_TEXT);
				mhl_tx_vbus_control(VBUS_ON);
			}
#endif
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL4,
					 0x90);


			

			
			enable_intr(hw_context, INTR_DISC,
				    BIT_MHL3_EST_INT_MASK
				    | BIT_MHL_EST_INT_MASK
				    | BIT_NOT_MHL_EST_INT_MASK
				    | BIT_CBUS_MHL3_DISCON_INT_MASK
				    | BIT_CBUS_MHL12_DISCON_INT_MASK
				    | BIT_RGND_READY_INT_MASK);
			mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0,
				(VAL_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X |
				BIT_MHL_PLL_CTL0_CRYSTAL_CLK_SEL |
				BIT_MHL_PLL_CTL0_ZONE_MASK_OE));

			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL0, 0xC0);
			mhl_tx_write_reg(hw_context, REG_M3_CTRL,
				VAL_M3_CTRL_PEER_VERSION_PENDING_VALUE);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL1, 0xA2);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL2, 0x03);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL3, 0x35);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL5, 0x02);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL6, 0x02);
			mhl_tx_write_reg(hw_context,
				REG_MHL_DP_CTL7, 0x08);
			mhl_tx_write_reg(hw_context,
				REG_COC_CTLC, 0xFF);
			mhl_tx_write_reg(hw_context, REG_DPD,
					 BIT_DPD_PWRON_PLL |
					 BIT_DPD_PDNTX12 |
					 BIT_DPD_OSC_EN |
					 BIT_DPD_PWRON_HSIC);
			enable_intr(hw_context, INTR_COC,
				    BIT_COC_PLL_LOCK_STATUS_CHANGE |
				    BIT_COC_CALIBRATION_DONE);
			
			enable_intr(hw_context, INTR_MERR, (
#ifdef PRINT_DDC_ABORTS
				BIT_CBUS_DDC_ABORT |
#endif
				BIT_CBUS_MSC_ABORT_RCVD |
				BIT_CBUS_CMD_ABORT));
			enable_intr(hw_context, INTR_MSC,
				    (BIT_CBUS_MSC_MT_DONE | BIT_CBUS_HPD_CHG |
				     BIT_CBUS_MSC_MR_WRITE_STAT |
				     BIT_CBUS_MSC_MR_MSC_MSG |
				     BIT_CBUS_MSC_MR_WRITE_BURST |
				     BIT_CBUS_MSC_MR_SET_INT |
				     BIT_CBUS_MSC_MT_DONE_NACK));
		} else {
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL9,
				BIT_DISC_CTRL9_WAKE_DRVFLT |
				BIT_DISC_CTRL9_NOMHL_EST |
				BIT_DISC_CTRL9_WAKE_PULSE_BYPASS);
			
			enable_intr(hw_context, INTR_DISC,
				BIT_NOT_MHL_EST_INT_MASK
				| BIT_CBUS_MHL3_DISCON_INT_MASK
				| BIT_CBUS_MHL12_DISCON_INT_MASK
				| BIT_RGND_READY_INT_MASK);
		}
	} else if (int_4_status & BIT_MHL_EST_INT) {
		uint8_t msc_compat =
		    BIT_CBUS_MSC_COMPATIBILITY_CONTROL_ENABLE_XDEVCAP;

		MHL_TX_DBG_ERR("oCBUS Connection Established\n");

		hw_context->cbus_mode = CM_oCBUS_PEER_VERSION_PENDING;
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL4, 0x10);

		mhl_tx_write_reg(hw_context,
				 REG_CBUS_MSC_COMPATIBILITY_CONTROL,
				 msc_compat);

		init_regs(hw_context);

		hw_context->intr_info->flags |= DRV_INTR_CONNECT;
	}

	return ret_val;
}

static int g2wb_err_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	if (intr_stat) {
		if (BIT_MDT_RCV_TIMEOUT & intr_stat) {
			MHL_TX_DBG_WARN("%sBIT_MDT_RCV_TIMEOUT%s\n",
					ANSI_ESC_YELLOW_TEXT,
					ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_RCV_SM_ABORT_PKT_RCVD & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_RCV_SM_ABORT_PKT_RCVD%s\n",
				       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_RCV_SM_ERROR & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_RCV_SM_ERROR%s\n",
				       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_TIMEOUT & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_XMIT_TIMEOUT %s\n",
				       ANSI_ESC_YELLOW_TEXT,
				       ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_SM_ABORT_PKT_RCVD & intr_stat) {
			MHL_TX_DBG_ERR
			    ("%sBIT_MDT_XMIT_SM_ABORT_PKT_RCVD "
			       "- status: 0x%02x%s\n",
			     ANSI_ESC_RED_TEXT,
			     mhl_tx_read_reg(hw_context,
			       REG_MDT_SM_STAT),
			     ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_SM_ERROR & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_XMIT_SM_ERROR%s\n",
				       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		}
	}
	return 0;
}

static int g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	int ret_val = 0;
	if (intr_stat) {
		if (BIT_MDT_XFIFO_EMPTY & intr_stat) {
			struct cbus_req *peek_req;
			struct mhl_dev_context *dev_context;

			dev_context = container_of((void *)hw_context,
						   struct mhl_dev_context,
						   drv_context);

			MHL_TX_DBG_WARN
			    ("%sHAWB XFIFO empty%s XFIFO_STAT: 0x%02x\n",
			     ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT,
			     mhl_tx_read_reg(hw_context,
					     REG_MDT_XFIFO_STAT));

			peek_req = peek_next_cbus_transaction(dev_context);
			if (NULL == peek_req) {
				disable_gen2_write_burst_xmit(hw_context);
				disable_gen2_write_burst_rcv(hw_context);
			} else if (MHL_WRITE_BURST != peek_req->command) {
				disable_gen2_write_burst_xmit(hw_context);
				disable_gen2_write_burst_rcv(hw_context);
			}
		}
		if (BIT_MDT_RFIFO_DATA_RDY & intr_stat) {
			uint8_t length;
			uint8_t mdt_buffer[20];

			
			mhl_tx_read_reg_block(hw_context,
					      REG_MDT_RCV_READ_PORT,
					      17, mdt_buffer);

			MHL_TX_DBG_WARN
			    ("%sgot G2WB incoming 0x%02x%02x%s = %02X\n",
			     ANSI_ESC_GREEN_TEXT, mdt_buffer[1], mdt_buffer[2],
			     ANSI_ESC_RESET_TEXT, intr_stat);

			
			length = mdt_buffer[0];
			/*
			 * There is no way to know how much
			 * of the scratch pad was written so read
			 * it all.  The app. will have to parse
			 * the data to know how much of it is valid.
			 */
			memcpy(hw_context->write_burst_data, &mdt_buffer[1],
			       16);

			
			hw_context->intr_info->flags |= DRV_INTR_WRITE_BURST;

			mhl_tx_write_reg(hw_context,
				REG_MDT_RCV_CONTROL,
				BIT_MDT_RCV_CONTROL_MDT_RFIFO_CLR_CUR
				| BIT_MDT_RCV_CONTROL_MDT_RCV_EN
				| hw_context->
				delayed_hawb_enable_reg_val);
			{
				int rfifo_stat;
				rfifo_stat = mhl_tx_read_reg(hw_context,
						    REG_MDT_RFIFO_STAT);
				if (rfifo_stat &
				    MSK_MDT_RFIFO_STAT_MDT_RFIFO_CNT) {
					ret_val = BIT_MDT_RFIFO_DATA_RDY;
				} else {
					switch (hw_context->cbus1_state) {
					case CBUS1_IDLE_RCV_ENABLED:
					case CBUS1_IDLE_RCV_DISABLED:
						break;
					case CBUS1_IDLE_RCV_PEND:
						hw_context->cbus1_state =
						    CBUS1_IDLE_RCV_ENABLED;
						break;
					case CBUS1_MSC_PEND_DLY_RCV_EN:
						MHL_TX_DBG_ERR(
						    "%sCan't get here%s\n",
						    ANSI_ESC_RED_TEXT,
						    ANSI_ESC_RESET_TEXT)
						    
						break;
					case CBUS1_MSC_PEND_DLY_RCV_DIS:
						break;
					case CBUS1_XMIT_PEND_XMIT_RCV_EN:
						break;
					case CBUS1_XMIT_PEND_XMIT_RCV_PEND:
						hw_context->cbus1_state =
						    CBUS1_XMIT_PEND_XMIT_RCV_EN;
						break;
					}
				}
			}
		}
		if (BIT_MDT_IDLE_AFTER_HAWB_DISABLE & intr_stat) {
			MHL_TX_DBG_INFO(
						 "hawb_idle\n");
			    if (MHL_WRITE_BURST ==
				hw_context->current_cbus_req.command) {
				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |=
				    DRV_INTR_MSC_DONE;
				hw_context->intr_info->msc_done_data = 0;
				hw_context->hawb_write_pending = false;
				hw_context->cbus1_state =
				    CBUS1_IDLE_RCV_ENABLED;
			}
		}
	}
	return ret_val;
}

static void enable_intr(struct drv_hw_context *hw_context,
			uint8_t intr_num, uint8_t intr_mask)
{
	g_intr_tbl[intr_num].mask = intr_mask;
	mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].mask_addr, intr_mask);
}

static void si_mhl_tx_drv_enable_emsc_block(struct drv_hw_context *hw_context)
{
	uint8_t intStatus;
	MHL_TX_DBG_INFO("Enabling EMSC and EMSC interrupts\n");

	mhl_tx_modify_reg(hw_context, REG_GENCTL,
			  BIT_GENCTL_EMSC_EN |
			  BIT_GENCTL_CLR_EMSC_RFIFO |
			  BIT_GENCTL_CLR_EMSC_XFIFO,
			  BIT_GENCTL_EMSC_EN |
			  BIT_GENCTL_CLR_EMSC_RFIFO |
			  BIT_GENCTL_CLR_EMSC_XFIFO);
	mhl_tx_modify_reg(hw_context, REG_GENCTL,
			  BIT_GENCTL_CLR_EMSC_RFIFO |
			  BIT_GENCTL_CLR_EMSC_XFIFO, 0);
	mhl_tx_modify_reg(hw_context, REG_COMMECNT,
			  BIT_COMMECNT_I2C_TO_EMSC_EN,
			  use_spi ? 0 : BIT_COMMECNT_I2C_TO_EMSC_EN);
	intStatus = mhl_tx_read_reg(hw_context, REG_EMSCINTR);
	mhl_tx_write_reg(hw_context, REG_EMSCINTR, intStatus);
	enable_intr(hw_context, INTR_BLOCK, BIT_EMSCINTR_SPI_DVLD);
}

void si_mhl_tx_drv_device_isr(struct drv_hw_context *hw_context,
			      struct interrupt_info *intr_info)
{
	uint8_t intr_num;
	uint8_t aggregated_intr_status[NUM_AGGREGATED_INTR_REGS];

	hw_context->intr_info = intr_info;

	MHL_TX_DBG_INFO("%sgot INTR COC_STAT_0:0x%02x%s\n",
			ANSI_ESC_GREEN_TEXT,
			mhl_tx_read_reg(hw_context, REG_COC_STAT_0),
			ANSI_ESC_RESET_TEXT);

	MHL_TX_DBG_INFO("%sdiv_ctl_main:0x%02x hdcp2x_tp1:0x%02x%s\n",
			ANSI_ESC_GREEN_TEXT,
			mhl_tx_read_reg(hw_context, REG_DIV_CTL_MAIN),
			mhl_tx_read_reg(hw_context, REG_HDCP2X_TP1),
			ANSI_ESC_RESET_TEXT);

	mhl_tx_read_reg_block(hw_context, REG_FAST_INTR_STAT,
			      sizeof(aggregated_intr_status),
			      aggregated_intr_status);
	MHL_TX_DBG_INFO("%s aggr intr status:"
			" %02x %02x %02x %02x %02x %02x %02x%s\n",
			ANSI_ESC_GREEN_TEXT,
			aggregated_intr_status[FAST_INTR_STAT],
			aggregated_intr_status[L1_INTR_STAT_0],
			aggregated_intr_status[L1_INTR_STAT_1],
			aggregated_intr_status[L1_INTR_STAT_2],
			aggregated_intr_status[L1_INTR_STAT_3],
			aggregated_intr_status[L1_INTR_STAT_4],
			aggregated_intr_status[L1_INTR_STAT_5],
			ANSI_ESC_RESET_TEXT);

	
	for (intr_num = 0;
	     (intr_num < MAX_INTR) && (is_interrupt_asserted()); intr_num++) {
		if (g_intr_tbl[intr_num].mask) {
			uint8_t aggregated_index, aggregated_id_bit,
			    aggregated_status;
			aggregated_index = g_intr_tbl[intr_num].aggr_stat_index;
			aggregated_id_bit =
			    g_intr_tbl[intr_num].aggr_stat_id_bit;
			aggregated_status =
			    aggregated_intr_status[aggregated_index];
			if (aggregated_status & aggregated_id_bit) {
				int reg_value;
				uint8_t intr_stat;

				reg_value = mhl_tx_read_reg(hw_context,
							    g_intr_tbl
							    [intr_num].
							    stat_addr);

				if (reg_value < 0)
					return;

				intr_stat = (uint8_t) reg_value;

				
				intr_stat =
				    intr_stat & g_intr_tbl[intr_num].mask;
				if (intr_stat) {
					int already_cleared;

#ifdef PRINT_ALL_INTR
					MHL_TX_DBG_ERR("INTR-%s = %02X\n",
						       g_intr_tbl[intr_num].
						       name, intr_stat);
#else
					MHL_TX_DBG_INFO("INTR-%s = %02X\n",
							g_intr_tbl[intr_num].
							name, intr_stat);
#endif

					already_cleared =
					    g_intr_tbl[intr_num].isr(hw_context,
								     intr_stat);
					if (already_cleared >= 0) {
						intr_stat &= ~already_cleared;
						if (intr_stat) {
							mhl_tx_write_reg
							    (hw_context,
							     g_intr_tbl
							     [intr_num].
							     stat_addr,
							     intr_stat);
						}
					}
				}
			}
		}
#ifdef PRINT_ALL_INTR
		
		else {
			uint8_t intr_stat;
			intr_stat = mhl_tx_read_reg(hw_context,
						    g_intr_tbl[intr_num].
						    stat_page,
						    g_intr_tbl[intr_num].
						    stat_offset);
			MHL_TX_DBG_ERR("INTN-%s = %02X\n",
				       g_intr_tbl[intr_num].name, intr_stat);

		}
#endif
	}
}

int cb_enum_begin(void *context)
{
	MHL_TX_DBG_INFO("%senum begin%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
	return 0;
}

int cb_enum_item(void *context, uint16_t columns, uint16_t rows,
	uint8_t bits_per_pixel, uint32_t vertical_refresh_rate_in_milliHz,
	uint16_t burst_id, union video_burst_descriptor_u *p_descriptor)
{
	MHL_TX_DBG_INFO("%senum item%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
	return 0;
}

int cb_enum_end(void *context)
{
	MHL_TX_DBG_INFO("%senum end%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
	return 0;
}

void cb_hpd_driven_low(void *context)
{
	MHL_TX_DBG_WARN("%sdriven_low%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
}

enum hpd_high_callback_status cb_hpd_driven_high(void *context,
	uint8_t *p_edid, size_t edid_length,
	struct MHL3_hev_dtd_item_t *p_hev_dtd, size_t num_hev_dtds,
	struct MHL3_hev_vic_item_t *p_hev_vic, size_t num_hev_vics,
	struct MHL3_3d_dtd_item_t *p_3d_dtd_items, size_t num_3d_dtds,
	struct MHL3_3d_vic_item_t *p_3d_vic, size_t num_3d_vics,
	union avif_or_cea_861_dtd_u *p_avif_or_dtd,
	size_t avif_or_dtd_max_length, union vsif_mhl3_or_hdmi_u *p_vsif,
	size_t vsif_max_length)
{
	MHL_TX_DBG_WARN("%sdriven_high%s\n", ANSI_ESC_GREEN_TEXT,
			ANSI_ESC_RESET_TEXT);
	return HH_VIDEO_NOT_RDY;
}

static void input_field_rate_measurement_callback(void *callback_param)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)callback_param;
	start_video(hw_context);
}

int si_mhl_tx_chip_initialize(struct drv_hw_context *hw_context)
{
	int ret_val;
	int status = -1;

	hw_context->pp_16bpp_override = pp_16bpp_automatic;
	set_pin(hw_context, TX_FW_WAKE, 1);
	platform_mhl_tx_hw_reset(TX_HW_RESET_PERIOD, TX_HW_RESET_DELAY);

	
	mhl_tx_write_reg(hw_context, REG_DPD, BIT_DPD_PWRON_PLL
			 | BIT_DPD_PDNTX12 | BIT_DPD_OSC_EN);

	ret_val = get_device_rev(hw_context);
	hw_context->chip_rev_id = (uint8_t) ret_val;
	if (0 == hw_context->chip_rev_id) {
		MHL_TX_DBG_ERR("ES%s0.0%s is not supported\n",
			       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		return status;
	}

	si_set_cbus_mode_leds(CM_NO_CONNECTION);

	ret_val = get_device_id(hw_context);
	if (ret_val > 0) {
		struct mhl_dev_context *dev_context =
		    container_of((void *)hw_context,
				 struct mhl_dev_context, drv_context);

		hw_context->chip_device_id = (uint16_t) ret_val;

		MHL_TX_DBG_ERR("%p: Found SiI%04X rev: %01X.%01X\n",
			       hw_context,
			       hw_context->chip_device_id,
			       hw_context->chip_rev_id >> 4,
			       hw_context->chip_rev_id & 0x0F
		);
#ifndef SWWA_BZ30759
		
		program_ext_clock_regs(hw_context, crystal_khz);
#endif
		mhl_tx_read_reg_block(hw_context, REG_AKSV_1,
				      5, hw_context->aksv);

		disconnect_mhl(hw_context, true);
		switch_to_d3(hw_context, false);
		hw_context->callbacks.display_timing_enum_begin = cb_enum_begin;
		hw_context->callbacks.display_timing_enum_item = cb_enum_item;
		hw_context->callbacks.display_timing_enum_end = cb_enum_end;
		hw_context->callbacks.hpd_driven_low = cb_hpd_driven_low;
		hw_context->callbacks.hpd_driven_high = cb_hpd_driven_high;

		status = mhl_tx_create_timer(dev_context,
			input_field_rate_measurement_callback,
			hw_context,
			&hw_context->
			input_field_rate_measurement_timer);
		if (status != 0) {
			MHL_TX_DBG_ERR(
			"Failed to allocate FIELD_RATE measurement timer\n");
		} else {
			MHL_TX_DBG_INFO
			    ("stopping timer for FIELD_RATE measurement\n");
			mhl_tx_stop_timer(dev_context,
					  hw_context->
					  input_field_rate_measurement_timer);
		}
	}

	return status;
}

void si_mhl_tx_drv_shutdown(struct drv_hw_context *hw_context)
{
	set_pin(hw_context, TX_FW_WAKE, 0);
}

int si_mhl_tx_drv_connection_is_mhl3(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	return IN_MHL3_MODE(hw_context) ? 1 : 0;
}

int si_mhl_tx_drv_get_highest_tmds_link_speed(struct mhl_dev_context
					      *dev_context)
{
	int link_speed = MHL_XDC_TMDS_000;

	if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_600) {
		link_speed = MHL_XDC_TMDS_600;
	} else if (dev_context->xdev_cap_cache.mxdc.tmds_speeds &
		MHL_XDC_TMDS_300) {
		link_speed = MHL_XDC_TMDS_300;
	} else if (dev_context->xdev_cap_cache.mxdc.tmds_speeds &
		MHL_XDC_TMDS_150) {
		link_speed = MHL_XDC_TMDS_150;
	}

	return link_speed;
}

static void si_mhl_tx_drv_set_lowest_tmds_link_speed(struct mhl_dev_context
					      *dev_context,
					      uint32_t pixel_clock_frequency,
					      uint8_t bits_per_pixel)
{
	uint32_t link_clock_frequency;
	uint32_t top_clock_frequency;
	uint8_t reg_val;
	bool found_fit, fits_1_5, fits_3_0, fits_6_0;

	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;

	link_clock_frequency =
	    pixel_clock_frequency * ((uint32_t) (bits_per_pixel >> 3));
	fits_1_5 = false;
	fits_3_0 = false;
	fits_6_0 = false;


	if (link_clock_frequency <= 600000000) {
		MHL_TX_DBG_WARN(
		    "Mode fit TMDS Link Speed = 6.0Gbps (%d)\n",
		    link_clock_frequency);
		if (dev_context->xdev_cap_cache.mxdc.tmds_speeds
			& MHL_XDC_TMDS_600) {
			MHL_TX_DBG_INFO(
			    "XDEVCAP TMDS Link Speed = 6.0Gbps is supported\n");
			found_fit = true;
			fits_6_0 = true;
			top_clock_frequency = 588000000;
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_6GBPS;
		}
	}
	if (link_clock_frequency <= 300000000) {
		MHL_TX_DBG_WARN(
		    "Mode fits TMDS Link Speed = 3.0Gbps (%d)\n",
		    link_clock_frequency);
		if (dev_context->xdev_cap_cache.mxdc.tmds_speeds
			& MHL_XDC_TMDS_300) {
			MHL_TX_DBG_INFO(
			    "XDEVCAP TMDS Link Speed = 3.0Gbps is supported\n");
			found_fit = true;
			fits_3_0 = true;
			top_clock_frequency = 294000000;
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_3GBPS;
		}
	}
	if (link_clock_frequency <= 150000000) {
		MHL_TX_DBG_WARN("Mode fits TMDS Link Speed = 1.5Gbps (%d)\n",
		     link_clock_frequency);
		if (dev_context->xdev_cap_cache.mxdc.tmds_speeds
			& MHL_XDC_TMDS_150) {
			MHL_TX_DBG_INFO(
			    "XDEVCAP TMDS Link Speed = 1.5Gbps is supported\n");
			found_fit = true;
			fits_1_5 = true;
			top_clock_frequency = 147000000; 
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_1_5GBPS;

		}
	}
	if (!found_fit) {
		MHL_TX_DBG_ERR(
			"Cannot fit mode to any supported TMDS Link Speeds\n");
		MHL_TX_DBG_INFO("Forcing TMDS Link Speed = 6.0Gbps\n");
		top_clock_frequency = 588000000;	
		reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_6GBPS;
	}
	switch (platform_get_flags() & PLATFORM_FLAG_LINK_SPEED) {
	case PLATFORM_FLAG_1_5GBPS:
		if (fits_1_5) {
			MHL_TX_DBG_WARN("Module parameter forcing "
				"TMDS Link Speed = 1.5Gbps\n");
			top_clock_frequency = 147000000;
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_1_5GBPS;
		}
		break;
	case PLATFORM_FLAG_3GBPS:
		if (fits_3_0) {
			MHL_TX_DBG_WARN("Module parameter forcing "
			    "TMDS Link Speed = 3.0Gbps\n");
			top_clock_frequency = 294000000;
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_3GBPS;
		}
		break;
	case PLATFORM_FLAG_6GBPS:
		if (fits_6_0) {
			MHL_TX_DBG_WARN("Module parameter forcing "
				"TMDS Link Speed = 6.0Gbps\n");
			top_clock_frequency = 588000000;
			reg_val = VAL_TX_ZONE_CTL3_TX_ZONE_6GBPS;
		}
		break;
	}

	
	mhl_tx_write_reg(hw_context, REG_MHL3_TX_ZONE_CTL, reg_val);

	
	MHL_TX_DBG_WARN("lcf = %d, tcf = %d\n", link_clock_frequency,
			top_clock_frequency);
	if (link_clock_frequency >= top_clock_frequency) {
		MHL_TX_DBG_ERR("Program 3E1[3] = 1 --- UNLIMITED MODE ON\n");
		mhl_tx_modify_reg(hw_context, REG_M3_P0CTRL,
				  BIT_M3_P0CTRL_MHL3_P0_PORT_EN |
				  BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN,
				  BIT_M3_P0CTRL_MHL3_P0_PORT_EN |
				  BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_PP);
	} else {
		MHL_TX_DBG_ERR("Program 3E1[3] = 0 --- UNLIMITED MODE OFF\n");
		mhl_tx_modify_reg(hw_context, REG_M3_P0CTRL,
				  BIT_M3_P0CTRL_MHL3_P0_PORT_EN |
				  BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN,
				  BIT_M3_P0CTRL_MHL3_P0_PORT_EN |
				  BIT_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_NORM);
	}

	
	
	switch (reg_val) {
	case VAL_TX_ZONE_CTL3_TX_ZONE_6GBPS:
		mhl_tx_modify_reg(dev_context, REG_M3_POSTM,
			MSK_M3_POSTM_RRP_DECODE, 0x40);
		si_mhl_tx_set_status(dev_context, true,
			MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x02);
		break;
	case VAL_TX_ZONE_CTL3_TX_ZONE_3GBPS:
		mhl_tx_modify_reg(dev_context, REG_M3_POSTM,
			MSK_M3_POSTM_RRP_DECODE, 0x40);
		si_mhl_tx_set_status(dev_context, true,
			MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x01);
		break;
	case VAL_TX_ZONE_CTL3_TX_ZONE_1_5GBPS:
		mhl_tx_modify_reg(dev_context, REG_M3_POSTM,
			MSK_M3_POSTM_RRP_DECODE, 0x38);
		si_mhl_tx_set_status(dev_context, true,
			MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x00);
		break;
	}
}

bool si_mhl_tx_drv_support_e_cbus_d(struct drv_hw_context *hw_context)
{
	return false;
}

int si_mhl_tx_drv_cbus_ready_for_edid(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	switch (hw_context->cbus_mode) {
	case CM_oCBUS_PEER_IS_MHL1_2:
	case CM_eCBUS_S:
	case CM_eCBUS_D:
		return 1;
	default:
		return 0;
	}
}

uint16_t si_mhl_tx_drv_get_blk_rcv_buf_size(void)
{
	return LOCAL_BLK_RCV_BUFFER_SIZE;
}

int si_mhl_tx_drv_start_avlink_bist(struct drv_hw_context *hw_context,
				    struct bist_setup_info *test_info)
{
	enum hpd_control_mode mode;
	uint32_t frame_count = 0;
	uint8_t start_cmd;
	uint8_t bist_mode_sel;

	switch (test_info->avlink_video_mode) {
	case 4:		
		MHL_TX_DBG_ERR("AV LINK_VIDEO_MODE 720p60\n");
		bist_mode_sel = 5;
		break;
	case 3:		
		MHL_TX_DBG_ERR("AV LINK_VIDEO_MODE 480p60\n");
		bist_mode_sel = 6;
		break;
	default:
		MHL_TX_DBG_ERR("Unsupported VIC received!\n");
		return -1;
	}
	mhl_tx_write_reg(hw_context, REG_BIST_VIDEO_MODE, bist_mode_sel);

	mhl_tx_write_reg(hw_context, REG_DPD, 0xFE);
	mhl_tx_write_reg(hw_context, REG_TEST_TXCTRL, 0x00);
	mhl_tx_write_reg(hw_context, REG_DCTL, 0x02);
	mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x00);
	mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xF0);
	mhl_tx_write_reg(hw_context, REG_M3_P0CTRL, 0x01);
	mhl_tx_write_reg(hw_context, REG_COC_CTL7, 0x01);
	mhl_tx_write_reg(hw_context, REG_COC_CTL11, 0xF8);
	mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x80);
	mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x19);

	switch (test_info->avlink_data_rate) {
	case 1:		
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 1.5Gbps\n");
		bist_mode_sel = 0x02;
		break;
	case 2:		
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 3.0Gbps\n");
		bist_mode_sel = 0x01;
		break;
	case 3:		
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 6.0Gbps\n");
		bist_mode_sel = 0x00;
		break;
	default:
		MHL_TX_DBG_ERR("Unsupported AVLINK_DATA_RATE %02X\n",
			       test_info->avlink_data_rate);
		return -1;
	}
	mhl_tx_write_reg(hw_context, REG_MHL3_TX_ZONE_CTL,
			 bist_mode_sel);

	switch (test_info->avlink_pattern) {
	case BIST_AVLINK_PATTERN_UNSPECIFIED:
	case BIST_AVLINK_PATTERN_PRBS:
		MHL_TX_DBG_ERR("AV LINK_PATTERN PRBS\n");
		mhl_tx_write_reg(hw_context, REG_BIST_TEST_SEL, 0x03);
		break;

	case BIST_AVLINK_PATTERN_FIXED_8:
		MHL_TX_DBG_ERR("AV LINK_PATTERN Fixed8\n");
		mhl_tx_write_reg(hw_context, REG_BIST_8BIT_PATTERN,
				 (uint8_t) test_info->avlink_fixed_pat);

		mhl_tx_write_reg(hw_context, REG_BIST_TEST_SEL, 0x09);
		break;

	case BIST_AVLINK_PATTERN_FIXED_10:
		MHL_TX_DBG_ERR("AV LINK_PATTERN Fixed10\n");
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_LOW,
			0x01);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_HIGH,
			0x07);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_PAT_LOW,
				(uint8_t) test_info->avlink_fixed_pat);
		{
			uint8_t STX_1, STX_0, STX, E98_pre, pat_lsb, pat_msb;
			pat_lsb = (uint8_t) test_info->avlink_fixed_pat;
			pat_msb = (uint8_t) (test_info->avlink_fixed_pat >> 8);
			STX_1 =
			    ((pat_lsb >> 1) ^ (pat_lsb >> 3) ^ (pat_lsb >> 4) ^
			     (pat_lsb >> 5) ^ (pat_lsb >> 7)) & 0x01;
			STX_0 =
			    ((pat_lsb >> 0) ^ (pat_lsb >> 2) ^ (pat_lsb >> 4) ^
			     (pat_lsb >> 6)) & 0x01;
			STX = (STX_1 << 1) | STX_0;
			E98_pre = ((pat_msb) ^ STX) & 0x3;
			mhl_tx_write_reg(hw_context,
				REG_TX_IP_BIST_PAT_HIGH, E98_pre);
		}
		
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
			0x03);
		break;

	default:
		MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
		return -1;
	}

	mode = platform_get_hpd_control_mode();

	if (HPD_CTRL_OPEN_DRAIN == mode)
		mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				 BITS_HPD_CTRL_OPEN_DRAIN_HIGH);
	else if (HPD_CTRL_PUSH_PULL == mode)
		mhl_tx_write_reg(hw_context, REG_HPD_CTRL,
				 BITS_HPD_CTRL_PUSH_PULL_HIGH);

	if (test_info->avlink_duration) {
		if (test_info->avlink_pattern == BIST_AVLINK_PATTERN_FIXED_8)
			frame_count = test_info->avlink_duration * 32;
	}

	mhl_tx_write_reg(hw_context, REG_BIST_DURATION_0,
			 (uint8_t) frame_count);
	frame_count >>= 8;
	mhl_tx_write_reg(hw_context, REG_BIST_DURATION_1,
			 (uint8_t) frame_count);
	frame_count >>= 8;
	mhl_tx_write_reg(hw_context, REG_BIST_DURATION_2,
			 (uint8_t) frame_count);

	start_cmd = BIT_BIST_EN |
	    BIT_BIST_ALWAYS_ON |
	    BIT_BIST_TRANS;
	mhl_tx_write_reg(hw_context, REG_BIST_CTRL, start_cmd);

	start_cmd |= BIT_BIST_START_BIT;
	mhl_tx_write_reg(hw_context, REG_BIST_CTRL, start_cmd);

	return 0;
}

int si_mhl_tx_drv_stop_avlink_bist(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_BIST_CTRL, 0x00);
	
	mhl_tx_modify_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
			  BIT_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
			  BIT_TX_IP_BIST_CNTLSTA_TXBIST_EN, 0x00);

	return 0;
}

int si_mhl_tx_drv_start_ecbus_bist(struct drv_hw_context *hw_context,
				   struct bist_setup_info *test_info)
{
	uint8_t start_cmd;
	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
				   struct mhl_dev_context, drv_context);

	if (dev_context->bist_trigger_info & BIST_TRIGGER_TEST_E_CBUS_D) {
		
		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, 0x60);

		switch (test_info->e_cbus_pattern) {
		case BIST_ECBUS_PATTERN_UNSPECIFIED:
		case BIST_ECBUS_PATTERN_PRBS:
			start_cmd = 0x01;
			break;

		case BIST_ECBUS_PATTERN_FIXED_8:
			mhl_tx_write_reg(hw_context, REG_DOC_CTL9,
					 (uint8_t) test_info->e_cbus_fixed_pat);

			mhl_tx_write_reg(hw_context, REG_DOC_CTLA,
					 (uint8_t) test_info->e_cbus_fixed_pat);
			start_cmd = 0x02;
			break;

		case BIST_ECBUS_PATTERN_FIXED_10:
			mhl_tx_write_reg(hw_context, REG_DOC_CTL9,
					 (uint8_t) test_info->e_cbus_fixed_pat);
			mhl_tx_write_reg(hw_context, REG_DOC_CTL8,
					 ((uint8_t)
					  (test_info->
					   e_cbus_fixed_pat >> 8)) & 0x03);

			start_cmd =
			    (uint8_t) ((test_info->
					e_cbus_fixed_pat & 0x0300) >> 6);
			if (test_info->e_cbus_fixed_pat & 0x8000) {
				mhl_tx_write_reg(hw_context,
						 REG_DOC_CTLA,
						 ~(uint8_t) test_info->
						 e_cbus_fixed_pat);
				mhl_tx_modify_reg(hw_context,
						  REG_DOC_CTL8, 0x0C,
						  ~start_cmd);
			} else {
				mhl_tx_write_reg(hw_context,
						 REG_DOC_CTLA,
						 (uint8_t) test_info->
						 e_cbus_fixed_pat);
				mhl_tx_modify_reg(hw_context,
						  REG_DOC_CTL8, 0x0C,
						  start_cmd);
			}
			start_cmd = 0x03;
			break;
		default:
			MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
			return -1;
		}

		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, start_cmd);

	} else {

		
		mhl_tx_write_reg(hw_context, REG_COC_CTL7, 0x60);

		switch (test_info->e_cbus_pattern) {
		case BIST_ECBUS_PATTERN_UNSPECIFIED:
		case BIST_ECBUS_PATTERN_PRBS:
			start_cmd = 0x01;
			break;

		case BIST_ECBUS_PATTERN_FIXED_8:
			mhl_tx_write_reg(hw_context, REG_COC_CTL9,
					 (uint8_t) test_info->e_cbus_fixed_pat);
			mhl_tx_write_reg(hw_context, REG_COC_CTLA,
					 (uint8_t) test_info->e_cbus_fixed_pat);
			start_cmd = 0x02;
			break;

		case BIST_ECBUS_PATTERN_FIXED_10:

		default:
			MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
			return -1;
		}

		mhl_tx_write_reg(hw_context, REG_COC_CTL7, start_cmd);
		mhl_tx_write_reg(hw_context, REG_TTXNUMB, 0x05);

		mhl_tx_modify_reg(hw_context, REG_COC_CTL0,
				  BIT_COC_CTL0_COC_CONTROL0_2, 0x00);

		if (dev_context->bist_trigger_info & BIST_TRIGGER_E_CBUS_RX) {
			
			mhl_tx_write_reg(hw_context, REG_PWD_SRST, 0xA0);
			mhl_tx_write_reg(hw_context, REG_PWD_SRST, 0x20);

			mhl_tx_modify_reg(hw_context, REG_COC_CTL0,
					  BIT_COC_CTL0_COC_CONTROL0_2,
					  BIT_COC_CTL0_COC_CONTROL0_2);
			MHL_TX_DBG_ERR("REG_COC_CTL0 = 0x%02x\n",
				       mhl_tx_read_reg(hw_context,
						       REG_COC_CTL0));
		} else {
			mhl_tx_modify_reg(hw_context, REG_COC_CTL14,
				  MSK_COC_CTL14_COC_CONTROL14_3_0,
				  0x02);
			mhl_tx_modify_reg(hw_context, REG_COC_CTL15,
					  BIT_COC_CTL15_COC_CONTROL15_7,
					  BIT_COC_CTL15_COC_CONTROL15_7);
		}
	}
	return 0;
}

int si_mhl_tx_drv_stop_ecbus_bist(struct drv_hw_context *hw_context,
				  struct bist_setup_info *test_info)
{
	int err_cnt;
	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
				   struct mhl_dev_context, drv_context);

	if (dev_context->bist_trigger_info & BIST_TRIGGER_TEST_E_CBUS_D) {
		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, 0x00);
		err_cnt = mhl_tx_read_reg(hw_context, REG_DOC_STAT_9) &
			0x0F;
		err_cnt <<= 8;
		err_cnt |= mhl_tx_read_reg(hw_context, REG_DOC_STAT_8);
	} else {
		
		mhl_tx_modify_reg(hw_context, REG_COC_CTL15,
				  MSK_COC_CTL15_COC_CONTROL15_6_4, 0x00);
		err_cnt = mhl_tx_read_reg(hw_context, REG_COC_STAT_4);

		err_cnt <<= 8;
		err_cnt |= mhl_tx_read_reg(hw_context, REG_COC_STAT_5);
		MHL_TX_DBG_INFO
		    ("eCBUS State Machine: "
		     "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_0),
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_1),
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_2),
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_3),
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_4),
		     mhl_tx_read_reg(hw_context, REG_COC_STAT_5));

		mhl_tx_write_reg(hw_context, REG_TTXNUMB, 0x0C);

		mhl_tx_write_reg(hw_context, REG_COC_CTL7, 0x00);
	}

	return err_cnt;
}

int si_mhl_tx_drv_start_impedance_bist(struct drv_hw_context *hw_context,
				       struct bist_setup_info *test_info)
{
	int status = 0;

	switch (test_info->impedance_mode) {
	case BIST_IMPEDANCE_MODE_AVLINK_TX_LOW:
	case BIST_IMPEDANCE_MODE_AVLINK_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_LOW,
				 0x01);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_HIGH,
				 0x07);
		if (test_info->impedance_mode ==
		    BIST_IMPEDANCE_MODE_AVLINK_TX_LOW) {
			mhl_tx_write_reg(hw_context,
					 REG_TX_IP_BIST_PAT_LOW, 0x00);
			mhl_tx_write_reg(hw_context,
					 REG_TX_IP_BIST_PAT_HIGH, 0x00);
		} else {
			mhl_tx_write_reg(hw_context,
					 REG_TX_IP_BIST_PAT_LOW, 0xFF);
			mhl_tx_write_reg(hw_context,
					 REG_TX_IP_BIST_PAT_HIGH, 0x01);

		}
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_EN);
		break;

	case BIST_IMPEDANCE_MODE_ECBUS_S_TX_LOW:
	case BIST_IMPEDANCE_MODE_ECBUS_S_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_TEST_TXCTRL, 0x81);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x00);
		mhl_tx_write_reg(hw_context, REG_M3_CTRL, 0x0E);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x0B);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3D);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xF0);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL0, 0xC5);
		mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x18);

		if (test_info->impedance_mode ==
		    BIST_IMPEDANCE_MODE_ECBUS_S_TX_LOW) {
			mhl_tx_write_reg(hw_context, REG_COC_CTL1, 0x00);
			mhl_tx_write_reg(hw_context, REG_COC_CTL2, 0x00);
		} else {
			mhl_tx_write_reg(hw_context, REG_COC_CTL1, 0x28);
			mhl_tx_write_reg(hw_context, REG_COC_CTL2, 0x28);
		}

		mhl_tx_write_reg(hw_context, REG_TTXNUMB, 0x05);
		mhl_tx_write_reg(hw_context, REG_COC_CTLB, 0xFF);
		mhl_tx_write_reg(hw_context, REG_COC_CTLC, 0xFF);
		mhl_tx_write_reg(hw_context, REG_COC_CTLD, 0x00);
		mhl_tx_write_reg(hw_context, REG_COC_CTLE, 0x18);
		mhl_tx_write_reg(hw_context, REG_ALICE0_ZONE_CTRL, 0xE8);
		mhl_tx_write_reg(hw_context, REG_BGR_BIAS, 0x87);
		mhl_tx_write_reg(hw_context, REG_ALICE0_ZONE_CTRL, 0xD0);
		mhl_tx_write_reg(hw_context, REG_COC_CTL3, 0x3F);
		mhl_tx_write_reg(hw_context, REG_COC_CTL6, 0x10);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0xCD);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL4, 0x2A);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x80);
		mhl_tx_write_reg(hw_context, REG_M3_SCTRL, 0x40);
		mhl_tx_write_reg(hw_context, REG_M3_P0CTRL,
				 BIT_M3_P0CTRL_MHL3_P0_PORT_EN);

		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_LOW,
				 0x01);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_INST_HIGH,
				 0x03);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_PAT_LOW,
				 0xFF);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_PAT_HIGH,
				 0x03);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CONF_LOW,
				 0x08);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CONF_HIGH,
				 0x00);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_EN);
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_EN |
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_RUN |
				 BIT_TX_IP_BIST_CNTLSTA_TXBIST_ON);

		mhl_tx_write_reg(hw_context, REG_COC_CTL11, 0xF0);
		mhl_tx_write_reg(hw_context, REG_COC_CTL7, 0x01);
		mhl_tx_write_reg(hw_context, REG_COC_CTL14, 0x0C);
		mhl_tx_write_reg(hw_context, REG_COC_CTL15, 0x80);
		break;

	case BIST_IMPEDANCE_MODE_ECBUS_D_TX_LOW:
	case BIST_IMPEDANCE_MODE_ECBUS_D_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_TEST_TXCTRL, 0x01);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0, 0x02);
		mhl_tx_write_reg(hw_context, REG_M3_CTRL, 0x0E);
		mhl_tx_write_reg(hw_context, REG_MHL_TOP_CTL, 0x82);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xF3);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0x07);
		mhl_tx_write_reg(hw_context, REG_MHL_DOC_CTL0, 0x81);
		mhl_tx_write_reg(hw_context, REG_MHL3_TX_ZONE_CTL, 0x00);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x0B);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3D);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL0, 0x02);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, 0x06);
		mhl_tx_write_reg(hw_context, REG_DOC_CTLE, 0x0F);

		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x00);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x08);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x09);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x0A);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x0B);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_DOC_CFG4, 0x02);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x80);
		if (test_info->impedance_mode ==
		    BIST_IMPEDANCE_MODE_ECBUS_D_TX_HIGH)
			mhl_tx_write_reg(hw_context, REG_DOC_CTL6, 0x60);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, 0x1E);
		mhl_tx_write_reg(hw_context, REG_DOC_CTLE, 0x0F);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL0, 0x60);
		break;

	default:
		MHL_TX_DBG_ERR("Invalid value 0x%02x specified in "
			       "IMPEDANCE_MODE field\n",
			       test_info->impedance_mode);
		status = -EINVAL;
	}
	return status;
}

void si_mhl_tx_drv_stop_impedance_bist(struct drv_hw_context *hw_context,
				       struct bist_setup_info *test_info)
{
	switch (test_info->impedance_mode) {
	case BIST_IMPEDANCE_MODE_AVLINK_TX_LOW:
	case BIST_IMPEDANCE_MODE_AVLINK_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA, 0);
		break;

	case BIST_IMPEDANCE_MODE_ECBUS_S_TX_LOW:
	case BIST_IMPEDANCE_MODE_ECBUS_S_TX_HIGH:
		
		mhl_tx_write_reg(hw_context, REG_TEST_TXCTRL, 0x80);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x00);
		mhl_tx_write_reg(hw_context, REG_M3_CTRL, 0x07);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x2F);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3F);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xBC);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL0, 0xC3);
		mhl_tx_write_reg(hw_context, REG_COC_CTL0, 0x5C);
		mhl_tx_write_reg(hw_context, REG_COC_CTL1, 0x0A);
		mhl_tx_write_reg(hw_context, REG_COC_CTL2, 0x14);

		mhl_tx_write_reg(hw_context, REG_TTXNUMB, 0x04);
		mhl_tx_write_reg(hw_context, REG_COC_CTLB, 0xFF);
		mhl_tx_write_reg(hw_context, REG_COC_CTLC, 0xFF);
		mhl_tx_write_reg(hw_context, REG_COC_CTLD, 0x21);
		mhl_tx_write_reg(hw_context, REG_COC_CTLE, 0x18);

		mhl_tx_write_reg(hw_context, REG_ALICE0_ZONE_CTRL, 0xE8);
		mhl_tx_write_reg(hw_context, REG_BGR_BIAS, 0x87);
		mhl_tx_write_reg(hw_context, REG_COC_CTL3, 0x40);
		mhl_tx_write_reg(hw_context, REG_COC_CTL6, 0x00);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL1, 0xC7);
		mhl_tx_write_reg(hw_context, REG_MHL_COC_CTL4, 0x2A);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL2, 0x80);
		mhl_tx_write_reg(hw_context, REG_M3_SCTRL, 0x41);
		mhl_tx_write_reg(hw_context, REG_M3_P0CTRL,
				 BIT_M3_P0CTRL_MHL3_P0_HDCP_EN);

		mhl_tx_write_reg(hw_context, REG_TX_IP_BIST_CNTLSTA,
				 0x00);
		mhl_tx_write_reg(hw_context, REG_COC_CTL7, 0x00);
		mhl_tx_write_reg(hw_context, REG_COC_CTL14, 0x00);
		break;

	case BIST_IMPEDANCE_MODE_ECBUS_D_TX_LOW:
	case BIST_IMPEDANCE_MODE_ECBUS_D_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_TEST_TXCTRL, 0x80);
		mhl_tx_write_reg(hw_context, REG_MHL_PLL_CTL0, 0x07);
		mhl_tx_write_reg(hw_context, REG_M3_CTRL, 0x07);
		mhl_tx_write_reg(hw_context, REG_MHL_TOP_CTL, 0x00);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL0, 0xBC);
		mhl_tx_write_reg(hw_context, REG_MHL_DOC_CTL0, 0x00);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL2, 0x2F);
		mhl_tx_write_reg(hw_context, REG_MHL_DP_CTL5, 0x3F);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL0, 0x40);
		mhl_tx_write_reg(hw_context, REG_DOC_CTL7, 0x00);
		mhl_tx_write_reg(hw_context, REG_DOC_CTLE, 0x00);
	}
}

void si_mhl_tx_drv_leave_bist_mode(struct drv_hw_context *hw_context)
{
	disconnect_mhl(hw_context, true);
}

int si_mhl_tx_drv_sample_edid_buffer(struct drv_hw_context *hw_context,
				     uint8_t *edid_buf)
{
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);
	mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA, 256,
			      edid_buf);
	return 0;
}

static void dump_page(struct drv_hw_context *hw_context, uint8_t slave_addr)
{
	int i;
	for (i = 0; i < 256; i++) {
		if (0 == (i & 0x0F))
			printk(KERN_DEFAULT "0x%02X:", i);

		if (8 == (i & 0x0F))
			printk("-");

		printk(KERN_DEFAULT " %02X ", mhl_tx_read_reg(hw_context,
			(slave_addr << 8) | (uint8_t)i));
		if (0x0F == (0x0F & i))
			printk(KERN_DEFAULT "\n");
	}
}

void si_dump_important_regs(struct drv_hw_context *hw_context)
{
	int i = 0;
	uint16_t registers[] = {
		REG_DEV_REV,
		REG_OTP_DBYTE510,
		REG_DPD,
		REG_TMDS_CCTRL,
		REG_TMDS_CTRL4,
		REG_TRXCTRL,
		REG_OTP_DBYTE510,
		REG_MHL_INT_0_MASK,
		REG_MHL_INT_1_MASK,
		REG_MHL_INT_2_MASK,
		REG_MHL_INT_3_MASK,
		REG_MDT_RCV_TIMEOUT,
		REG_MDT_XMIT_TIMEOUT,
		REG_MDT_RCV_CONTROL,
		REG_MDT_XMIT_CONTROL,
		REG_MDT_RFIFO_STAT,
		REG_MDT_XFIFO_STAT,
		REG_MDT_INT_0,
		REG_MDT_INT_0_MASK,
		REG_MDT_INT_1_MASK,
		REG_CBUS_STATUS,
		REG_DDC_ABORT_INT_MASK,
		REG_MSC_MT_ABORT_INT_MASK,
		REG_MSC_MR_ABORT_INT_MASK,
		REG_CBUS_RX_DISC_INT0_MASK,
		0x0000
	};
	MHL_TX_DBG_ERR("Important Registers:\n");
	while (registers[i]) {
		MHL_TX_DBG_ERR("%02X:%02X = %02X\n",
		       (uint8_t)(registers[i] >> 8), (uint8_t)registers[i],
		       mhl_tx_read_reg(hw_context, registers[i]));
		i++;
	}

	printk(KERN_DEFAULT "\nDump PAGE_0:\n");
	dump_page(hw_context, SA_TX_PAGE_0);

	printk(KERN_DEFAULT "\nDump PAGE_1:\n");
	dump_page(hw_context, SA_TX_PAGE_1);

	printk(KERN_DEFAULT "\nDump PAGE_2:\n");
	dump_page(hw_context, SA_TX_PAGE_2);

	printk(KERN_DEFAULT "\nDump PAGE_3:\n");
	dump_page(hw_context, SA_TX_PAGE_3);

	printk(KERN_DEFAULT "\nDump PAGE_4:\n");
	dump_page(hw_context, SA_TX_PAGE_4);

	printk(KERN_DEFAULT "\nDump PAGE_5:\n");
	dump_page(hw_context, SA_TX_CBUS);

	printk(KERN_DEFAULT "\nDump PAGE_6:\n");
	dump_page(hw_context, SA_TX_PAGE_6);

	printk(KERN_DEFAULT "\nDump PAGE_7:\n");
	dump_page(hw_context, SA_TX_PAGE_7);
}

int si_mhl_tx_drv_get_pp_16bpp_override(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	return (int)hw_context->pp_16bpp_override;
}
void si_mhl_tx_drv_set_pp_16bpp_override(struct mhl_dev_context *dev_context,
		int override)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	hw_context->pp_16bpp_override = (enum pp_16bpp_override_t)override;
}

int si_mhl_tx_drv_get_hpd_status(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	int ret_val;

	ret_val = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if (ret_val < 0)
		return 0;
	else if (ret_val & BIT_CBUS_STATUS_CBUS_HPD)
		return 1;
	else
		return 0;
}

uint32_t si_mhl_tx_drv_get_hdcp2_status(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context =
	    (struct drv_hw_context *)&dev_context->drv_context;
	uint32_t ret_val;

	uint8_t ro_gp0;
	uint8_t ro_auth[2];

	

	ro_gp0 = mhl_tx_read_reg(hw_context, REG_HDCP2X_GP_OUT0);
	mhl_tx_read_reg_block(hw_context, REG_HDCP2X_AUTH_STAT,
			      sizeof(ro_auth), ro_auth);
	ret_val = (ro_gp0     << 16)
		 |(ro_auth[1] <<  8)
		 | ro_auth[0];
	return ret_val;

}
