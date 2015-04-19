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
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "platform.h"
#include "mhl_rcp_inputdev.h"

enum rcp_state_e {
	PH0_IDLE,
	PH3_PRESS_AND_HOLD_KEY,
	ph8_hold_mode,
	num_rcp_states
};

static char *state_strings[num_rcp_states] = {
	"idle",
	"press_and_hold_key",
	"hold_mode"
};

enum rcp_event_e {
	RCP_NORMAL_KEY_PRESS,
	RCP_NORMAL_KEY_PRESS_SAME,
	RCP_NORMAL_KEY_RELEASE,
	RCP_NORMAL_KEY_RELEASE_SAME,
	RCP_HOLD_KEY_PRESS,
	RCP_HOLD_KEY_PRESS_SAME,
	RCP_HOLD_KEY_RELEASE,
	RCP_HOLD_KEY_RELEASE_SAME,
	RCP_T_HOLD_MAINTAIN_EXPIRED,
	RCP_T_PRESS_MODE_EXPIRED,
	NUM_RCP_EVENTS
};

static char *event_strings[NUM_RCP_EVENTS] = {
	"normal_key_press",
	"normal_key_press_same",
	"normal_key_release",
	"normal_key_release_same",
	"press_and_hold_key_press",
	"press_and_hold_key_press_same",
	"press_and_hold_key_release",
	"press_and_hold_key_release_same",
	"rcp_T_hold_maintain_expired",
	"rcp_T_press_mode_expired"
};

enum rcp_state_e current_rcp_state = PH0_IDLE;
uint8_t rcp_previous_key = 0, rcp_current_key = 0;

struct rcp_keymap_t rcpSupportTable[MHL_NUM_RCP_KEY_CODES] = {
	{0, 0, 0, {KEY_SELECT,	0}, (MHL_DEV_LD_GUI)}, 
	{0, 1, 0, {KEY_UP,	0}, (MHL_DEV_LD_GUI)}, 
	{0, 1, 0, {KEY_DOWN,	0}, (MHL_DEV_LD_GUI)}, 
	{0, 1, 0, {KEY_LEFT,	0}, (MHL_DEV_LD_GUI)}, 
	{0, 1, 0, {KEY_RIGHT,	0}, (MHL_DEV_LD_GUI)}, 

	{1, 1, 0, {KEY_RIGHT, KEY_UP},   (MHL_DEV_LD_GUI)}, 
	{1, 1, 0, {KEY_RIGHT, KEY_DOWN}, (MHL_DEV_LD_GUI)}, 
	{1, 1, 0, {KEY_LEFT,  KEY_UP},   (MHL_DEV_LD_GUI)}, 
	{1, 1, 0, {KEY_LEFT,  KEY_DOWN}, (MHL_DEV_LD_GUI)}, 

	{0, 0, 0, {KEY_MENU, 0}, (MHL_DEV_LD_GUI)}, 
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0}, 
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0}, 
	{0, 0, 0, {KEY_BOOKMARKS, 0}, 0}, 
	{0, 0, 0, {KEY_EXIT, 0}, (MHL_DEV_LD_GUI)}, 

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 0, 0, {KEY_NUMERIC_0, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_1, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_2, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_3, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_4, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_5, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_6, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_7, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_8, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_NUMERIC_9, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},

	
	{0, 0, 0, {KEY_DOT, 0}, 0},

	
	{0, 0, 0, {KEY_ENTER, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_CLEAR, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO |
		MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 1, 0, {KEY_CHANNELUP, 0}, (MHL_DEV_LD_TUNER)},
	
	{0, 1, 0, {KEY_CHANNELDOWN, 0}, (MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_UNKNOWN, 0}, (MHL_DEV_LD_TUNER)},
	
	{0, 0, 0, {KEY_SOUND, 0}, (MHL_DEV_LD_AUDIO)},
	
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},
	
	{0, 0, 0, {KEY_PROGRAM, 0}, 0},
	
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},
	
	{0, 1, 0, {KEY_PAGEUP, 0}, 0},
	
	{0, 1, 0, {KEY_PAGEDOWN, 0}, 0},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 1, 0, {KEY_VOLUMEUP, 0}, (MHL_DEV_LD_SPEAKER)},
	
	{0, 1, 0, {KEY_VOLUMEDOWN, 0}, (MHL_DEV_LD_SPEAKER)},
	
	{0, 0, 0, {KEY_MUTE, 0}, (MHL_DEV_LD_SPEAKER)},
	
	{0, 0, 0, {KEY_PLAY, 0}, (MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)},
	
	{0, 0, 0, {KEY_STOP, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)},
	
	{0, 0, 0, {KEY_PLAYPAUSE, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)},
	
	{0, 0, 0, {KEY_RECORD, 0}, (MHL_DEV_LD_RECORD)},
	
	{0, 1, 0, {KEY_REWIND, 0}, (MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)},
	
	{0, 1, 0, {KEY_FASTFORWARD, 0}, (MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)},
	
	{0, 0, 0, {KEY_EJECTCD, 0}, (MHL_DEV_LD_MEDIA)},
	
	{0, 1, 0, {KEY_NEXTSONG, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA)},
	
	{0, 1, 0, {KEY_PREVIOUSSONG, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA)},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},
	
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 0, 0, {KEY_PLAYPAUSE, 0}, (MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)},
	
	{0, 0, 0, {KEY_PLAYPAUSE, 0}, (MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)},
	
	{0, 0, 0, {KEY_RECORD, 0}, (MHL_DEV_LD_RECORD)},
	
	{0, 0, 0, {KEY_PAUSE, 0}, (MHL_DEV_LD_RECORD)},
	
	{0, 0, 0, {KEY_STOP, 0},
		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)},
	
	{0, 0, 0, {KEY_MUTE, 0}, (MHL_DEV_LD_SPEAKER)},
	
	{0, 0, 0, {KEY_MUTE, 0}, (MHL_DEV_LD_SPEAKER)},

	
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},
	{0, 0, 0, {KEY_UNKNOWN, 0}, 0},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 0, 0, {KEY_F1, 0}, 0},
	{0, 0, 0, {KEY_F2, 0}, 0},
	{0, 0, 0, {KEY_F3, 0}, 0},
	{0, 0, 0, {KEY_F4, 0}, 0},
	{0, 0, 0, {KEY_F5, 0}, 0},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},
	{0, 0, 0, {KEY_RESERVED, 0}, 0},

	
	{0, 0, 0, {KEY_VENDOR, 0}, 0},

	
	{0, 0, 0, {KEY_RESERVED, 0}, 0}
};

static u16 rcp_def_keymap[MHL_NUM_RCP_KEY_CODES]
#ifdef OLD_KEYMAP_TABLE
	= {
	KEY_SELECT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_MENU,
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_EXIT,
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_NUMERIC_0,
	KEY_NUMERIC_1,
	KEY_NUMERIC_2,
	KEY_NUMERIC_3,
	KEY_NUMERIC_4,
	KEY_NUMERIC_5,
	KEY_NUMERIC_6,
	KEY_NUMERIC_7,
	KEY_NUMERIC_8,
	KEY_NUMERIC_9,
	KEY_DOT,
	KEY_ENTER,
	KEY_CLEAR,
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_RESERVED,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_PLAY,
	KEY_STOP,
	KEY_PLAYPAUSE,
	KEY_UNKNOWN,		
	KEY_REWIND,
	KEY_FASTFORWARD,
	KEY_UNKNOWN,		
	KEY_NEXTSONG,
	KEY_PREVIOUSSONG,
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_PLAY,
	KEY_PAUSE,
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_STOP,
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_UNKNOWN,		
	KEY_RESERVED,		
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,		
	KEY_VENDOR,
	KEY_RESERVED,		
}
#endif
;

#ifdef OLD_KEYMAP_TABLE
int generate_rcp_input_event(struct mhl_dev_context *dev_context,
			     uint8_t rcp_keycode)
{
	int status = -EINVAL;

	if (dev_context->rcp_input_dev) {
		if (rcp_keycode < ARRAY_SIZE(rcp_def_keymap) &&
		    rcp_def_keymap[rcp_keycode] != KEY_UNKNOWN &&
		    rcp_def_keymap[rcp_keycode] != KEY_RESERVED) {

			input_report_key(dev_context->rcp_input_dev,
					 rcp_keycode, 1);
			input_report_key(dev_context->rcp_input_dev,
					 rcp_keycode, 0);
			input_sync(dev_context->rcp_input_dev);

			status = 0;
		}
	}
	return status;
}
#else

static int rcp_trigger_key_action(struct mhl_dev_context *dev_context,
				  uint8_t index, bool press_release)
{
	int status = -EINVAL;

	if (dev_context->rcp_input_dev) {
		input_report_key(dev_context->rcp_input_dev, index,
				 press_release);
		input_sync(dev_context->rcp_input_dev);
		status = 0;
	}
	return status;
}

static int handle_rcp_event(struct mhl_dev_context *dev_context,
	uint8_t current_key, uint8_t prev_key, enum rcp_event_e event)
{
	int status = 0;
	uint8_t current_index = current_key & MHL_RCP_KEY_ID_MASK;
	uint8_t prev_index = prev_key & MHL_RCP_KEY_ID_MASK;

	MHL_TX_DBG_ERR("received 0x%02x: %s(%d) in state: %s(%d)\n",
		       current_key, event_strings[event], event,
		       state_strings[current_rcp_state], current_rcp_state);
	
	switch (current_rcp_state) {
	case PH0_IDLE:
		switch (event) {
		case RCP_NORMAL_KEY_PRESS:
		case RCP_NORMAL_KEY_PRESS_SAME:
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   1);
			
			break;
		case RCP_NORMAL_KEY_RELEASE:
		case RCP_NORMAL_KEY_RELEASE_SAME:
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   0);
			
			break;
		case RCP_HOLD_KEY_PRESS:
		case RCP_HOLD_KEY_PRESS_SAME:
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_press_mode,
					   T_PRESS_MODE);
			current_rcp_state = PH3_PRESS_AND_HOLD_KEY;
			break;

		case RCP_HOLD_KEY_RELEASE:
		case RCP_HOLD_KEY_RELEASE_SAME:
			MHL_TX_DBG_ERR("unexpected %s(%d) in state: %s(%d)\n",
				       event_strings[event], event,
				       state_strings[current_rcp_state],
				       current_rcp_state);
			break;
		default:
			MHL_TX_DBG_ERR("unexpected event: %d in state: %d\n",
				       event, current_rcp_state);
			
			status = -EINVAL;
		}
		break;
	case PH3_PRESS_AND_HOLD_KEY:
		switch (event) {
		case RCP_NORMAL_KEY_PRESS:
		case RCP_NORMAL_KEY_PRESS_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_press_mode);
			rcp_trigger_key_action(dev_context, prev_index, 0);
			
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   1);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_NORMAL_KEY_RELEASE:
		case RCP_NORMAL_KEY_RELEASE_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_press_mode);
			rcp_trigger_key_action(dev_context, prev_index, 0);
			rcp_trigger_key_action(dev_context, current_index, 1);
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   0);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_HOLD_KEY_PRESS:
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_press_mode,
					   T_PRESS_MODE);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 1);
			
			break;
		case RCP_HOLD_KEY_PRESS_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_press_mode);
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_hold_maintain,
					   T_HOLD_MAINTAIN);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 1);
			current_rcp_state = ph8_hold_mode;
			break;
		case RCP_HOLD_KEY_RELEASE:
		case RCP_HOLD_KEY_RELEASE_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_press_mode);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_T_PRESS_MODE_EXPIRED:
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_hold_maintain,
					   T_HOLD_MAINTAIN);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 0);
			current_rcp_state = ph8_hold_mode;
			break;
		default:
			MHL_TX_DBG_ERR("unexpected event: %d in state: %d\n",
				       event, current_rcp_state);
			
			status = -EINVAL;
		}
		break;
	case ph8_hold_mode:
		switch (event) {
		case RCP_NORMAL_KEY_PRESS:
		case RCP_NORMAL_KEY_PRESS_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context, prev_index, 0);
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   1);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_NORMAL_KEY_RELEASE:
		case RCP_NORMAL_KEY_RELEASE_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context, prev_index, 0);
			rcp_trigger_key_action(dev_context, current_index, 1);
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   0);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_HOLD_KEY_PRESS:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_hold_maintain);
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_press_mode,
					   T_PRESS_MODE);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 1);
			current_rcp_state = PH3_PRESS_AND_HOLD_KEY;
			break;
		case RCP_HOLD_KEY_PRESS_SAME:
			mhl_tx_start_timer(dev_context,
					   dev_context->timer_T_hold_maintain,
					   T_HOLD_MAINTAIN);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 1);
			
			break;
		case RCP_HOLD_KEY_RELEASE:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context, prev_index, 0);
			rcp_trigger_key_action(dev_context, current_index, 1);
			status =
			    rcp_trigger_key_action(dev_context, current_index,
						   0);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_HOLD_KEY_RELEASE_SAME:
			mhl_tx_stop_timer(dev_context,
					  dev_context->timer_T_hold_maintain);
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;
		case RCP_T_HOLD_MAINTAIN_EXPIRED:
			status =
			    rcp_trigger_key_action(dev_context, prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;
		default:
			MHL_TX_DBG_ERR("unexpected event: %d in state: %d\n",
				       event, current_rcp_state);
			
			status = -EINVAL;
		}
		break;
	default:
		MHL_TX_DBG_ERR("irrational state value:%d\n",
			       current_rcp_state);
	}
	return status;
}

static void timer_callback_T_hold_maintain_handler(void *param)
{
	struct mhl_dev_context *dev_context = (struct mhl_dev_context *)param;
	handle_rcp_event(dev_context, rcp_current_key, rcp_previous_key,
			RCP_T_HOLD_MAINTAIN_EXPIRED);
}

static void timer_callback_T_press_mode_handler(void *param)
{
	struct mhl_dev_context *dev_context = (struct mhl_dev_context *)param;
	handle_rcp_event(dev_context, rcp_current_key, rcp_previous_key,
			RCP_T_PRESS_MODE_EXPIRED);
}

int generate_rcp_input_event(struct mhl_dev_context *dev_context,
			     uint8_t rcp_keycode)
{
	int status = -EINVAL;
	int index = rcp_keycode & MHL_RCP_KEY_ID_MASK;

	if (rcp_def_keymap[index] != KEY_UNKNOWN &&
	    rcp_def_keymap[index] != KEY_RESERVED) {

		enum rcp_event_e event;
		int mhl_key_press =
		    (rcp_keycode & MHL_RCP_KEY_RELEASED_MASK) ? 0 : 1;

		if (mhl_key_press) {
			if (rcpSupportTable[index].press_and_hold_key) {
				if (index == rcp_previous_key)
					event = RCP_HOLD_KEY_PRESS_SAME;
				else
					event = RCP_HOLD_KEY_PRESS;
			} else {
				if (index == rcp_previous_key)
					event = RCP_NORMAL_KEY_PRESS_SAME;
				else
					event = RCP_NORMAL_KEY_PRESS;
			}
		} else {
			if (rcpSupportTable[index].press_and_hold_key) {
				if (index == rcp_previous_key)
					event = RCP_HOLD_KEY_RELEASE_SAME;
				else
					event = RCP_HOLD_KEY_RELEASE;
			} else {
				if (index == rcp_previous_key)
					event = RCP_NORMAL_KEY_RELEASE_SAME;
				else
					event = RCP_NORMAL_KEY_RELEASE;
			}
		}
		status =
		    handle_rcp_event(dev_context, rcp_keycode, rcp_current_key,
				     event);
	}

	rcp_previous_key = rcp_current_key;
	rcp_current_key = rcp_keycode;

	return status;
}
#endif

uint8_t init_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	int i;
	uint8_t error;
	struct input_dev *rcp_input_dev;
	int ret;

	if (dev_context->rcp_input_dev != NULL) {
		MHL_TX_DBG_INFO("RCP input device already exists!\n");
		return 0;
	}

	rcp_input_dev = input_allocate_device();
	if (!rcp_input_dev) {
		MHL_TX_DBG_ERR("Failed to allocate RCP input device\n");
		return -ENOMEM;
	}

	set_bit(EV_KEY, rcp_input_dev->evbit);

	
	rcp_input_dev->name = "MHL Remote Control";
	rcp_input_dev->keycode = rcp_def_keymap;
	rcp_input_dev->keycodesize = sizeof(u16);
	rcp_input_dev->keycodemax = ARRAY_SIZE(rcp_def_keymap);

	for (i = 1; i < ARRAY_SIZE(rcp_def_keymap); i++) {
		u16 keycode = rcp_def_keymap[i];
		if (keycode != KEY_UNKNOWN && keycode != KEY_RESERVED)
			set_bit(keycode, rcp_input_dev->keybit);
	}

	rcp_input_dev->id.bustype = BUS_VIRTUAL;

	error = input_register_device(rcp_input_dev);
	if (error) {
		MHL_TX_DBG_ERR("Failed to register device\n");
		input_free_device(rcp_input_dev);
		return error;
	}
	ret =
	    mhl_tx_create_timer(dev_context,
				timer_callback_T_press_mode_handler,
				dev_context, &dev_context->timer_T_press_mode);
	if (ret != 0) {
		MHL_TX_DBG_ERR("failed in created timer_T_press_mode!\n");
	} else {
		ret =
		    mhl_tx_create_timer(dev_context,
					timer_callback_T_hold_maintain_handler,
					dev_context,
					&dev_context->timer_T_hold_maintain);
		if (ret != 0) {
			MHL_TX_DBG_ERR
			    ("failed to create timer_T_hold_maintain!\n");
		} else {

			MHL_TX_DBG_INFO("device created\n");

			dev_context->rcp_input_dev = rcp_input_dev;

			return 0;
		}
		mhl_tx_delete_timer(dev_context,
				    &dev_context->timer_T_press_mode);
	}
	return -1;
}

void destroy_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	if (dev_context->timer_T_press_mode) {
		mhl_tx_delete_timer(dev_context,
				    &dev_context->timer_T_press_mode);
	}
	if (dev_context->timer_T_hold_maintain) {
		mhl_tx_delete_timer(dev_context,
				    &dev_context->timer_T_hold_maintain);
	}
	if (dev_context->rcp_input_dev) {
		input_unregister_device(dev_context->rcp_input_dev);
		dev_context->rcp_input_dev = NULL;
	}
}

void rcp_input_dev_one_time_init(struct mhl_dev_context *dev_context)
{
	int i;
	for (i = 0; i < MHL_NUM_RCP_KEY_CODES; ++i)
		rcp_def_keymap[i] = rcpSupportTable[i].map[0];
}
