/*
 * Broadcom Event  protocol definitions
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
 * Dependencies: proto/bcmeth.h
 *
 * $Id: dnglevent.h 542421 2015-03-19 18:14:47Z $
 *
 */


#ifndef _DNGLEVENT_H_
#define _DNGLEVENT_H_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif
#include <proto/bcmeth.h>

#include <packed_section_start.h>
#define BCMILCP_BCM_SUBTYPE_DNGLEVENT		5
#define BCM_DNGL_EVENT_MSG_VERSION		1
#define DNGL_E_SOCRAM_IND			0x2
typedef BWL_PRE_PACKED_STRUCT struct
{
	uint16  version; 
	uint16  reserved; 
	uint16  event_type; 
	uint16  datalen; 
} BWL_POST_PACKED_STRUCT bcm_dngl_event_msg_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_event {
	struct ether_header eth;
	bcmeth_hdr_t        bcm_hdr;
	bcm_dngl_event_msg_t      dngl_event;
	
} BWL_POST_PACKED_STRUCT bcm_dngl_event_t;


#define  SOCRAM_IND_ASSRT_TAG		0x1
#define SOCRAM_IND_TAG_HEALTH_CHECK	0x2
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_socramind {
	uint16			tag;	
	uint16			length; 
	uint8			value[1]; 
} BWL_POST_PACKED_STRUCT bcm_dngl_socramind_t;

#define	HEALTH_CHECK_TOP_LEVEL_MODULE_PCIEDEV_RTE 1
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_healthcheck {
	uint16			top_module_tag;	
	uint16			top_module_len; 
	uint8			value[1]; 
} BWL_POST_PACKED_STRUCT bcm_dngl_healthcheck_t;

#define HEALTH_CHECK_PCIEDEV_VERSION	1
#define HEALTH_CHECK_PCIEDEV_FLAG_IN_D3_SHIFT	0
#define HEALTH_CHECK_PCIEDEV_FLAG_IN_D3_FLAG	1 << HEALTH_CHECK_PCIEDEV_FLAG_IN_D3_SHIFT
#define HEALTH_CHECK_PCIEDEV_INDUCED_IND	0x1
#define HEALTH_CHECK_PCIEDEV_H2D_DMA_IND	0x2
#define HEALTH_CHECK_PCIEDEV_D2H_DMA_IND	0x3
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_pcie_hc {
	uint16			version; 
	uint16			reserved;
	uint16			pcie_err_ind_type; 
	uint16			pcie_flag;
	uint32			pcie_control_reg;
} BWL_POST_PACKED_STRUCT bcm_dngl_pcie_hc_t;

#include <packed_section_end.h>

#endif 
