/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _CXD224X_H
#define _CXD224X_H

#define CONFIG_CXD224X_NFC_RST
#define CXDNFC_MAGIC 'S'
#define CXDNFC_POWER_CTL		_IO(CXDNFC_MAGIC, 0x01)
#define CXDNFC_WAKE_CTL			_IO(CXDNFC_MAGIC, 0x02)
#define CXDNFC_RST_CTL			_IO(CXDNFC_MAGIC, 0x03)

#define CXDNFC_RST_ACTIVE 1            
struct cxd224x_platform_data {
	unsigned int irq_gpio;
	uint32_t irq_gpio_flags;
	unsigned int en_gpio;	
	uint32_t en_gpio_flags;
	unsigned int rst_gpio;	
	unsigned int wake_gpio;	
	uint32_t wake_gpio_flags;
	unsigned int rfs_gpio;	
};

#endif
