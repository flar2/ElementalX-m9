/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#ifndef __MSM_FD_DEV_H__
#define __MSM_FD_DEV_H__

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ctrls.h>
#include <media/msm_fd.h>

#define MSM_FD_MAX_RESULT_BUFS 5
#define MSM_FD_MAX_CLK_NUM 10
#define MSM_FD_MAX_CLK_RATES 5
#define MSM_FD_MAX_FACES_DETECTED 32

struct msm_fd_size {
	int width;
	int height;
	u32 reg_val;
	int work_size;
};

struct msm_fd_setings {
	unsigned int min_size_index;
	unsigned int angle_index;
	unsigned int direction_index;
	unsigned int threshold;
	unsigned int speed;
};

struct msm_fd_format {
	struct msm_fd_size *size;
	struct v4l2_rect crop;
	int bytesperline;
	int sizeimage;
	u32 pixelformat;
};

struct msm_fd_mem_pool {
	struct msm_fd_device *fd_device;
	struct ion_client *client;
	int domain_num;
};

struct msm_fd_buf_handle {
	int fd;
	struct msm_fd_mem_pool *pool;
	void *handle;
	unsigned long size;
	ion_phys_addr_t addr;
};

struct msm_fd_buffer {
	struct vb2_buffer vb;
	atomic_t active;
	struct completion completion;
	struct msm_fd_format format;
	struct msm_fd_setings settings;
	ion_phys_addr_t work_addr;
	struct list_head list;
};

struct msm_fd_stats {
	atomic_t frame_id;
	u32 face_cnt;
	struct msm_fd_face_data face_data[MSM_FD_MAX_FACES_DETECTED];
};

struct fd_ctx {
	struct msm_fd_device *fd_device;
	struct v4l2_fh fh;
	struct vb2_queue vb2_q;
	unsigned int sequence;
	atomic_t subscribed_for_event;
	struct msm_fd_format format;
	struct msm_fd_setings settings;
	struct msm_fd_mem_pool mem_pool;
	struct msm_fd_stats *stats;
	struct msm_fd_buf_handle work_buf;
	struct completion *wait_stop_stream;
};

enum msm_fd_device_state {
	MSM_FD_DEVICE_IDLE,
	MSM_FD_DEVICE_RUNNING,
};

enum msm_fd_mem_resources {
	MSM_FD_IOMEM_CORE,
	MSM_FD_IOMEM_MISC,
	MSM_FD_IOMEM_VBIF,
	MSM_FD_IOMEM_LAST
};

struct msm_fd_device {
	struct mutex lock;
	spinlock_t slock;
	int ref_count;

	int irq_num;
	struct resource *res_mem[MSM_FD_IOMEM_LAST];
	void __iomem *iomem_base[MSM_FD_IOMEM_LAST];
	struct resource *ioarea[MSM_FD_IOMEM_LAST];
	struct regulator *vdd;

	unsigned int clk_num;
	struct clk *clk[MSM_FD_MAX_CLK_NUM];
	unsigned int clk_rates_num;
	unsigned int clk_rates[MSM_FD_MAX_CLK_RATES][MSM_FD_MAX_CLK_NUM];

	uint32_t bus_client;

	struct iommu_domain *iommu_domain;
	int iommu_domain_num;
	unsigned int iommu_attached_cnt;

	struct device *iommu_dev;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device video;

	enum msm_fd_device_state state;
	struct list_head buf_queue;
	struct workqueue_struct *work_queue;
	struct work_struct work;
};

#endif 
