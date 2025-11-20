/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * egl.cpp - Helper class for managing GBM interactions.
 */

#include "libcamera/internal/gbm.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(GBM)

GBM::GBM()
{
	fd_ = 0;
	gbm_device_ = NULL;
}

GBM::~GBM()
{
	if (gbm_device_)
		gbm_device_destroy(gbm_device_);
}

int GBM::createDevice()
{
	const char *dri_node = "/dev/dri/renderD128"; //TODO: get from an env or config setting

	fd_ = open(dri_node, O_RDWR | O_CLOEXEC);
	if (fd_ < 0) {
		LOG(GBM, Error) << "Open " << dri_node << " fail " << fd_;
		return fd_;
	}

	gbm_device_ = gbm_create_device(fd_);
	if (!gbm_device_) {
		LOG(GBM, Error) << "gbm_crate_device fail";
		goto fail;
	}

	format_ = libcamera::formats::ARGB8888;

	return 0;
fail:
	close(fd_);
	return -errno;
}
} //namespace libcamera
