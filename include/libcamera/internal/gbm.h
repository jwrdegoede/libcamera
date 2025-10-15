/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * gbm.h - Helper class for managing GBM interactions.
 */

#pragma once

#include <gbm.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(GBM)

/**
 * \brief Helper class for managing GBM interactions
 *
 */
class GBM
{
public:
	GBM();
	~GBM();

	int createDevice();
	struct gbm_device *getDevice() { return gbm_device_; }
	PixelFormat getPixelFormat() { return format_; }

private:
	int fd_;
	struct gbm_device *gbm_device_;
	PixelFormat format_;
};

} // namespace libcamera
