/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS utility functions
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_util.h"

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

bool camssV4L2DeviceFormatMatchesStreamConfig(const V4L2DeviceFormat &fmt,
					      const StreamConfiguration &cfg,
					      const char *msgPrefix)
{
	if (cfg.pixelFormat != fmt.fourcc.toPixelFormat(false) || cfg.size != fmt.size ||
	    cfg.stride != fmt.planes[0].bpl) {
		LOG(Camss, Error)
			<< msgPrefix
			<< " StreamConfiguration vs V4L2DeviceFormat mismatch"
			<< " pixelFormat " << cfg.pixelFormat << ", " << fmt.fourcc.toPixelFormat(false)
			<< " size " << cfg.size << ", " << fmt.size
			<< " stride " << cfg.stride << ", " << fmt.planes[0].bpl
			<< " frameSize " << cfg.frameSize << ", " << fmt.planes[0].size;
		return false;
	}

	return true;
}

} /* namespace libcamera */
