/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS utility functions
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

namespace libcamera {

struct StreamConfiguration;
class V4L2DeviceFormat;

bool camssV4L2DeviceFormatMatchesStreamConfig(const V4L2DeviceFormat &fmt,
					      const StreamConfiguration &cfg,
					      const char *msgPrefix);

} /* namespace libcamera */
