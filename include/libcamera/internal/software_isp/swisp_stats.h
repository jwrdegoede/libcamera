/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_stats.h - Statistics data format used by the software ISP and software IPA
 */

#pragma once

namespace libcamera {

struct SwIspStats {
	unsigned long sumR_;
	unsigned long sumB_;
	unsigned long sumG_;

	float bright_ratio;
	float too_bright_ratio;
};

} /* namespace libcamera */
