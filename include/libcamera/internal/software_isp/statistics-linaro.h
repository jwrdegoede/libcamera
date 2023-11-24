/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * statistics.h - Statistics data format used by the software ISP
 */

#pragma once

namespace libcamera {

struct SwIspStats {
	float bright_ratio;
	float too_bright_ratio;
};

}
