/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats.cpp - software statistics base class
 */

#include "libcamera/internal/software_isp/swstats.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(SwStats)

void SwStats::processFrame(const uint8_t *src, unsigned int stride)
{
	unsigned int y_end = window_.y + window_.height;

	startFrame();

	/* Adjust src to point to starting corner of the statistics window */
	src += window_.y * stride;
	src += window_.x * bpp_ / 8;

	switch (patternSize_.height) {
	case 1:
		for (unsigned int y = window_.y ; y < y_end; y++) {
			if (!(y & y_skip_mask_))
				(this->*stats0_)(src, stride);
			src += stride;
		}
		break;
	case 2:
		for (unsigned int y = window_.y ; y < y_end; y += 2) {
			if (!(y & y_skip_mask_))
				(this->*stats0_)(src, stride);
			src += 2 * stride;
		}
		break;
	case 4:
		for (unsigned int y = window_.y ; y < y_end; y += 4) {
			if (y & y_skip_mask_) {
				src += 4 * stride;
				continue;
			}

			(this->*stats0_)(src, stride);
			src += 2 * stride;
			(this->*stats2_)(src, stride);
			src += 2 * stride;
		}
		break;
	}

	finishFrame();
}

SwStats::~SwStats()
{
}

} /* namespace libcamera */
