/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats.h - software statistics base class
 */

#pragma once

namespace libcamera {

struct DebayerParams {
	/* Red Gain, 128 = 0.5, 256 = 1.0, 512 = 2.0, etc. */
	unsigned int gainR;
	/* Blue Gain, same range as Red Gain. */
	unsigned int gainB;
	/* Gamma correction, 1.0 is no correction. */
	float gamma;
};

} /* namespace libcamera */
