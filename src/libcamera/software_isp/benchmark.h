/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * Simple builtin benchmark to measure software ISP processing times
 */

#pragma once

#include <stdint.h>
#include <time.h>

namespace libcamera {

class Benchmark
{
public:
	Benchmark();
	~Benchmark();

	void startFrame(void);
	void finishFrame(void);

private:
	unsigned int measuredFrames_;
	int64_t frameProcessTime_;
	timespec frameStartTime_;
	/* Skip 30 frames for things to stabilize then measure 30 frames */
	static constexpr unsigned int kFramesToSkip = 30;
	static constexpr unsigned int kLastFrameToMeasure = 60;
};

} /* namespace libcamera */
