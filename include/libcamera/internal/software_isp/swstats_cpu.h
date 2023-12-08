/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats_cpu.h - CPU based software statistics implementation
 */

#pragma once

#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/swisp_stats.h"
#include "libcamera/internal/software_isp/swstats.h"

namespace libcamera {

/**
 * \class SwStatsCpu
 * \brief Implementation for the Software statistics on the CPU.
 */
class SwStatsCpu : public SwStats
{
public:
	SwStatsCpu();
	~SwStatsCpu() { }

	bool isValid() const { return sharedStats_.fd().isValid(); }
	const SharedFD &getStatsFD() { return sharedStats_.fd(); }
	int configure(const StreamConfiguration &inputCfg);
private:
	void statsBGGR10PLine0(const uint8_t *src, unsigned int stride);
	void statsGBRG10PLine0(const uint8_t *src, unsigned int stride);
	void resetStats(void);
	void finishStats(void);

	SharedMemObject<SwIspStats> sharedStats_;
	SwIspStats stats_;
	bool swap_lines_;
};

} /* namespace libcamera */
