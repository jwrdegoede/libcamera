/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * debayer_cpu.h - CPU based debayering header
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

#include <libcamera/base/object.h>

#include "libcamera/internal/software_isp/swstats_cpu.h"
#include "libcamera/internal/software_isp/debayer.h"

namespace libcamera {

class DebayerCpu : public Debayer, public Object
{
public:
	/*
	  * FIXME this should be a plain (implementation independent)  SwStats
	  * this can be fixed once getStats() is dropped.
	  */
	DebayerCpu(std::unique_ptr<SwStatsCpu> stats);
	~DebayerCpu() {}

	/*
	 * Setup the Debayer object according to the passed in parameters.
	 * Return 0 on success, a negative errno value on failure
	 * (unsupported parameters).
	 */
	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs);

	/*
	 * Get width and height at which the bayer-pattern repeats.
	 * Return pattern-size or an empty Size for an unsupported inputFormat.
	 */
	Size patternSize(PixelFormat inputFormat);

	std::vector<PixelFormat> formats(PixelFormat input);
	std::tuple<unsigned int, unsigned int>
		strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);

	void process(FrameBuffer *input, FrameBuffer *output, DebayerParams params);

	const SharedFD &getStatsFD() { return stats_->getStatsFD(); }

	unsigned int frameSize() { return outputConfig_.frameSize; }

	/* FIXME this should be dropped once AWB has moved to the IPA */
	SwIspStats getStats() { return stats_->getStats(); }
private:
	void process2(const uint8_t *src, uint8_t *dst);
	void process4(const uint8_t *src, uint8_t *dst);
	/* CSI-2 packed 10-bit raw bayer format (all the 4 orders) */
	void debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src);
	void debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src);
	void debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src);
	void debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src);

	typedef void (DebayerCpu::*debayerFn)(uint8_t *dst, const uint8_t *src);

	struct DebayerInputConfig {
		Size patternSize;
		unsigned int bpp; /* Memory used per pixel, not precision */
		unsigned int x_shift; /* Offset of 0/1 applied to window_.x */
		unsigned int stride;
		std::vector<PixelFormat> outputFormats;
	};

	struct DebayerOutputConfig {
		unsigned int bpp; /* Memory used per pixel, not precision */
		unsigned int stride;
		unsigned int frameSize;
	};

	int getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config);
	int getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config);
	int setDebayerFunctions(PixelFormat inputFormat, PixelFormat outputFormat);

	debayerFn debayer0_;
	debayerFn debayer1_;
	debayerFn debayer2_;
	debayerFn debayer3_;
	uint8_t red_[256];
	uint8_t green_[256];
	uint8_t blue_[256];
	Rectangle window_;
	DebayerInputConfig inputConfig_;
	DebayerOutputConfig outputConfig_;
	std::unique_ptr<SwStatsCpu> stats_;
	float gamma_correction_;
};

} /* namespace libcamera */
