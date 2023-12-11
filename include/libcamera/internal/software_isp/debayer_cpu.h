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

/**
 * \class DebayerCpu
 * \brief Class for debayering on the CPU
 *
 * Implementation for CPU based debayering
 */
class DebayerCpu : public Debayer, public Object
{
public:
	/*
	  * FIXME this should be a plain (implementation independent)  SwStats
	  * this can be fixed once getStats() is dropped.
	  */
	/**
	 * \brief Constructs a DebayerCpu object.
	 * \param[in] stats Pointer to the stats object to use.
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

	/**
	 * \brief Get the file descriptor for the statistics.
	 *
	 * \return the file descriptor pointing to the statistics.
	 */
	const SharedFD &getStatsFD() { return stats_->getStatsFD(); }

	/**
	 * \brief Get the output frame size.
	 *
	 * \return The output frame size.
	 */
	unsigned int frameSize() { return outputConfig_.frameSize; }
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

	uint8_t gamma_[1024];
	uint8_t red_[256];
	uint8_t green_[256];
	uint8_t blue_[256];
	debayerFn debayer0_;
	debayerFn debayer1_;
	debayerFn debayer2_;
	debayerFn debayer3_;
	Rectangle window_;
	DebayerInputConfig inputConfig_;
	DebayerOutputConfig outputConfig_;
	std::unique_ptr<SwStatsCpu> stats_;
	unsigned int x_shift_; /* Offset of 0/1 applied to window_.x */
	float gamma_correction_;
};

} /* namespace libcamera */
