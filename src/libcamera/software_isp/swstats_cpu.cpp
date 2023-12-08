/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats_cpu.cpp - CPU based software statistics implementation
 */

#include "libcamera/internal/software_isp/swstats_cpu.h"

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"

namespace libcamera {

SwStatsCpu::SwStatsCpu()
	: SwStats()
{
	sharedStats_ = SharedMemObject<SwIspStats>("softIsp_stats");
	if (!sharedStats_.fd().isValid())
		LOG(SwStats, Error)
			<< "Failed to create shared memory for statistics";
}

/* for brightness values in the 0 to 255 range: */
static const unsigned int BRIGHT_LVL = 170U << 8;
static const unsigned int TOO_BRIGHT_LVL = 210U << 8;

static const unsigned int RED_Y_MUL = 77;		/* 0.30 * 256 */
static const unsigned int GREEN_Y_MUL = 150;		/* 0.59 * 256 */
static const unsigned int BLUE_Y_MUL = 29;		/* 0.11 * 256 */

static inline __attribute__((always_inline)) void
statsBayer10P(const int width, const uint8_t *src0, const uint8_t *src1, bool bggr, unsigned int &bright_sum_, unsigned int &too_bright_sum_, SwIspStats &stats_)
{
	const int width_in_bytes = width * 5 / 4;
	uint8_t r, g, b, g2;
	unsigned int y_val;
	unsigned int sumR = 0;
	unsigned int sumG = 0;
	unsigned int sumB = 0;

	unsigned int bright_sum = 0;
	unsigned int too_bright_sum = 0;

	for (int x = 0; x < width_in_bytes; x += 5) {
		if (bggr) {
			/* BGGR */
			b  = src0[x];
			g  = src0[x + 1];
			g2 = src1[x];
			r  = src1[x + 1];
		} else {
			/* GBRG */
			g  = src0[x];
			b  = src0[x + 1];
			r  = src1[x];
			g2 = src1[x + 1];
		}
		g = g + g2 / 2;

		sumR += r;
		sumG += g;
		sumB += b;

		y_val = r * RED_Y_MUL;
		y_val += g * GREEN_Y_MUL;
		y_val += b * BLUE_Y_MUL;
		if (y_val > BRIGHT_LVL) ++bright_sum;
		if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
	}

	stats_.sumR_ += sumR;
	stats_.sumG_ += sumG;
	stats_.sumB_ += sumB;

	bright_sum_ += bright_sum;
	too_bright_sum_ += too_bright_sum;
}

void SwStatsCpu::statsBGGR10PLine0(const uint8_t *src, unsigned int stride)
{
	statsBayer10P(window_.width, src, src + stride, true, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::statsGBRG10PLine0(const uint8_t *src, unsigned int stride)
{
	statsBayer10P(window_.width, src, src + stride, false, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::statsGRBG10PLine0(const uint8_t *src, unsigned int stride)
{
	/* GRBG is BGGR with the lines swapped */
	statsBayer10P(window_.width, src + stride, src, true, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::statsRGGB10PLine0(const uint8_t *src, unsigned int stride)
{
	/* RGGB is GBRG with the lines swapped */
	statsBayer10P(window_.width, src + stride, src, false, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::resetStats(void)
{
	stats_.sumR_ = 0;
	stats_.sumB_ = 0;
	stats_.sumG_ = 0;

	bright_sum_ = 0;
	too_bright_sum_ = 0;
}

void SwStatsCpu::finishStats(void)
{
	/* calculate the fractions of "bright" and "too bright" pixels */
	stats_.bright_ratio = (float)bright_sum_ / (window_.height * window_.width / 16);
	stats_.too_bright_ratio = (float)too_bright_sum_ / (window_.height * window_.width / 16);

	*sharedStats_ = stats_;
	statsReady.emit(0);
}

int SwStatsCpu::configure(const StreamConfiguration &inputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);

	startFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::resetStats;
	finishFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::finishStats;

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		bpp_ = 10;
		patternSize_.height = 2;
		patternSize_.width = 4; /* 5 bytes per *4* pixels */
		y_skip_mask_ = 0x0c; /* Skip every 3th and 4th line */
		x_shift_ = 0;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR10PLine0;
			return 0;
		case BayerFormat::GBRG:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsGBRG10PLine0;
			return 0;
		case BayerFormat::GRBG:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsGRBG10PLine0;
			return 0;
		case BayerFormat::RGGB:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsRGGB10PLine0;
			return 0;
		default:
			break;
		}
	/* } else if (future supported fmts) { ... */
	}

	LOG(SwStats, Info)
		<< "Unsupported input format " << inputCfg.pixelFormat.toString();
	return -EINVAL;
}

} /* namespace libcamera */
