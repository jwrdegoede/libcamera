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
static const unsigned int BRIGHT_LVL = 200U << 8;
static const unsigned int TOO_BRIGHT_LVL = 240U << 8;

static const unsigned int RED_Y_MUL = 77; /* 0.30 * 256 */
static const unsigned int GREEN_Y_MUL = 150; /* 0.59 * 256 */
static const unsigned int BLUE_Y_MUL = 29; /* 0.11 * 256 */

#define SWISP_LINARO_START_LINE_STATS(pixel_t) \
	pixel_t r, g, g2, b;                   \
	unsigned int y_val;                    \
                                               \
	unsigned int sumR = 0;                 \
	unsigned int sumG = 0;                 \
	unsigned int sumB = 0;

#define SWISP_LINARO_ACCUMULATE_LINE_STATS(div) \
	sumR += r;                              \
	sumG += g;                              \
	sumB += b;                              \
                                                \
	y_val = r * RED_Y_MUL;                  \
	y_val += g * GREEN_Y_MUL;               \
	y_val += b * BLUE_Y_MUL;                \
	stats_.y_histogram[y_val / (256 * 16 * (div))]++;

#define SWISP_LINARO_FINISH_LINE_STATS() \
	stats_.sumR_ += sumR;            \
	stats_.sumG_ += sumG;            \
	stats_.sumB_ += sumB;

void SwStatsCpu::statsBGGR8Line0(const uint8_t *src[])
{
	const uint8_t *src0 = src[1] + window_.x;
	const uint8_t *src1 = src[2] + window_.x;

	SWISP_LINARO_START_LINE_STATS(uint8_t)

	if (swap_lines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (int x = 0; x < (int)window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		SWISP_LINARO_ACCUMULATE_LINE_STATS(1)
	}

	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR10Line0(const uint8_t *src[])
{
	const uint16_t *src0 = (const uint16_t *)src[1] + window_.x;
	const uint16_t *src1 = (const uint16_t *)src[2] + window_.x;

	SWISP_LINARO_START_LINE_STATS(uint16_t)

	if (swap_lines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (int x = 0; x < (int)window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		/* divide Y by 4 for 10 -> 8 bpp value */
		SWISP_LINARO_ACCUMULATE_LINE_STATS(4)
	}

	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR12Line0(const uint8_t *src[])
{
	const uint16_t *src0 = (const uint16_t *)src[1] + window_.x;
	const uint16_t *src1 = (const uint16_t *)src[2] + window_.x;

	SWISP_LINARO_START_LINE_STATS(uint16_t)

	if (swap_lines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (int x = 0; x < (int)window_.width; x += 4) {
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];

		g = (g + g2) / 2;

		/* divide Y by 16 for 12 -> 8 bpp value */
		SWISP_LINARO_ACCUMULATE_LINE_STATS(16)
	}

	SWISP_LINARO_FINISH_LINE_STATS()
}

static inline __attribute__((always_inline)) void
statsBayer10P(const int width, const uint8_t *src0, const uint8_t *src1, bool bggr, SwIspStats &stats_)
{
	const int width_in_bytes = width * 5 / 4;

	SWISP_LINARO_START_LINE_STATS(uint8_t)

	for (int x = 0; x < width_in_bytes; x += 5) {
		if (bggr) {
			/* BGGR */
			b = src0[x];
			g = src0[x + 1];
			g2 = src1[x];
			r = src1[x + 1];
		} else {
			/* GBRG */
			g = src0[x];
			b = src0[x + 1];
			r = src1[x];
			g2 = src1[x + 1];
		}
		g = (g + g2) / 2;

		SWISP_LINARO_ACCUMULATE_LINE_STATS(1)
	}

	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwStatsCpu::statsBGGR10PLine0(const uint8_t *src[])
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;

	if (swap_lines_)
		std::swap(src0, src1);

	statsBayer10P(window_.width, src0, src1, true, stats_);
}

void SwStatsCpu::statsGBRG10PLine0(const uint8_t *src[])
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;

	if (swap_lines_)
		std::swap(src0, src1);

	statsBayer10P(window_.width, src0, src1, false, stats_);
}

void SwStatsCpu::resetStats(void)
{
	stats_.sumR_ = 0;
	stats_.sumB_ = 0;
	stats_.sumG_ = 0;
	std::fill_n(stats_.y_histogram, 16, 0);
}

void SwStatsCpu::finishStats(void)
{
	*sharedStats_ = stats_;
	statsReady.emit(0);
}

/*
 * Check if order is a standard Bayer order and setup x_shift_ and swap_lines_
 * so that a single BGGR stats function can be used for all 4 standard orders.
 */
int SwStatsCpu::setupStandardBayerOrder(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::BGGR:
		x_shift_ = 0;
		swap_lines_ = false;
		break;
	case BayerFormat::GBRG:
		x_shift_ = 1; /* BGGR -> GBRG */
		swap_lines_ = false;
		break;
	case BayerFormat::GRBG:
		x_shift_ = 0;
		swap_lines_ = true; /* BGGR -> GRBG */
		break;
	case BayerFormat::RGGB:
		x_shift_ = 1; /* BGGR -> GBRG */
		swap_lines_ = true; /* GBRG -> RGGB */
		break;
	default:
		return -EINVAL;
	}

	patternSize_.height = 2;
	patternSize_.width = 2;
	y_skip_mask_ = 0x02; /* Skip every 3th and 4th line */
	return 0;
}

int SwStatsCpu::configure(const StreamConfiguration &inputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);

	startFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::resetStats;
	finishFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::finishStats;

	if (bayerFormat.packing == BayerFormat::Packing::None &&
	    setupStandardBayerOrder(bayerFormat.order) == 0) {
		bpp_ = (bayerFormat.bitDepth + 7) & ~7;
		switch (bayerFormat.bitDepth) {
		case 8:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR8Line0;
			return 0;
		case 10:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR10Line0;
			return 0;
		case 12:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR12Line0;
			return 0;
		}
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		bpp_ = 10;
		patternSize_.height = 2;
		patternSize_.width = 4; /* 5 bytes per *4* pixels */
		y_skip_mask_ = 0x02; /* Skip every 3th and 4th line */
		x_shift_ = 0;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GRBG:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR10PLine0;
			swap_lines_ = bayerFormat.order == BayerFormat::GRBG;
			return 0;
		case BayerFormat::GBRG:
		case BayerFormat::RGGB:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsGBRG10PLine0;
			swap_lines_ = bayerFormat.order == BayerFormat::RGGB;
			return 0;
		default:
			break;
		}
	}

	LOG(SwStats, Info)
		<< "Unsupported input format " << inputCfg.pixelFormat.toString();
	return -EINVAL;
}

} /* namespace libcamera */
