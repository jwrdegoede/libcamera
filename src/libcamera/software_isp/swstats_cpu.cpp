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

#include "swstats_cpu.h"

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"

namespace libcamera {

/**
 * \class SwStatsCpu
 * \brief Class for gathering statistics on the CPU
 *
 * CPU based software ISP statistics implementation.
 *
 * This class offers a configure function + functions to gather statistics
 * on a line by line basis. This allows CPU based software debayering to
 * interlace debayering and statistics gathering on a line by line basis
 * while the input data is still hot in the cache.
 *
 * It is also possible to specify a window over which to gather
 * statistics instead of processing the whole frame.
 */

LOG_DEFINE_CATEGORY(SwStatsCpu)

SwStatsCpu::SwStatsCpu()
{
	sharedStats_ = SharedMemObject<SwIspStats>("softIsp_stats");
	if (!sharedStats_.fd().isValid())
		LOG(SwStatsCpu, Error)
			<< "Failed to create shared memory for statistics";
}

static const unsigned int kRedYMul = 77; /* 0.299 * 256 */
static const unsigned int kGreenYMul = 150; /* 0.587 * 256 */
static const unsigned int kBlueYMul = 29; /* 0.114 * 256 */

#define SWSTATS_START_LINE_STATS(pixel_t) \
	pixel_t r, g, g2, b;              \
	unsigned int yVal;                \
                                          \
	unsigned int sumR = 0;            \
	unsigned int sumG = 0;            \
	unsigned int sumB = 0;

#define SWSTATS_ACCUMULATE_LINE_STATS(div) \
	sumR += r;                         \
	sumG += g;                         \
	sumB += b;                         \
                                           \
	yVal = r * kRedYMul;               \
	yVal += g * kGreenYMul;            \
	yVal += b * kBlueYMul;             \
	stats_.yHistogram[yVal * SwIspStats::kYHistogramSize / (256 * 256 * (div))]++;

#define SWSTATS_FINISH_LINE_STATS() \
	stats_.sumR_ += sumR;       \
	stats_.sumG_ += sumG;       \
	stats_.sumB_ += sumB;

void SwStatsCpu::statsBGGR10PLine0(const uint8_t *src[])
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;
	const int widthInBytes = window_.width * 5 / 4;

	if (swapLines_)
		std::swap(src0, src1);

	SWSTATS_START_LINE_STATS(uint8_t)

	/* x += 5 sample every other 2x2 block */
	for (int x = 0; x < widthInBytes; x += 5) {
		/* BGGR */
		b = src0[x];
		g = src0[x + 1];
		g2 = src1[x];
		r = src1[x + 1];
		g = (g + g2) / 2;
		/* Data is already 8 bits, divide by 1 */
		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

void SwStatsCpu::statsGBRG10PLine0(const uint8_t *src[])
{
	const uint8_t *src0 = src[1] + window_.x * 5 / 4;
	const uint8_t *src1 = src[2] + window_.x * 5 / 4;
	const int widthInBytes = window_.width * 5 / 4;

	if (swapLines_)
		std::swap(src0, src1);

	SWSTATS_START_LINE_STATS(uint8_t)

	/* x += 5 sample every other 2x2 block */
	for (int x = 0; x < widthInBytes; x += 5) {
		/* GBRG */
		g = src0[x];
		b = src0[x + 1];
		r = src1[x];
		g2 = src1[x + 1];
		g = (g + g2) / 2;
		/* Data is already 8 bits, divide by 1 */
		SWSTATS_ACCUMULATE_LINE_STATS(1)
	}

	SWSTATS_FINISH_LINE_STATS()
}

/**
 * \brief Reset state to start statistics gathering for a new frame.
 *
 * This may only be called after a successful setWindow() call.
 */
void SwStatsCpu::startFrame(void)
{
	stats_.sumR_ = 0;
	stats_.sumB_ = 0;
	stats_.sumG_ = 0;
	stats_.yHistogram.fill(0);
}

/**
 * \brief Finish statistics calculation for the current frame.
 *
 * This may only be called after a successful setWindow() call.
 */
void SwStatsCpu::finishFrame(void)
{
	*sharedStats_ = stats_;
	statsReady.emit(0);
}

/**
 * \brief Configure the statistics object for the passed in input format.
 * \param[in] inputCfg The input format
 *
 * \return 0 on success, a negative errno value on failure
 */
int SwStatsCpu::configure(const StreamConfiguration &inputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		patternSize_.height = 2;
		patternSize_.width = 4; /* 5 bytes per *4* pixels */
		/* Skip every 3th and 4th line, sample every other 2x2 block */
		ySkipMask_ = 0x02;
		xShift_ = 0;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GRBG:
			stats0_ = &SwStatsCpu::statsBGGR10PLine0;
			swapLines_ = bayerFormat.order == BayerFormat::GRBG;
			return 0;
		case BayerFormat::GBRG:
		case BayerFormat::RGGB:
			stats0_ = &SwStatsCpu::statsGBRG10PLine0;
			swapLines_ = bayerFormat.order == BayerFormat::RGGB;
			return 0;
		default:
			break;
		}
	}

	LOG(SwStatsCpu, Info)
		<< "Unsupported input format " << inputCfg.pixelFormat.toString();
	return -EINVAL;
}

/**
 * \brief Specify window coordinates over which to gather statistics.
 * \param[in] window The window object.
 */
void SwStatsCpu::setWindow(Rectangle window)
{
	window_ = window;

	window_.x &= ~(patternSize_.width - 1);
	window_.x += xShift_;
	window_.y &= ~(patternSize_.height - 1);

	/* width_ - xShift_ to make sure the window fits */
	window_.width -= xShift_;
	window_.width &= ~(patternSize_.width - 1);
	window_.height &= ~(patternSize_.height - 1);
}

} /* namespace libcamera */