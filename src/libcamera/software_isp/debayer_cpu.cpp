/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * debayer_cpu.cpp - CPU based debayering class
 */

#include "libcamera/internal/software_isp/debayer_cpu.h"

#include <math.h>

#include <libcamera/formats.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

DebayerCpu::DebayerCpu(std::unique_ptr<SwStatsCpu> stats)
	: stats_(std::move(stats)), gamma_correction_(1.0)
{
	/* Initialize gamma to 1.0 curve */
	for (int i = 0; i < 1024; i++)
		gamma_[i] = i / 4;
}

#define DECLARE_SRC_POINTERS(pixel_t) \
	const pixel_t *prev = (const pixel_t *)(src - inputConfig_.stride); \
	const pixel_t *curr = (const pixel_t *)src; \
	const pixel_t *next = (const pixel_t *)(src + inputConfig_.stride);

// RGR
// GBG
// RGR
#define BGGR_BGR888(p, n, div) \
	*dst++ = blue_[curr[x] / (div)]; \
	*dst++ = green_[(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div))]; \
	*dst++ = red_[(prev[x - p] + prev[x + n] + next[x - p]  + next[x + n]) / (4 * (div))]; \
	x++;

// GBG
// RGR
// GBG
#define GRBG_BGR888(p, n, div) \
	*dst++ = blue_[(prev[x] + next[x]) / (2 * (div))]; \
	*dst++ = green_[curr[x] / (div)]; \
	*dst++ = red_[(curr[x - p] + curr[x + n]) / (2 * (div))]; \
	x++;

// GRG
// BGB
// GRG
#define GBRG_BGR888(p, n, div) \
	*dst++ = blue_[(curr[x - p] + curr[x + n]) / (2 * (div))]; \
	*dst++ = green_[curr[x] / (div)]; \
	*dst++ = red_[(prev[x] + next[x]) / (2 * (div))]; \
	x++;

// BGB
// GRG
// BGB
#define RGGB_BGR888(p, n, div) \
	*dst++ = blue_[(prev[x - p] + prev[x + n] + next[x - p]  + next[x + n]) / (4 * (div))]; \
	*dst++ = green_[(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div))]; \
	*dst++ = red_[curr[x] / (div)]; \
	x++;

void DebayerCpu::debayer8_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 1, 1)
	}
}

void DebayerCpu::debayer8_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 1, 1)
	}
}

void DebayerCpu::debayer10_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		BGGR_BGR888(1, 1, 4)
		GBRG_BGR888(1, 1, 4)
	}
}

void DebayerCpu::debayer10_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		GRBG_BGR888(1, 1, 4)
		RGGB_BGR888(1, 1, 4)
	}
}

void DebayerCpu::debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	DECLARE_SRC_POINTERS(uint8_t)

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 * x++ in the for-loop skips the 5th byte with 4 x 2 lsb-s for 10bit packed.
	 */
	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		BGGR_BGR888(2, 1, 1)
		/* Odd pixel BGGR -> GBRG */
		GBRG_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 2, 1)
	}
}

void DebayerCpu::debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		GRBG_BGR888(2, 1, 1)
		/* Odd pixel GRBG -> RGGB */
		RGGB_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 2, 1)
	}
}

void DebayerCpu::debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		GBRG_BGR888(2, 1, 1)
		/* Odd pixel GBGR -> BGGR */
		BGGR_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		GBRG_BGR888(1, 1, 1)
		BGGR_BGR888(1, 2, 1)
	}
}

void DebayerCpu::debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		RGGB_BGR888(2, 1, 1)
		/* Odd pixel RGGB -> GRBG*/
		GRBG_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		RGGB_BGR888(1, 1, 1)
		GRBG_BGR888(1, 2, 1)
	}
}

static bool isStandardBayerOrder(BayerFormat::Order order)
{
	return order == BayerFormat::BGGR || order == BayerFormat::GBRG ||
	       order == BayerFormat::GRBG || order == BayerFormat::RGGB;
}

int DebayerCpu::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888 });
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
	    	config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888 });
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported input format " << inputFormat.toString();
	return -EINVAL;
}

int DebayerCpu::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::RGB888) {
		config.bpp = 24;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported output format " << outputFormat.toString();
	return -EINVAL;
}

/*
 * Check for standard Bayer orders and set x_shift_ and swap debayer0/1, so that
 * a single pair of BGGR debayer functions can be used for all 4 standard orders.
 */
int DebayerCpu::setupStandardBayerOrder(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::BGGR:
		break;
	case BayerFormat::GBRG:
		x_shift_ = 1; /* BGGR -> GBRG */
		break;
	case BayerFormat::GRBG:
		std::swap(debayer0_, debayer1_); /* BGGR -> GRBG */
		break;
	case BayerFormat::RGGB:
		x_shift_ = 1; /* BGGR -> GBRG */
		std::swap(debayer0_, debayer1_); /* GBRG -> RGGB */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* TODO: this ignores outputFormat since there is only 1 supported outputFormat for now */
int DebayerCpu::setDebayerFunctions(PixelFormat inputFormat, [[maybe_unused]] PixelFormat outputFormat)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	x_shift_ = 0;

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		switch (bayerFormat.bitDepth) {
		case 8:
			debayer0_ = &DebayerCpu::debayer8_BGBG_BGR888;
			debayer1_ = &DebayerCpu::debayer8_GRGR_BGR888;
			break;
		case 10:
			debayer0_ = &DebayerCpu::debayer10_BGBG_BGR888;
			debayer1_ = &DebayerCpu::debayer10_GRGR_BGR888;
			break;
		}
		setupStandardBayerOrder(bayerFormat.order);
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			debayer0_ = &DebayerCpu::debayer10P_BGBG_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_GRGR_BGR888;
			return 0;
		case BayerFormat::GBRG:
			debayer0_ = &DebayerCpu::debayer10P_GBGB_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_RGRG_BGR888;
			return 0;
		case BayerFormat::GRBG:
			debayer0_ = &DebayerCpu::debayer10P_GRGR_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_BGBG_BGR888;
			return 0;
		case BayerFormat::RGGB:
			debayer0_ = &DebayerCpu::debayer10P_RGRG_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_GBGB_BGR888;
			return 0;
		default:
			break;
		}
	}

	LOG(Debayer, Error) << "Unsupported input output format combination";
	return -EINVAL;
}

int DebayerCpu::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	const Size &stats_pattern_size = stats_->patternSize();
	if (inputConfig_.patternSize.width != stats_pattern_size.width ||
	    inputConfig_.patternSize.height != stats_pattern_size.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat.toString();
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	if (setDebayerFunctions(inputCfg.pixelFormat, outputCfg.pixelFormat) != 0)
		return -EINVAL;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/* Don't pass x,y since process() already adjusts src before passing it */
	stats_->setWindow(Rectangle(window_.size()));

	return 0;
}

Size DebayerCpu::patternSize(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerCpu::formats(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerCpu::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerCpu::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* round up to multiple of 8 for 64 bits alignment */
	unsigned int stride = (size.width * config.bpp / 8 + 7) & ~7;

	return std::make_tuple(stride, stride * size.height);
}

void DebayerCpu::process2(const uint8_t *src, uint8_t *dst)
{
	const unsigned int y_end = window_.y + window_.height;
	const unsigned int x_shift = x_shift_ * inputConfig_.bpp / 8;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 2) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process4(const uint8_t *src, uint8_t *dst)
{
	const unsigned int y_end = window_.y + window_.height;
	const unsigned int x_shift = x_shift_ * inputConfig_.bpp / 8;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 4) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		

		stats_->processLine2(y, src, inputConfig_.stride);
		(this->*debayer2_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer3_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process(FrameBuffer *input, FrameBuffer *output, DebayerParams params)
{
	/* Apply DebayerParams */
	if (params.gamma != gamma_correction_) {
		for (int i = 0; i < 1024; i++)
			gamma_[i] = 255 * powf(i / 1023.0, params.gamma);

		gamma_correction_ = params.gamma;
	}

	for (int i = 0; i < 256; i++) {
		int idx;

		/* Apply gamma after gain! */
		idx = std::min({ i * params.gainR / 64U, 1023U });
		red_[i] = gamma_[idx];

		idx = std::min({ i * params.gainG / 64U, 1023U });
		green_[i] = gamma_[idx];

		idx = std::min({ i * params.gainB / 64U, 1023U });
		blue_[i] = gamma_[idx];
	}

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		return;
	}

	if (inputConfig_.patternSize.height == 2)
		process2(in.planes()[0].data(), out.planes()[0].data());
	else
		process4(in.planes()[0].data(), out.planes()[0].data());

	metadata.planes()[0].bytesused = out.planes()[0].size();

	outputBufferReady.emit(output);
	inputBufferReady.emit(input);
}

} /* namespace libcamera */
