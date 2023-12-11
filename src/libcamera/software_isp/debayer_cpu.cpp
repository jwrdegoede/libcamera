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
	/* Gamma curve is stored in green lookup, initialize to 1.0 curve */
	for (int i = 0; i < 256; i++)
		green_[i] = i;
}

struct ctxt_8bit_src {
	/* Pointers to previous, current and next lines */
	const uint8_t *prev;
	const uint8_t *curr;
	const uint8_t *next;

	/* Pointers to gamma correction tables */
	const uint8_t *red;
	const uint8_t *green;
	const uint8_t *blue;
};

static inline void bggr8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[c.curr[x]];
	*dst++ = c.green[(c.prev[x] + c.curr[x - p] + c.curr[x + n] + c.next[x]) / 4];
	*dst++ = c.red[(c.prev[x - p] + c.prev[x + n] + c.next[x - p]  + c.next[x + n]) / 4];
}

static inline void grbg8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.prev[x] + c.next[x]) / 2];
	*dst++ = c.green[c.curr[x]];
	*dst++ = c.red[(c.curr[x - p] + c.curr[x + n]) / 2];
}

static inline void gbrg8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.curr[x - p] + c.curr[x + n]) / 2];
	*dst++ = c.green[c.curr[x]];
	*dst++ = c.red[(c.prev[x] + c.next[x]) / 2];
}

static inline void rggb8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.prev[x - p] + c.prev[x + n] + c.next[x - p]  + c.next[x + n]) / 4];
	*dst++ = c.green[(c.prev[x] + c.curr[x - p] + c.curr[x + n] + c.next[x]) / 4];
	*dst++ = c.red[c.curr[x]];
}

void DebayerCpu::debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 * x++ in the for-loop skips the 5th byte with 4 x 2 lsb-s for 10bit packed.
	 */
	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		bggr8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel BGGR -> GBRG */
		gbrg8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		bggr8_bgr888(c, dst, x++, 1, 1);
		gbrg8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		grbg8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel GRBG -> RGGB */
		rggb8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		grbg8_bgr888(c, dst, x++, 1, 1);
		rggb8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		gbrg8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel GBGR -> BGGR */
		bggr8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		gbrg8_bgr888(c, dst, x++, 1, 1);
		bggr8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		rggb8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel RGGB -> GRBG*/
		grbg8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		rggb8_bgr888(c, dst, x++, 1, 1);
		grbg8_bgr888(c, dst, x++, 1, 2);
	}
}

int DebayerCpu::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
	    	config.bpp = 10;
		config.patternSize.height = 2;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.x_shift = 0;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888 });

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GBRG:
		case BayerFormat::GRBG:
		case BayerFormat::RGGB:
			return 0;
		default:
			break;
		}
	/* } else if (future supported fmts) { ... */
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

/* TODO: this ignores outputFormat since there is only 1 supported outputFormat for now */
int DebayerCpu::setDebayerFunctions(PixelFormat inputFormat, [[maybe_unused]] PixelFormat outputFormat)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

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

	stats_->setWindow(window_);

	/* This *must* be done after the stats_->setWindow() call! */
	window_.x += inputConfig_.x_shift;

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
	unsigned int y_end = window_.y + window_.height;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 2) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process4(const uint8_t *src, uint8_t *dst)
{
	unsigned int y_end = window_.y + window_.height;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 4) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		

		stats_->processLine2(y, src, inputConfig_.stride);
		(this->*debayer2_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer3_)(dst, src);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process(FrameBuffer *input, FrameBuffer *output, DebayerParams params)
{
	/*
	 * HACK use stats from previous frame to calculate red and blue gains
	 * for simple AWB.
	 * This overrides the passed in gains since nothing is setting these yet.
	 * FIXME this needs to be moved to the IPA providing the params and
	 * our caller then passing these in.
	 */
	struct SwIspStats stats = stats_->getStats();

	/* Clamp max gain at 4.0, this also avoids 0 division */
	if (stats.sumR_ <= stats.sumG_ / 4)
		params.gainR = 1024;
	else
		params.gainR  = 256 * stats.sumG_ / stats.sumR_;

	if (stats.sumB_ <= stats.sumG_ / 4)
		params.gainB = 1024;
	else
		params.gainB = 256 * stats.sumG_ / stats.sumB_;
	/**** End of HACK / FIXME ****/

	/* Apply DebayerParams */
	if (params.gamma != gamma_correction_) {
		for (int i = 0; i < 256; i++)
			green_[i] = 255 * powf(i / 255.0, params.gamma);

		gamma_correction_ = params.gamma;
	}

	for (int i = 0; i < 256; i++) {
		int idx;

		/* Use gamma curve stored in green lookup, apply gamma after gain! */
		idx = std::min({ i * params.gainR / 256U, 255U });
		red_[i] = green_[idx];

		idx = std::min({ i * params.gainB / 256U, 255U });
		blue_[i] = green_[idx];
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
