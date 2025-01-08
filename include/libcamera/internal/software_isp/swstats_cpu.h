/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based software statistics implementation
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

#include "benchmark.h"

namespace libcamera {

class PixelFormat;
class MappedFrameBuffer;
struct StreamConfiguration;

class SwStatsCpu
{
public:
	SwStatsCpu();
	~SwStatsCpu() = default;

	bool isValid() const { return sharedStats_.fd().isValid(); }

	const SharedFD &getStatsFD() { return sharedStats_.fd(); }

	const Size &patternSize() { return patternSize_; }

	int configure(const StreamConfiguration &inputCfg);
	void setWindow(const Rectangle &window);
	void startFrame();
	void finishFrame(uint32_t frame, uint32_t bufferId);
	void processFrame(uint32_t frame, uint32_t bufferId, FrameBuffer *input, bool wantSharpness);

	void processLine0(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < static_cast<unsigned int>(window_.y) ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats0_)(src);
	}

	void processLine2(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < static_cast<unsigned int>(window_.y) ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats2_)(src);
	}

	Signal<uint32_t, uint32_t> statsReady;

private:
	using statsProcessFn = void (SwStatsCpu::*)(const uint8_t *src[]);
	using processFrameFn = void (SwStatsCpu::*)(MappedFrameBuffer &in);
	using finishFrameFn = void (SwStatsCpu::*)();

	int setupStandardBayerOrder(BayerFormat::Order order);
	/* Bayer 8 bpp unpacked */
	void statsBGGR8Line0(const uint8_t *src[]);
	/* Bayer 10 bpp unpacked */
	void statsBGGR10Line0(const uint8_t *src[]);
	/* Bayer 12 bpp unpacked */
	void statsBGGR12Line0(const uint8_t *src[]);
	/* Bayer 10 bpp packed */
	void statsBGGR10PLine0(const uint8_t *src[]);
	void statsGBRG10PLine0(const uint8_t *src[]);
	/* YUV420 3 planes */
	void statsYUV420Line0(const uint8_t *src[]);

	void processBayerFrame2(MappedFrameBuffer &in);
	void processYUV420Frame(MappedFrameBuffer &in);
	void processYUV420FrameSharpness(MappedFrameBuffer &in);
	void calculateSharpness(uint8_t *frameY);
	void finishYUV420Frame();

	processFrameFn processFrame_;
	processFrameFn processSharpness_;
	finishFrameFn finishFrame_;

	/* Variables set by configure(), used every line */
	statsProcessFn stats0_;
	statsProcessFn stats2_;
	bool swapLines_;

	unsigned int ySkipMask_;

	Rectangle window_;
	
	
	Size patternSize_;

	Size frameSize_;

	unsigned int xShift_;
	unsigned int stride_;

	SharedMemObject<SwIspStats> sharedStats_;
	SwIspStats stats_;
	Benchmark bench_;
};

} /* namespace libcamera */
