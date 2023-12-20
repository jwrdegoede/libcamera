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

#include <stdint.h>

#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

class PixelFormat;
struct StreamConfiguration;

class SwStatsCpu
{
public:
	SwStatsCpu();
	~SwStatsCpu() = default;

	/**
	 * \brief Gets whether the statistics object is valid.
	 *
	 * \return true if it's valid, false otherwise
	 */
	bool isValid() const { return sharedStats_.fd().isValid(); }

	/**
	 * \brief Get the file descriptor for the statistics.
	 *
	 * \return the file descriptor
	 */
	const SharedFD &getStatsFD() { return sharedStats_.fd(); }

	/**
	 * \brief Get the pattern size.
	 *
	 * For some input-formats, e.g. Bayer data, processing is done multiple lines
	 * and/or columns at a time. Get width and height at which the (bayer) pattern
	 * repeats. Window values are rounded down to a multiple of this and the height
	 * also indicates if processLine2() should be called or not.
	 * This may only be called after a successful configure() call.
	 *
	 * \return the pattern size
	 */
	const Size &patternSize() { return patternSize_; }

	int configure(const StreamConfiguration &inputCfg);
	void setWindow(Rectangle window);
	void startFrame();
	void finishFrame();

	/**
	 * \brief Process line 0.
	 * \param[in] y The y coordinate.
	 * \param[in] src The input data.
	 *
	 * This function processes line 0 for input formats with patternSize height == 1.
	 * It'll process line 0 and 1 for input formats with patternSize height >= 2.
	 * This function may only be called after a successful setWindow() call.
	 */
	void processLine0(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats0_)(src);
	}

	/**
	 * \brief Process line 2 and 3.
	 * \param[in] y The y coordinate.
	 * \param[in] src The input data.
	 *
	 * This function processes line 2 and 3 for input formats with patternSize height == 4.
	 * This function may only be called after a successful setWindow() call.
	 */
	void processLine2(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats2_)(src);
	}

	/**
	 * \brief Signals that the statistics are ready.
	 *
	 * The int parameter isn't actually used.
	 */
	Signal<int> statsReady;

private:
	/**
	 * \brief Called when there is data to get statistics from.
	 * \param[in] src The input data
	 *
	 * These functions take an array of (patternSize_.height + 1) src
	 * pointers each pointing to a line in the source image. The middle
	 * element of the array will point to the actual line being processed.
	 * Earlier element(s) will point to the previous line(s) and later
	 * element(s) to the next line(s).
	 *
	 * See the documentation of DebayerCpu::debayerFn for more details.
	 */
	using statsProcessFn = void (SwStatsCpu::*)(const uint8_t *src[]);

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
	/* IGIG_GBGR_IGIG_GRGB 10 bpp unpacked */
	void statsRGBIR10Line0(const uint8_t *src[]);
	void statsRGBIR10Line2(const uint8_t *src[]);

	/* Variables set by configure(), used every line */
	statsProcessFn stats0_;
	statsProcessFn stats2_;
	bool swapLines_;

	/**
	 * \brief Skip lines where this bitmask is set in y.
	 */
	unsigned int ySkipMask_;

	/**
	 * \brief Statistics window, set by setWindow(), used ever line.
	 */
	Rectangle window_;

	/**
	 * \brief The size of the bayer pattern.
	 *
	 * Valid sizes are: 2x2, 4x2 or 4x4.
	 */
	Size patternSize_;

	/**
	 * \brief The offset of x, applied to window_.x for bayer variants.
	 *
	 * This can either be 0 or 1.
	 */
	unsigned int xShift_;

	SharedMemObject<SwIspStats> sharedStats_;
	SwIspStats stats_;
};

} /* namespace libcamera */
