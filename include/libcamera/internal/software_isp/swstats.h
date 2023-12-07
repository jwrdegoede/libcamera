/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats.h - software statistics base class
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

namespace libcamera {

class PixelFormat;
struct SharedFD;
struct StreamConfiguration;

LOG_DECLARE_CATEGORY(SwStats)

class SwStats
{
public:
	virtual ~SwStats() = 0;

	virtual bool isValid() const = 0;

	/*
	 * Setup the SwStats object for the passed in input format.
	 * Return 0 on success, a negative errno value on failure
	 * (unsupported format or failed to create the SharedMemObject).
	 */
	virtual int configure(const StreamConfiguration &inputCfg) = 0;

	/* Get the FD for the SharedMemObject for the stats. */
	virtual const SharedFD &getStatsFD() = 0;

protected:
	typedef void (SwStats::*statsProcessFn)(const uint8_t *src, unsigned int stride);
	typedef void (SwStats::*statsVoidFn)();

	/* Variables set by configure(), used every line */
	statsProcessFn stats0_;
	statsProcessFn stats2_;

	unsigned int bpp_;         /* Memory used per pixel, not precision */
	unsigned int y_skip_mask_; /* Skip lines where this bitmask is set in y */

	/* Statistics windows, set by setWindow(), used every line */
	Rectangle window_;

	/* Variables set by configure(), used once per frame */
	statsVoidFn startFrame_;
	statsVoidFn finishFrame_;
	Size patternSize_;
	unsigned int x_shift_; /* Offset of 0/1 applied to window_.x for bayer variants */

public:
	/*
	 * For some input-formats, e.g. Bayer data, processing is done multiple lines
	 * and/or columns at a time. Get width and height at which the (bayer) pattern
	 * repeats. Window values are rounded down to a multiple of this and the height
	 * also indicates if processLine2() should be called or not.
	 * This may only be called after a successful configure() call.
	 */
	const Size &patternSize() { return patternSize_; }

	/*
	 * Specify window coordinates over which to gather statistics.
	 */
	void setWindow(Rectangle window)
	{
		window_ = window;

		window_.x &= ~(patternSize_.width - 1);
		window_.x += x_shift_;
		window_.y &= ~(patternSize_.height - 1);

		/* width_ - x_shift_ to make sure the window fits */
		window_.width -= x_shift_;
		window_.width &= ~(patternSize_.width - 1);
		window_.height &= ~(patternSize_.height - 1);
	}

	/*
	 * Reset state to start statistic gathering for a new frame.
	 * This may only be called after a successful setWindow() call.
	 */
	void startFrame()
	{
		(this->*startFrame_)();
	}

	/*
	 * Process line 0 of input formats with patternSize height == 1.
	 * Process line 0 + 1 of input formats with patternSize height >= 2.
	 * This may only be called after a successful setWindow() call.
	 */
	void processLine0(unsigned int y, const uint8_t *src, unsigned int stride)
	{
		if ((y & y_skip_mask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats0_)(src + window_.x * bpp_ / 8, stride);
	}

	/*
	 * Process lines 2 + 3 of input formats with patternSize height == 4.
	 * This may only be called after a successful setWindow() call.
	 */
	void processLine2(unsigned int y, const uint8_t *src, unsigned int stride)
	{
		if ((y & y_skip_mask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats2_)(src + window_.x * bpp_ / 8, stride);
	}

	/*
	 * Finish statistics calculation for current frame and
	 * store the results in the SharedMemObject.
	 * This may only be called after a successful setWindow() call.
	 */
	void finishFrame()
	{
		(this->*finishFrame_)();
	}

	/*
	 * Process a whole frame at once, replacing manually calling
	 * startFrame() + processLine0() + finishFrame().
	 * This may only be called after a successful setWindow() call.
	 */
	void processFrame(const uint8_t *src, unsigned int stride);

	/* The int parameter isn't actually used */
	Signal<int> statsReady;
};

} /* namespace libcamera */
