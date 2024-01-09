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

/**
 * \class SwStats
 * \brief Base class for the software ISP statistics.
 *
 * Base class for the software ISP statistics.
 */
class SwStats
{
public:
	virtual ~SwStats() = 0;

	/**
	 * \brief Gets wether the statistics object is valid.
	 * 
	 * \return true if it's valid, false otherwise.
	 */
	virtual bool isValid() const = 0;

	/**
	 * \brief Configure the statistics object for the passed in input format.
	 * \param[in] inputCfg The input format
	 *
	 * \return 0 on success, a negative errno value on failure.
	 */
	virtual int configure(const StreamConfiguration &inputCfg) = 0;

	/**
	 * \brief Get the file descriptor for the statistics.
	 *
	 * \return the file descriptor
	 */
	virtual const SharedFD &getStatsFD() = 0;

protected:
	/**
	 * \brief Called when there is data to get statistics from.
	 * \param[in] src The input data
	 * \param[in] stride The stride in bytes
	 */
	typedef void (SwStats::*statsProcessFn)(const uint8_t *src, unsigned int stride);
	/**
	 * \brief Called when the statistics gathering is done or when a new frame starts.
	 */
	typedef void (SwStats::*statsVoidFn)();

	/* Variables set by configure(), used every line */
	/**
	 * \brief The function called when a line is ready for statistics processing.
	 *
	 * Used for line 0 and 1, repeating if there isn't a 3rd and a 4th line in the bayer order.
	 */
	statsProcessFn stats0_;
	/**
	 * \brief The function called when a line is ready for statistics processing.
	 *
	 * Used for line 3 and 4, only needed if the bayer order has 4 different lines.
	 */
	statsProcessFn stats2_;

	/**
	 * \brief The memory used per pixel in bits.
	 */
	unsigned int bpp_;
	/**
	 * \brief Skip lines where this bitmask is set in y.
	 */
	unsigned int y_skip_mask_;

	/**
	 * \brief Statistics window, set by setWindow(), used ever line.
	 */
	Rectangle window_;

	/**
	 * \brief The function called at the start of a frame.
	 */
	statsVoidFn startFrame_;
	/**
	 * \brief The function called at the end of a frame.
	 */
	statsVoidFn finishFrame_;
	/**
	 * \brief The size of the bayer pattern.
	 */
	Size patternSize_;
	/**
	 * \brief The offset of x, applied to window_.x for bayer variants.
	 *
	 * This can either be 0 or 1.
	 */
	unsigned int x_shift_;

public:
	/**
	 * \brief Get the pattern size.
	 *
	 * For some input-formats, e.g. Bayer data, processing is done multiple lines
	 * and/or columns at a time. Get width and height at which the (bayer) pattern
	 * repeats. Window values are rounded down to a multiple of this and the height
	 * also indicates if processLine2() should be called or not.
	 * This may only be called after a successful configure() call.
	 *
	 * \return the pattern size.
	 */
	const Size &patternSize() { return patternSize_; }

	/**
	 * \brief Specify window coordinates over which to gather statistics.
	 * \param[in] window The window object.
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

	/**
	 * \brief Reset state to start statistics gathering for a new frame.
	 * 
	 * This may only be called after a successful setWindow() call.
	 */
	void startFrame()
	{
		(this->*startFrame_)();
	}

	/**
	 * \brief Process line 0.
	 * \param[in] y The y coordinate.
	 * \param[in] src The input data.
	 * \param[in] stride The stride.
	 *
	 * This function processes line 0 for input formats with patternSize height == 1.
	 * It'll process line 0 and 1 for input formats with patternSize height >= 2.
	 * This function may only be called after a successful setWindow() call.
	 */
	void processLine0(unsigned int y, const uint8_t *src, unsigned int stride)
	{
		if ((y & y_skip_mask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats0_)(src + window_.x * bpp_ / 8, stride);
	}

	/**
	 * \brief Process line 2 and 3.
	 * \param[in] y The y coordinate.
	 * \param[in] src The input data.
	 * \param[in] stride The stride.
	 *
	 * This function processes line 2 and 3 for input formats with patternSize height == 4.
	 * This function may only be called after a successful setWindow() call.
	 */
	void processLine2(unsigned int y, const uint8_t *src, unsigned int stride)
	{
		if ((y & y_skip_mask_) || y < (unsigned int)window_.y ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats2_)(src + window_.x * bpp_ / 8, stride);
	}

	/**
	 * \brief Finish statistics calculation for the current frame.
	 * 
	 * This may only be called after a successful setWindow() call.
	 */
	void finishFrame()
	{
		(this->*finishFrame_)();
	}

	/**
	 * \brief Signals that the statistics are ready.
	 *
	 * The int parameter isn't actually used.
	 */
	Signal<int> statsReady;
};

} /* namespace libcamera */
