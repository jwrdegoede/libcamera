/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * debayer.h - debayering base class
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/software_isp/debayer_params.h"

namespace libcamera {

class FrameBuffer;

LOG_DECLARE_CATEGORY(Debayer)

class Debayer
{
public:
	virtual ~Debayer() = 0;

	/*
	 * Setup the Debayer object according to the passed in parameters.
	 * Return 0 on success, a negative errno value on failure
	 * (unsupported parameters).
	 */
	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs) = 0;

	/*
	 * Get width and height at which the bayer-pattern repeats.
	 * Return pattern-size or an empty Size for an unsupported inputFormat.
	 */
	virtual Size patternSize(PixelFormat inputFormat) = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat inputFormat) = 0;
	virtual std::tuple<unsigned int, unsigned int>
		strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) = 0;

	/*
	 * Note DebayerParams is passed by value deliberately so that a copy is passed
	 * when this is run in another thread to invokeMethod.
	 */
	virtual void process(FrameBuffer *input, FrameBuffer *output, DebayerParams params) = 0;

	/* sizes() - Get supported output sizes for given input fmt + size */
	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize)
	{
		Size pattern_size = patternSize(inputFormat);

		if (pattern_size.isNull())
			return {};

		/*
		 * For debayer interpolation a border of pattern-height x pattern-width
		 * is kept around the entire image. Combined with a minimum-size of
		 * pattern-height x pattern-width this means the input-size needs to be
		 * at least (3 * pattern-height) x (3 * pattern-width).
		 */
		if (inputSize.width < (3 * pattern_size.width) ||
		    inputSize.height < (3 * pattern_size.height)) {
			LOG(Debayer, Warning)
				<< "Input format size too small: " << inputSize.toString();
			return {};
		}

		return SizeRange(Size(pattern_size.width, pattern_size.height),
				 Size((inputSize.width - 2 * pattern_size.width) & ~(pattern_size.width - 1),
				      (inputSize.height - 2 * pattern_size.height) & ~(pattern_size.height - 1)),
				 pattern_size.width, pattern_size.height);
	}

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;
};

} /* namespace libcamera */
