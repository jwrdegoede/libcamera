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

/**
 * \class Debayer
 * \brief Base debayering class
 *
 * Base class that provides functions for setting up the debayering process.
 */
class Debayer
{
public:
	virtual ~Debayer() = 0;

	/**
	 * \brief Configure the debayer object according to the passed in parameters.
	 * \param[in] inputCfg The input configuration.
	 * \param[in] outputCfgs The output configurations.
	 *
	 * \return 0 on success, a negative errno on failure.
	 */
	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs) = 0;

	/**
	 * \brief Get the width and height at which the bayer pattern repeats.
	 * \param[in] inputFormat The input format.
	 *
	 * Valid sizes are: 2x2, 4x2 or 4x4.
	 *
	 * \return pattern size or an empty size for unsupported inputFormats.
	 */
	virtual Size patternSize(PixelFormat inputFormat) = 0;

	/**
	 * \brief Get the supported output formats.
	 * \param[in] inputFormat The input format.
	 *
	 * \return all supported output formats or an empty vector if there are none.
	 */
	virtual std::vector<PixelFormat> formats(PixelFormat inputFormat) = 0;

	/**
	 * \brief Get the stride and the frame size.
	 * \param[in] outputFormat The output format.
	 * \param[in] size The output size.
	 *
	 * \return a tuple of the stride and the frame size, or a tuple with 0,0 if there is no valid output config.
	 */
	virtual std::tuple<unsigned int, unsigned int>
		strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) = 0;

	/**
	 * \brief Process the bayer data into the requested format.
	 * \param[in] input The input buffer.
	 * \param[in] output The output buffer.
	 * \param[in] params The parameters to be used in debayering.
	 *
	 * \note DebayerParams is passed by value deliberately so that a copy is passed
	 * when this is run in another thread by invokeMethod().
	 */
	virtual void process(FrameBuffer *input, FrameBuffer *output, DebayerParams params) = 0;

	/**
	 * \brief Get the supported output sizes for the given input format and size.
	 * \param[in] inputFormat The input format.
	 * \param[in] inputSize The input size.
	 *
	 * \return The valid size ranges or an empty range if there are none.
	 */
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

	/**
	 * \brief Signals when the input buffer is ready.
	 */
	Signal<FrameBuffer *> inputBufferReady;

	/**
	 * \brief Signals when the output buffer is ready.
	 */
	Signal<FrameBuffer *> outputBufferReady;
};

} /* namespace libcamera */
