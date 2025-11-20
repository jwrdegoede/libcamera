/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * debayering base class
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/global_configuration.h"
#include "libcamera/internal/software_isp/benchmark.h"
#include "libcamera/internal/software_isp/debayer_params.h"

namespace libcamera {

class FrameBuffer;

LOG_DECLARE_CATEGORY(Debayer)

class Debayer : public Object
{
public:
	Debayer (const GlobalConfiguration &configuration);
	virtual ~Debayer() = 0;

	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			      bool ccmEnabled) = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat inputFormat) = 0;

	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) = 0;

	virtual void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params) = 0;

	virtual SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) = 0;

	/**
	 * \brief Get the file descriptor for the statistics
	 *
	 * \return the file descriptor pointing to the statistics
	 */
	virtual const SharedFD &getStatsFD() = 0;

	/**
	 * \brief Get the output frame size
	 *
	 * \return The output frame size
	 */
	unsigned int frameSize() { return outputConfig_.frameSize; }

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

	/**
	 * struct DebayerInputConfig
	 *
	 * Structure to describe the incoming Bayer parameters.
	 */
	struct DebayerInputConfig {
		Size patternSize;			/**< patternSize size of the Bayer pattern in pixels */
		unsigned int bpp;			/**< bpp Memory used per pixel, not precision */
		unsigned int stride;			/**< stride Line stride in bytes */
		std::vector<PixelFormat> outputFormats;	/**< outputFormats List of supported output pixel formats */
	};

	/**
	 * struct DebayerOutputConfig
	 *
	 * Structure to describe the output pattern requested to the Debayer logic.
	 */
	struct DebayerOutputConfig {
		unsigned int bpp;			/**< bpp Memory used per pixel, not precision */
		unsigned int stride;			/**< stride Line stride in bytes */
		unsigned int frameSize;			/**< framesize Total frame size in bytes */
	};

	DebayerInputConfig inputConfig_;		/**< inputConfig_ debayer input config params */
	DebayerOutputConfig outputConfig_;		/**< outputConfig_ debayer output config data */
	DebayerParams::LookupTable red_;		/**< red_ DebayerParams red_ lookup table */
	DebayerParams::LookupTable green_;		/**< green_ DebayerParams green_ lookup table */
	DebayerParams::LookupTable blue_;		/**< blue_ DebayerParams blue_ lookup table */
	DebayerParams::CcmLookupTable redCcm_;		/**< redCcm_ Red Colour Correction matrix lookup table */
	DebayerParams::CcmLookupTable greenCcm_;	/**< greenCcm_ Green Colour Correction matrix lookup table */
	DebayerParams::CcmLookupTable blueCcm_;		/**< blueCcm_ Blue Colour Correction matrix lookup table */
	DebayerParams::LookupTable gammaLut_;		/**< gammaLut_ Gamma Lut lookup table */
	bool swapRedBlueGains_;				/**< swapRedBlueGains_ bool to indicate swapping of red/blue gains */
	Benchmark bench_;				/**< bench_ an instance of the Benchmark class */

private:
	/**
	 * \fn patternSize(PixelFormat inputFormat)
	 * \var DebayerInputConfig::patternSize
	 * \brief Size of the Bayer pattern in pixels
	 *
	 * The width and height of the Bayer color filter array pattern. For standard
	 * Bayer formats (BGGR, GRBG, GBRG, RGGB) this is typically 2x2 pixels.
	 */
	virtual Size patternSize(PixelFormat inputFormat) = 0;

protected:
	void setParams(DebayerParams &params);
};

} /* namespace libcamera */
