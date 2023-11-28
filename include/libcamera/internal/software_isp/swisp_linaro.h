/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_linaro.h - software ISP implementation by Linaro
 */

#pragma once

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/statistics-linaro.h"
#include "libcamera/internal/software_isp.h"

namespace libcamera {

class SwIspLinaro : public SoftwareIsp
{
public:
	SwIspLinaro(const std::string &name);
	~SwIspLinaro() {}

	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }
	bool isValid() const;

	std::vector<PixelFormat> formats(PixelFormat input);
	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs);
	int exportBuffers(unsigned int output, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start();
	void stop();

	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

	const SharedFD &getStatsFD() { return sharedStats_.fd(); }

	void process(FrameBuffer *input, FrameBuffer *output);

private:
	SharedMemObject<SwIspStats> sharedStats_;

	class IspWorker : public Object
	{
	public:
		IspWorker(SwIspLinaro *swIsp);

		std::vector<PixelFormat> formats(PixelFormat input);
		SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);
		unsigned int outStride(const PixelFormat &outputFormat,
				       const Size &outSize);

		int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg);
		unsigned int outBufferSize();
		void process(FrameBuffer *input, FrameBuffer *output);

	private:
		SwIspLinaro *swIsp_;

		typedef void (SwIspLinaro::IspWorker::*debayerFn)(uint8_t *dst, const uint8_t *src);
		typedef SizeRange (*outSizesFn)(const Size &inSize);
		typedef unsigned int (*outStrideFn)(const Size &outSize);
		struct debayerInfo {
			PixelFormat outPixelFmt;
			debayerFn debayer;
			outSizesFn getOutSizes;
			outStrideFn getOutStride;
		};
		// TODO: use inputFormat+outputFormat as the map key
		// to enable multiple output formats
		// TODO: use BayerFormat instead of PixelFormat as inputFormat
		std::map<PixelFormat, IspWorker::debayerInfo> debayerInfos_;
		int setDebayerInfo(PixelFormat format);
		debayerInfo *debayerInfo_;

		/* CSI-2 packed 10-bit raw bayer format (all the 4 orders) */
		void debayerRaw10P(uint8_t *dst, const uint8_t *src);
		static SizeRange outSizesRaw10P(const Size &inSize);
		static unsigned int outStrideRaw10P(const Size &outSize);

		unsigned int width_;
		unsigned int height_;
		unsigned int stride_;
		Point redShift_;
		unsigned int outWidth_;
		unsigned int outHeight_;
		unsigned int outStride_;

		unsigned long rNumerat_, rDenomin_; /* red gain for AWB */
		unsigned long bNumerat_, bDenomin_; /* blue gain for AWB */
		unsigned long gNumerat_, gDenomin_; /* green gain for AWB */

		SwIspStats stats_;
	};

	std::unique_ptr<IspWorker> ispWorker_;
	Thread ispWorkerThread_;
};

} /* namespace libcamera */
