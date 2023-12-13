/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_linaro.h - software ISP implementation by Linaro
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/software_isp/debayer_cpu.h"
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
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
	{
		return debayer_->configure(inputCfg, outputCfgs);
	}

	int exportBuffers(unsigned int output, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start();
	void stop();

	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

	const SharedFD &getStatsFD() { return debayer_->getStatsFD(); }

	void process(FrameBuffer *input, FrameBuffer *output);

private:
	void statsReady(int dummy);
	void inputReady(FrameBuffer *input);
	void outputReady(FrameBuffer *output);

	std::unique_ptr<DebayerCpu> debayer_;
	Thread ispWorkerThread_;
	/* FIXME debayerParams should come from the IPA */
	DebayerParams debayerParams_;
};

} /* namespace libcamera */
