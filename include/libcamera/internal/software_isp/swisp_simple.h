/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_simple.h - Simple software ISP implementation
 */

#pragma once

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_proxy.h>

#include "libcamera/internal/software_isp.h"
#include "libcamera/internal/software_isp/debayer_cpu.h"

namespace libcamera {

class SwIspSimple : public SoftwareIsp
{
public:
	SwIspSimple(PipelineHandler *pipe, const ControlInfoMap &sensorControls);
	~SwIspSimple() {}

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
	void processStats(const ControlList &sensorControls);

	int start();
	void stop();

	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

	Signal<const ControlList &> &getSignalSetSensorControls();

	void process(FrameBuffer *input, FrameBuffer *output);

private:
	void statsReady(int dummy);
	void inputReady(FrameBuffer *input);
	void outputReady(FrameBuffer *output);

	std::unique_ptr<DebayerCpu> debayer_;
	Thread ispWorkerThread_;
	/* FIXME debayerParams should come from the IPA */
	DebayerParams debayerParams_;

	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
};

} /* namespace libcamera */
