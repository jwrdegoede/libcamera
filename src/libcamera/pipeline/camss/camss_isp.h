/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS ISP virtual base class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <memory>
#include <string>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
struct StreamConfiguration;

class CamssIsp
{
public:
	struct Pipe {
		Size input;
		Size main;
		Size viewfinder;
	};

	int init(std::shared_ptr<MediaDevice> media, unsigned int index);

	PipeConfig calculatePipeConfig(Pipe *pipe);

	int configure(const PipeConfig &pipeConfig, V4L2DeviceFormat *inputFormat);

	int configureOutput(const StreamConfiguration &cfg,
			    V4L2DeviceFormat *outputFormat)
	{
		return configureVideoDevice(output_.get(), PAD_OUTPUT, cfg,
					    outputFormat);
	}

	int configureViewfinder(const StreamConfiguration &cfg,
				V4L2DeviceFormat *outputFormat)
	{
		return configureVideoDevice(viewfinder_.get(), PAD_VF, cfg,
					    outputFormat);
	}

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int start();
	int stop();

	int enableLinks(bool enable);

	std::unique_ptr<V4L2Subdevice> imgu_;
	std::unique_ptr<V4L2VideoDevice> input_;
	std::unique_ptr<V4L2VideoDevice> param_;
	std::unique_ptr<V4L2VideoDevice> output_;
	std::unique_ptr<V4L2VideoDevice> viewfinder_;
	std::unique_ptr<V4L2VideoDevice> stat_;

	std::vector<std::unique_ptr<FrameBuffer>> paramBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statBuffers_;

private:
	static constexpr unsigned int PAD_INPUT = 0;
	static constexpr unsigned int PAD_PARAM = 1;
	static constexpr unsigned int PAD_OUTPUT = 2;
	static constexpr unsigned int PAD_VF = 3;
	static constexpr unsigned int PAD_STAT = 4;

	int linkSetup(const std::string &source, unsigned int sourcePad,
		      const std::string &sink, unsigned int sinkPad,
		      bool enable);

	int configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				 const StreamConfiguration &cfg,
				 V4L2DeviceFormat *outputFormat);

	std::string name_;
	std::shared_ptr<MediaDevice> media_;
};

} /* namespace libcamera */
