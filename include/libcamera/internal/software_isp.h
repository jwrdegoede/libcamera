/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * software_isp.h - Interface for a software implementation of an ISP
 */

#pragma once

#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

namespace libcamera {

class FrameBuffer;
class PixelFormat;
struct StreamConfiguration;

class SoftwareIsp
{
public:
	SoftwareIsp(const std::string &name);
	virtual ~SoftwareIsp();

	virtual int loadConfiguration(const std::string &filename) = 0;

	virtual bool isValid() const = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat input) = 0;
	virtual SizeRange sizes(const Size &input) = 0;

	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size) = 0;

	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs) = 0;
	virtual int exportBuffers(unsigned int output, unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers) = 0;

	virtual int start() = 0;
	virtual void stop() = 0;

	virtual int queueBuffers(FrameBuffer *input,
				 const std::map<unsigned int, FrameBuffer *> &outputs) = 0;

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

	const std::string &name() const { return name_; }
private:
	std::string name_;
};

} /* namespace libcamera */
