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
#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/pipeline_handler.h"

namespace libcamera {

class FrameBuffer;
class PixelFormat;
struct StreamConfiguration;

LOG_DECLARE_CATEGORY(SoftwareIsp)

class SoftwareIsp
{
public:
	SoftwareIsp(PipelineHandler *pipe, const ControlInfoMap &sensorControls);
	virtual ~SoftwareIsp();

	virtual int loadConfiguration(const std::string &filename) = 0;

	virtual bool isValid() const = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat input) = 0;
	virtual SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) = 0;

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

	virtual void processStats(const ControlList &sensorControls) = 0; // rather merge with queueBuffers()?

	virtual Signal<const ControlList &> &getSignalSetSensorControls() = 0;

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

	/* The int parameter isn't actually used */
	Signal<int> ispStatsReady;
};

class SoftwareIspFactoryBase
{
public:
	SoftwareIspFactoryBase();
	virtual ~SoftwareIspFactoryBase() = default;

	static std::unique_ptr<SoftwareIsp> create(PipelineHandler *pipe,
						   const ControlInfoMap &sensorControls);
	static SoftwareIspFactoryBase *&factory();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(SoftwareIspFactoryBase)

	static void registerType(SoftwareIspFactoryBase *factory);
	virtual std::unique_ptr<SoftwareIsp> createInstance(PipelineHandler *pipe,
							    const ControlInfoMap &sensorControls) const = 0;
};

template<typename _SoftwareIsp>
class SoftwareIspFactory : public SoftwareIspFactoryBase
{
public:
	SoftwareIspFactory()
		: SoftwareIspFactoryBase()
	{
	}

	std::unique_ptr<SoftwareIsp> createInstance(PipelineHandler *pipe,
						    const ControlInfoMap &sensorControls) const override
	{
		return std::make_unique<_SoftwareIsp>(pipe, sensorControls);
	}
};

#define REGISTER_SOFTWAREISP(softwareIsp) \
	static SoftwareIspFactory<softwareIsp> global_##softwareIsp##Factory;

} /* namespace libcamera */
