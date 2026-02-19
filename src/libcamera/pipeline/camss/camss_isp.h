/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS ISP virtual base class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <stdint.h>
#include <vector>

#include <libcamera/base/signal.h>

#include <libcamera/stream.h>

namespace libcamera {

class ControlList;
class FrameBuffer;
struct StreamConfiguration;
struct V4L2DeviceFormat;

class CamssIsp
{
public:
	virtual ~CamssIsp() = 0;

	virtual bool isValid() = 0;

	virtual StreamConfiguration generateConfiguration(const StreamConfiguration &raw) const = 0;
	virtual StreamConfiguration validate(const StreamConfiguration &raw, const StreamConfiguration &req) const = 0;
	virtual int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg) = 0;

	virtual int allocateBuffers(unsigned int bufferCount) = 0;
	virtual void freeBuffers() = 0;
	virtual int exportOutputBuffers([[maybe_unused]] const Stream *stream,
					[[maybe_unused]] unsigned int count,
					[[maybe_unused]] std::vector<std::unique_ptr<FrameBuffer>> *buffers) { return 0; }
	virtual void queueBuffers(Request *request, FrameBuffer *input) = 0;

	virtual void processStats(const uint32_t frame, const uint32_t statsBufferId,
				  const ControlList &sensorControls) = 0;

	virtual int start() = 0;
	virtual void stop() = 0;

	static constexpr unsigned int kBufferCount = 4;

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;
	Signal<FrameBuffer *> paramBufferReady;
	Signal<uint32_t, uint32_t> statsReady;
	Signal<uint32_t, const ControlList &> metadataReady;
	Signal<const ControlList &> setSensorControls;
	Stream outStream_;
};

} /* namespace libcamera */
