/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS CSI phy/decoder and VFE handling
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Partially based on other pipeline-handlers which are:
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 * Copyright (C) 2019, Google Inc.
 */

#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class PixelFormat;
class Request;
class Size;
class SizeRange;
struct StreamConfiguration;
enum class Transform;

class CamssCsiCamera
{
public:
	CamssCsiCamera();

	StreamConfiguration generateConfiguration(void) const;
	StreamConfiguration validate(const StreamConfiguration &req) const;
	int configure(const StreamConfiguration &cfg, const Transform &transform);
	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	PixelFormat mbusCodeToPixelFormat(unsigned int code) const;
	unsigned int PixelFormatToMbusCode(const PixelFormat &format) const;

	bool acquireDevice();
	void releaseDevice();

	int start();
	void stop();

	CameraSensor *sensor() { return sensor_.get(); }
	const CameraSensor *sensor() const { return sensor_.get(); }

	FrameBuffer *queueBuffer(Request *request, FrameBuffer *rawBuffer);
	void tryReturnBuffer(FrameBuffer *buffer);
	Signal<FrameBuffer *> &bufferReady() { return output_->bufferReady; }
	/*
	 * \todo camss kernel driver does not support this atm. Once supported
	 * this needs to take frameStart signal from the csi-decoder.
	 */
	Signal<uint32_t> &frameStart() { return links_[0].sinkSubdev->frameStart; }
	bool supportsFrameStart() { return false; }

	Signal<> bufferAvailable;
	Stream rawStream_;

private:
	friend class CamssCsi;

	static constexpr unsigned int kBufferCount = 4;

	/* 3 links: sensor -> phy, phy -> csid, csid->vfe */
	enum LinkIndex {
		SensorPhyLink,
		PhyCsidLink,
		CsidVfeLink,
		LinkCount
	};

	struct linkInfo {
		MediaLink *link;
		std::unique_ptr<V4L2Subdevice> sinkSubdev;
	};

	V4L2PixelFormat mbusCodeToV4L2PixelFormat(unsigned int code) const;
	V4L2SubdeviceFormat getSensorFormat(Size size = {}, const PixelFormat &format = {}) const;
	int tryFormat(const V4L2SubdeviceFormat &sensorFormat, StreamConfiguration &cfg) const;
	void freeBuffers();

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2VideoDevice> output_;
	std::array<linkInfo, LinkCount> links_;

	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::queue<FrameBuffer *> availableBuffers_;
	unsigned int bufferCount_;
};

class CamssCsi
{
public:
	using Cameras = std::vector<std::unique_ptr<CamssCsiCamera>>;

	CamssCsi();
	Cameras match(PipelineHandler *pipe, DeviceEnumerator *enumerator);
private:
	void getEntities(std::vector<MediaEntity *> &ents, const char *fmt, unsigned int max);
	std::unique_ptr<CamssCsiCamera> enumCamera(MediaEntity *phy);

	static constexpr unsigned int kMaxCsiPhys = 5;
	static constexpr unsigned int kMinCsiDecoders = 2;
	static constexpr unsigned int kMaxCsiDecoders = 7;
	static constexpr unsigned int kMinVfes = 2;
	static constexpr unsigned int kMaxVfes = 7;
	std::shared_ptr<MediaDevice> camssMediaDev_;
	std::vector<MediaEntity *> phys_;
	std::vector<MediaEntity *> csids_;
	std::vector<MediaEntity *> vfes_;
};

} /* namespace libcamera */
