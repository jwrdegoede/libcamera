/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS dummy ISP class for raw only image capture.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/ipa/softisp_ipa_interface.h>
#include <libcamera/ipa/softisp_ipa_proxy.h>

/* FIXME drop once using own camss IPA, which will map on IPA side */
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "camss_isp.h"

namespace libcamera {

class CameraSensor;
class CamssFrames;
class ControlInfoMap;
class DeviceEnumerator;
class MediaDevice;
class PipelineHandler;
class Stream;
class SwStatsCpu;
class V4L2VideoDevice;

class CamssIspRaw : public CamssIsp
{
public:
	CamssIspRaw(PipelineHandler *pipe, const CameraSensor *sensor,
		    CamssFrames *frameInfos, ControlInfoMap *ispControls);
	~CamssIspRaw() override;

	bool isValid() override;
	Size getMargins(PixelFormat inputFormat) override;

	StreamConfiguration generateConfiguration(const StreamConfiguration &raw) const override;
	StreamConfiguration validate(const StreamConfiguration &raw, const StreamConfiguration &req) const override;
	int configure(const StreamConfiguration &inputCfg,
		      const StreamConfiguration &outputCfg,
		      ControlInfoMap *ispControls) override;

	int allocateBuffers(unsigned int bufferCount) override;
	void freeBuffers() override;
	int exportOutputBuffers(const Stream *stream, unsigned int count,
				std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	void queueBuffers(Request *request, FrameBuffer *input) override;

	void processStats(const uint32_t frame, const uint32_t statsBufferId,
			  const ControlList &sensorControls) override;

	int start() override;
	void stop() override;

private:
	const CameraSensor *sensor_;
	class CamssFrames *frameInfos_;
	SharedMemObject<DebayerParams> sharedParams_;
	std::unique_ptr<SwStatsCpu> stats_;
	std::unique_ptr<ipa::softisp::IPAProxySoftIsp> ipa_;
};

} /* namespace libcamera */
