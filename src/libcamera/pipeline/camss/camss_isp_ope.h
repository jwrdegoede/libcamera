/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS OPE ISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_proxy.h>

/* FIXME drop once using own camss IPA, which will map on IPA side */
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/debayer_params.h"

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

class CamssIspOpe : public CamssIsp
{
public:
	static std::unique_ptr<CamssIspOpe> match(PipelineHandler *pipe,
						  DeviceEnumerator *enumerator,
						  const CameraSensor *sensor,
						  CamssFrames *frameInfos,
						  ControlInfoMap *ispControls);

	bool isValid() override;

	CamssIspOpe(PipelineHandler *pipe, std::shared_ptr<MediaDevice> opeMediaDev,
		    const CameraSensor *sensor, CamssFrames *frameInfos,
		    ControlInfoMap *ispControls);
	~CamssIspOpe() override;

	StreamConfiguration generateConfiguration(const StreamConfiguration &raw) const override;
	StreamConfiguration validate(const StreamConfiguration &raw, const StreamConfiguration &req) const override;
	int configure(const StreamConfiguration &inputCfg,
		      const StreamConfiguration &outputCfg) override;

	int allocateBuffers(unsigned int bufferCount) override;
	void freeBuffers() override;
	int exportOutputBuffers(const Stream *stream, unsigned int count,
				std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	void queueBuffers(Request *request, FrameBuffer *input) override;

	void paramsComputed(uint32_t frame);
	void paramsBufferReady(FrameBuffer *f);
	void statsReady(uint32_t frame, uint32_t bufferId);

	int start() override;
	void stop() override;

private:
	static constexpr Size kMinOutputSize = Size(24, 16);

	int init();
	int trySetCfg(V4L2VideoDevice *v4l2Dev, const StreamConfiguration &cfg,
		      bool set, const char *msgPrefix) const;

	std::shared_ptr<MediaDevice> opeMediaDev_;
	const CameraSensor *sensor_;
	const Stream *rawStream_;
	class CamssFrames *frameInfos_;
	SharedMemObject<DebayerParams> sharedParams_;
	std::unique_ptr<SwStatsCpu> stats_;
	std::unique_ptr<V4L2VideoDevice> params_;
	std::unique_ptr<V4L2VideoDevice> input_;
	std::unique_ptr<V4L2VideoDevice> output_;
	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
	std::vector<std::unique_ptr<FrameBuffer>> paramBuffers_;
	std::vector<IPABuffer> ipaBuffers_;
	/* FIXME drop once using own camss IPA, which will map on IPA side */
	std::map<unsigned int, MappedFrameBuffer> mappedIpaBuffers_;
	unsigned int inputBufferCount_;
	unsigned int outputBufferCount_;
};

} /* namespace libcamera */
