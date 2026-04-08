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

#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/debayer_params.h"

#include "camss_isp.h"

namespace libcamera {

class CameraSensor;
class ControlInfoMap;
class Converter;
class DeviceEnumerator;
class MediaDevice;
class PipelineHandler;
class SwStatsCpu;

class CamssIspOpe : public CamssIsp
{
public:
	static std::unique_ptr<CamssIspOpe> match(PipelineHandler *pipe,
						  DeviceEnumerator *enumerator,
						  const CameraSensor *sensor,
						  ControlInfoMap *ispControls);

	CamssIspOpe(PipelineHandler *pipe, const CameraSensor *sensor, ControlInfoMap *ispControls);
	~CamssIspOpe() override;

	bool isValid() override;

	StreamConfiguration generateConfiguration(const StreamConfiguration &raw) const override;
	StreamConfiguration validate(const StreamConfiguration &raw, const StreamConfiguration &req) const override;
	int configure(const StreamConfiguration &inputCfg,
		      const StreamConfiguration &outputCfg) override;

	int exportOutputBuffers(const Stream *stream, unsigned int count,
				std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	void queueBuffers(Request *request, FrameBuffer *input) override;

	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls) override;

	int start() override;
	void stop() override;

private:
	std::shared_ptr<MediaDevice> opeMediaDev_;
	std::unique_ptr<Converter> converter_;
	const CameraSensor *sensor_;
	SharedMemObject<DebayerParams> sharedParams_;
	std::unique_ptr<SwStatsCpu> stats_;
	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
};

} /* namespace libcamera */
