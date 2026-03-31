/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS OPE ISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <memory>
#include <vector>

#include "camss_isp.h"

namespace libcamera {

class CameraSensor;
class ControlInfoMap;
class Converter;
class DeviceEnumerator;
class MediaDevice;
class PipelineHandler;
class SoftwareIsp;

class CamssIspOpe : public CamssIsp
{
public:
	static std::unique_ptr<CamssIspOpe> match(PipelineHandler *pipe,
						  DeviceEnumerator *enumerator,
						  const CameraSensor *sensor,
						  ControlInfoMap *ispControls);

	CamssIspOpe(PipelineHandler *pipe, const CameraSensor *sensor, ControlInfoMap *ispControls);
	~CamssIspOpe();

	bool isValid();

	StreamConfiguration generateConfiguration(const StreamConfiguration &raw) const;
	StreamConfiguration validate(const StreamConfiguration &raw, const StreamConfiguration &req) const;
	int configure(const StreamConfiguration &cfg, const V4L2DeviceFormat *inputFormat,
		      V4L2DeviceFormat *outputFormat);

	int exportOutputBuffers(const Stream *stream, unsigned int count,
				std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	void queueBuffers(Request *request, FrameBuffer *input);

	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls);

	int start();
	void stop();

private:
	std::shared_ptr<MediaDevice> opeMediaDev_;
	std::unique_ptr<Converter> converter_;
	const CameraSensor *sensor_;
};

} /* namespace libcamera */
