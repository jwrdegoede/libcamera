/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS softISP class
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
class PipelineHandler;
class SoftwareIsp;

class CamssIspSoft : public CamssIsp
{
public:
	CamssIspSoft(PipelineHandler *pipe, const CameraSensor *sensor, ControlInfoMap *ispControls);
	~CamssIspSoft();

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
	std::unique_ptr<SoftwareIsp> swIsp_;
	const CameraSensor *sensor_;
};

} /* namespace libcamera */
