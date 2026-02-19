/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS softISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_isp_soft.h"

#include <libcamera/base/log.h>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/software_isp.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "camss_frames.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

/**
 * \class CamssIspSoft
 * \brief CAMSS ISP class using the Software ISP
 */

/**
 * \brief Constructs CamssIspSoft object
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
CamssIspSoft::CamssIspSoft(PipelineHandler *pipe, const CameraSensor *sensor,
			   CamssFrames *frameInfos, ControlInfoMap *ispControls)
	: sensor_(sensor), frameInfos_(frameInfos)
{
	swIsp_ = std::make_unique<SoftwareIsp>(pipe, sensor, ispControls);

	swIsp_->inputBufferReady.connect(this,
					 [&](FrameBuffer *f) { inputBufferReady.emit(f); });
	swIsp_->outputBufferReady.connect(this,
					  [&](FrameBuffer *f) { outputBufferReady.emit(f); });
	swIsp_->ispStatsReady.connect(this,
				      [&](uint32_t frame, uint32_t statsBufferId) {
					      statsReady.emit(frame, statsBufferId);
				      });
	swIsp_->metadataReady.connect(this,
				      [&](uint32_t frame, const ControlList &metadata) {
					      metadataReady.emit(frame, metadata);
				      });
	swIsp_->setSensorControls.connect(this,
					  [&](const ControlList &sensorControls) {
						  setSensorControls.emit(sensorControls);
					  });
}

CamssIspSoft::~CamssIspSoft() = default;

bool CamssIspSoft::isValid()
{
	return swIsp_->isValid();
}

Size CamssIspSoft::getMargins(PixelFormat inputFormat)
{
	Size trySize(1024, 1024);
	SizeRange sizes = swIsp_->sizes(inputFormat, trySize);

	return Size(1024 - sizes.max.width, 1024 - sizes.max.height);
}

StreamConfiguration CamssIspSoft::generateConfiguration(const StreamConfiguration &raw) const
{
	/*
	 * The raw stream config may contain multiple format <-> sizes tupples.
	 * Since the softIsp can always crop / downscale and since it supports
	 * the same set of output pixel-formats for all supported input pixel-
	 * formats, simply use the best size, which should be preset in raw.size
	 * and also use the input pixel-format which matches that.
	 */
	SizeRange sizes = swIsp_->sizes(raw.pixelFormat, raw.size);
	std::vector<PixelFormat> pixelFormats = swIsp_->formats(raw.pixelFormat);

	if (sizes.max.isNull() || pixelFormats.empty())
		return {};

	std::vector<SizeRange> sizesVector = { sizes };
	std::map<PixelFormat, std::vector<SizeRange>> formats;

	for (unsigned int i = 0; i < pixelFormats.size(); i++)
		formats[pixelFormats[i]] = sizesVector;

	StreamConfiguration cfg{ StreamFormats{ formats } };
	cfg.size = sizes.max;
	cfg.pixelFormat = pixelFormats[0];
	cfg.bufferCount = kBufferCount;

	return cfg;
}

namespace {

/*
 * \todo copy-pasted from src/libcamera/pipeline/simple/simple.cpp turn this
 * into a member of SizeRange ?
 * \todo also see V4L2M2MConverter::adjustSizes() which is also similar.
 */
static Size adjustSize(const Size &requestedSize, const SizeRange &supportedSizes)
{
	ASSERT(supportedSizes.min <= supportedSizes.max);

	if (supportedSizes.min == supportedSizes.max)
		return supportedSizes.max;

	unsigned int hStep = supportedSizes.hStep;
	unsigned int vStep = supportedSizes.vStep;

	if (hStep == 0)
		hStep = supportedSizes.max.width - supportedSizes.min.width;
	if (vStep == 0)
		vStep = supportedSizes.max.height - supportedSizes.min.height;

	Size adjusted = requestedSize.boundedTo(supportedSizes.max)
				.expandedTo(supportedSizes.min);

	return adjusted.shrunkBy(supportedSizes.min)
		.alignedDownTo(hStep, vStep)
		.grownBy(supportedSizes.min);
}

} /* namespace */

StreamConfiguration CamssIspSoft::validate(const StreamConfiguration &raw, const StreamConfiguration &req) const
{
	StreamConfiguration cfg;

	SizeRange sizes = swIsp_->sizes(raw.pixelFormat, raw.size);
	std::vector<PixelFormat> formats = swIsp_->formats(raw.pixelFormat);

	cfg.size = adjustSize(req.size, sizes);

	if (cfg.size.isNull() || formats.empty())
		return {};

	for (unsigned int i = 0; i < formats.size(); i++) {
		if (formats[i] == req.pixelFormat)
			cfg.pixelFormat = req.pixelFormat;
	}

	if (!cfg.pixelFormat.isValid())
		cfg.pixelFormat = formats[0];

	std::tie(cfg.stride, cfg.frameSize) =
		swIsp_->strideAndFrameSize(cfg.pixelFormat, cfg.size);

	cfg.bufferCount = std::max(kBufferCount, req.bufferCount);

	return cfg;
}

int CamssIspSoft::configure(const StreamConfiguration &inputCfg,
			    const StreamConfiguration &outputCfg)
{
	std::vector<std::reference_wrapper<const StreamConfiguration>> outputCfgs;
	outputCfgs.push_back(outputCfg);

	/* \todo refactor SoftwareIsp to remove the need to pass this */
	ipa::soft::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor_->controls();

	return swIsp_->configure(inputCfg, outputCfgs, configInfo);
}

int CamssIspSoft::allocateBuffers([[maybe_unused]] unsigned int bufferCount)
{
	frameInfos_->init({}, {}, true);
	return 0;
}

void CamssIspSoft::freeBuffers()
{
	frameInfos_->clear();
}

int CamssIspSoft::exportOutputBuffers(const Stream *stream, unsigned int count,
				      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return swIsp_->exportBuffers(stream, count, buffers);
}

void CamssIspSoft::queueBuffers(Request *request, FrameBuffer *input)
{
	std::map<const Stream *, FrameBuffer *> outputs;
	for (const auto &[stream, outbuffer] : request->buffers()) {
		if (stream == &outStream_)
			outputs[stream] = outbuffer;
	}

	swIsp_->queueRequest(request->sequence(), request->controls());
	swIsp_->queueBuffers(request->sequence(), input, outputs);
}

void CamssIspSoft::processStats(const uint32_t frame, const uint32_t statsBufferId,
				const ControlList &sensorControls)
{
	swIsp_->processStats(frame, statsBufferId, sensorControls);
}

int CamssIspSoft::start()
{
	return swIsp_->start();
}

void CamssIspSoft::stop()
{
	swIsp_->stop();
}

} /* namespace libcamera */
