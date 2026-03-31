/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS OPE ISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_isp_ope.h"

#include <libcamera/base/log.h>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/converter.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

/**
 * \class CamssIspOpe
 * \brief CAMSS ISP class for the Offline Processing Engine (OPE) ISP
 */

/**
 * \brief Constructs CamssIspOpe object
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
CamssIspOpe::CamssIspOpe([[maybe_unused]] PipelineHandler *pipe, const CameraSensor *sensor, [[maybe_unused]] ControlInfoMap *ispControls)
	: sensor_(sensor)
{
#if 0
	swIsp_ = std::make_unique<SoftwareIsp>(pipe, sensor, ispControls);

	swIsp_->ispStatsReady.connect(this,
				      [&](uint32_t frame, uint32_t bufferId) {
						statsReady.emit(frame, bufferId);
				      });
	swIsp_->metadataReady.connect(this,
				      [&](uint32_t frame, const ControlList &metadata) {
						metadataReady.emit(frame, metadata);
				      });
	swIsp_->setSensorControls.connect(this,
					  [&](const ControlList &sensorControls) {
						setSensorControls.emit(sensorControls);
					  });
#endif
}

CamssIspOpe::~CamssIspOpe() = default;

bool CamssIspOpe::isValid()
{
	return true;
}

StreamConfiguration CamssIspOpe::generateConfiguration(const StreamConfiguration &raw) const
{
	/*
	 * The raw stream config may contain multiple format <-> sizes tupples.
	 * Since the OPE only supports only a single output pixel-format and does
	 * downscaling, simply use the best size, which should be preset in raw.size
	 * and also use the input pixel-format which matches that.
	 */
	SizeRange sizes = converter_->sizes(raw.size);
	std::vector<PixelFormat> pixelFormats = converter_->formats(raw.pixelFormat);

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

StreamConfiguration CamssIspOpe::validate(const StreamConfiguration &raw, const StreamConfiguration &req) const
{
	StreamConfiguration cfg;

	SizeRange sizes = converter_->sizes(raw.size);
	std::vector<PixelFormat> formats = converter_->formats(raw.pixelFormat);

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
		converter_->strideAndFrameSize(cfg.pixelFormat, cfg.size);

	cfg.bufferCount = std::max(kBufferCount, req.bufferCount);
	cfg.setStream(const_cast<Stream *>(&outStream_));

	return cfg;
}

int CamssIspOpe::configure(const StreamConfiguration &cfg, const V4L2DeviceFormat *inputFormat,
			   V4L2DeviceFormat *outputFormat)
{
	const std::vector<V4L2PixelFormat> &outputV4L2Formats =
		V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);

	if (outputV4L2Formats.empty()) {
		LOG(Camss, Error)
			<< "No V4L2Format for pixel format " << cfg.pixelFormat;
		return -EINVAL;
	}

	StreamConfiguration inputCfg;
	inputCfg.size = inputFormat->size;
	inputCfg.pixelFormat = inputFormat->fourcc.toPixelFormat();
	inputCfg.stride = inputFormat->planes[0].bpl;

	std::vector<std::reference_wrapper<const StreamConfiguration>> outputCfgs;
	outputCfgs.push_back(cfg);

	int ret = converter_->configure(inputCfg, outputCfgs);
	if (ret)
		return ret;

	converter_->inputBufferReady.connect(this,
					     [&](FrameBuffer *f) { inputBufferReady.emit(f); });
	converter_->outputBufferReady.connect(this,
					      [&](FrameBuffer *f) {
						      // HACK FIXME
						      metadataReady.emit(f->metadata().sequence, {});
						      outputBufferReady.emit(f);
					      });

	outputFormat->size = cfg.size;
	outputFormat->fourcc = outputV4L2Formats[0];
	outputFormat->planes[0].bpl = cfg.stride;
	outputFormat->planes[0].size = cfg.frameSize;

	return 0;
}

int CamssIspOpe::exportOutputBuffers(const Stream *stream, unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return converter_->exportBuffers(stream, count, buffers);
}

void CamssIspOpe::queueBuffers(Request *request, FrameBuffer *input)
{
	std::map<const Stream *, FrameBuffer *> outputs;
	for (const auto &[stream, outbuffer] : request->buffers()) {
		if (stream == &outStream_)
			outputs[stream] = outbuffer;
	}

	//	swIsp_->queueRequest(request->sequence(), request->controls());
	converter_->queueBuffers(input, outputs);
}

void CamssIspOpe::processStats([[maybe_unused]] const uint32_t frame, [[maybe_unused]] const uint32_t bufferId,
			       [[maybe_unused]] const ControlList &sensorControls)
{
	//	swIsp_->processStats(frame, bufferId, sensorControls);
}

int CamssIspOpe::start()
{
	return converter_->start();
}

void CamssIspOpe::stop()
{
	converter_->stop();
}

std::unique_ptr<CamssIspOpe> CamssIspOpe::match(PipelineHandler *pipe,
						DeviceEnumerator *enumerator,
						const CameraSensor *sensor,
						ControlInfoMap *ispControls)
{
	std::unique_ptr<CamssIspOpe> ope = std::make_unique<CamssIspOpe>(pipe, sensor, ispControls);

	DeviceMatch opeDm("qcom-camss-ope");

	ope->opeMediaDev_ = pipe->acquireMediaDevice(enumerator, opeDm);
	if (!ope->opeMediaDev_)
		return nullptr;

	ope->converter_ = ConverterFactoryBase::create(ope->opeMediaDev_);
	if (!ope->converter_)
		return nullptr;

	LOG(Camss, Info) << "Using OPE for " << sensor->entity()->name();

	return ope;
}

} /* namespace libcamera */
