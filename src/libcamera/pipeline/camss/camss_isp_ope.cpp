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

#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/converter.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"
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
	sharedParams_ = SharedMemObject<DebayerParams>("softIsp_params");
	if (!sharedParams_) {
		LOG(Camss, Error) << "Failed to create shared memory for parameters";
		return;
	}

	const GlobalConfiguration &configuration = pipe->cameraManager()->_d()->configuration();

	stats_ = std::make_unique<SwStatsCpu>(configuration);
	if (!stats_->isValid()) {
		LOG(Camss, Error) << "Failed to create SwStatsCpu object";
		return;
	}

	stats_->statsReady.connect(this,
				   [&](uint32_t frame, uint32_t bufferId) {
					   statsReady.emit(frame, bufferId);
				   });

	ipa_ = IPAManager::createIPA<ipa::soft::IPAProxySoft>(pipe, "simple", 0, 0);
	if (!ipa_) {
		LOG(Camss, Error) << "Creating IPA failed";
		return;
	}

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor->model() + ".yaml", "uncalibrated.yaml");

	IPACameraSensorInfo sensorInfo{};
	int ret = sensor->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(Camss, Error) << "Camera sensor information not available";
		ipa_.reset();
		return;
	}

	bool ccmEnabled;
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 stats_->getStatsFD(),
			 sharedParams_.fd(),
			 sensorInfo,
			 sensor->controls(),
			 ispControls,
			 &ccmEnabled);
	if (ret) {
		LOG(Camss, Error) << "IPA init failed";
		ipa_.reset();
		return;
	}

	/* ipa_->saveIspParams signal is ignored because M2M OPE has not params */
	ipa_->metadataReady.connect(this,
				    [&](uint32_t frame, const ControlList &metadata) {
						metadataReady.emit(frame, metadata);
				    });
	ipa_->setSensorControls.connect(this,
					[&](const ControlList &sensorControls) {
						setSensorControls.emit(sensorControls);
					});
}

CamssIspOpe::~CamssIspOpe() = default;

bool CamssIspOpe::isValid()
{
	return !!ipa_;
}

StreamConfiguration CamssIspOpe::generateConfiguration(const StreamConfiguration &raw) const
{
	/* Converters support the same size-ranges for all output formats */
	std::vector<PixelFormat> pixelFormats = converter_->formats(raw.pixelFormat);
	SizeRange sizes = converter_->sizes(raw.size);

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

	std::vector<PixelFormat> formats = converter_->formats(raw.pixelFormat);
	SizeRange sizes = converter_->sizes(raw.size);

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

int CamssIspOpe::configure(const StreamConfiguration &inputCfg,
			   const StreamConfiguration &outputCfg)
{
	ipa::soft::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor_->controls();

	int ret = ipa_->configure(configInfo);
	if (ret < 0)
		return ret;

	ret = stats_->configure(inputCfg);
	if (ret < 0)
		return ret;

	/* Use 2/3 center of image to reduce CPU load */
	Rectangle statsWindow;
	statsWindow.width = inputCfg.size.width * 2 / 3;
	statsWindow.height = inputCfg.size.height * 2 / 3;
	statsWindow.x = (inputCfg.size.width - statsWindow.width) / 2;
	statsWindow.y = (inputCfg.size.height - statsWindow.height) / 2;
	/* stats_->setWindow() takes care of necessary alignment itself */
	stats_->setWindow(statsWindow);

	std::vector<std::reference_wrapper<const StreamConfiguration>> outputCfgs;
	outputCfgs.push_back(outputCfg);

	ret = converter_->configure(inputCfg, outputCfgs);
	if (ret)
		return ret;

	converter_->inputBufferReady.connect(this,
					     [&](FrameBuffer *f) {
						     inputBufferReady.emit(f);
					     });
	converter_->outputBufferReady.connect(this,
					      [&](FrameBuffer *f) {
						      outputBufferReady.emit(f);
					      });

	return 0;
}

int CamssIspOpe::exportOutputBuffers(const Stream *stream, unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return converter_->exportBuffers(stream, count, buffers);
}

void CamssIspOpe::queueBuffers(Request *request, FrameBuffer *input)
{
	ipa_->queueRequest(request->sequence(), request->controls());
	/* Calculate stats for the whole frame */
	stats_->processFrame(request->sequence(), 0, input);

	std::map<const Stream *, FrameBuffer *> outputs;
	for (const auto &[stream, outbuffer] : request->buffers()) {
		if (stream == &outStream_)
			outputs[stream] = outbuffer;
	}

	converter_->queueBuffers(input, outputs);
}

void CamssIspOpe::processStats(const uint32_t frame, const uint32_t bufferId,
			       const ControlList &sensorControls)
{
	ipa_->processStats(frame, bufferId, sensorControls);
}

int CamssIspOpe::start()
{
	int ret = ipa_->start();
	if (ret)
		return ret;

	ret = converter_->start();
	if (ret) {
		ipa_->stop();
		return ret;
	}

	return 0;
}

void CamssIspOpe::stop()
{
	converter_->stop();
	ipa_->stop();
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
