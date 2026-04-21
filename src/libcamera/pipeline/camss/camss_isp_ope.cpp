/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS OPE ISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_isp_ope.h"

#include <string.h>

#include <libcamera/base/log.h>

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "camss_util.h"

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

	/* FIXME ipa_->saveIspParams signal is ignored because M2M OPE has no params */
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

static const std::array camssIspOpeSupportedFormats{
	formats::NV12,
	formats::NV21,
	formats::NV16,
	formats::NV61,
	formats::NV24,
	formats::NV42,
};

int CamssIspOpe::trySetCfg(V4L2VideoDevice *v4l2Dev, const StreamConfiguration &cfg,
			   bool set, const char *msgPrefix) const
{
	int ret;

	V4L2DeviceFormat v4l2Format;
	v4l2Format.fourcc = v4l2Dev->toV4L2PixelFormat(cfg.pixelFormat);
	v4l2Format.size = cfg.size;
	v4l2Format.planes[0].bpl = cfg.stride;
	v4l2Format.planesCount = 1;

	if (set)
		ret = v4l2Dev->setFormat(&v4l2Format);
	else
		ret = v4l2Dev->tryFormat(&v4l2Format);

	if (ret < 0) {
		LOG(Camss, Error) << msgPrefix << " error: " << strerror(-ret);
		return ret;
	}

	if (!camssV4L2DeviceFormatMatchesStreamConfig(v4l2Format, cfg, msgPrefix))
		return -EINVAL;

	return 0;
}

StreamConfiguration CamssIspOpe::generateConfiguration(const StreamConfiguration &raw) const
{
	if (trySetCfg(input_.get(), raw, false, "OPE input try format"))
		return {};

	/* OPE always supports all output formats */
	std::vector<SizeRange> sizesVector = { SizeRange(kMinOutputSize, raw.size, 2, 2) };
	std::map<PixelFormat, std::vector<SizeRange>> formats;
	for (unsigned int i = 0; i < camssIspOpeSupportedFormats.size(); i++)
		formats[camssIspOpeSupportedFormats[i]] = sizesVector;

	StreamConfiguration cfg{ StreamFormats{ formats } };
	cfg.size = raw.size;
	cfg.pixelFormat = camssIspOpeSupportedFormats[0];
	cfg.bufferCount = kBufferCount;

	return cfg;
}

StreamConfiguration CamssIspOpe::validate(const StreamConfiguration &raw, const StreamConfiguration &req) const
{
	/* Must actually set the format here to allow try-fmt on output_ below to work */
	if (trySetCfg(input_.get(), raw, true, "OPE input set format"))
		return {};

	StreamConfiguration cfg;

	for (unsigned int i = 0; i < camssIspOpeSupportedFormats.size(); i++) {
		if (camssIspOpeSupportedFormats[i] == req.pixelFormat)
			cfg.pixelFormat = req.pixelFormat;
	}

	if (!cfg.pixelFormat.isValid())
		cfg.pixelFormat = camssIspOpeSupportedFormats[0];

	if (SizeRange(kMinOutputSize, raw.size, 2, 2).contains(req.size))
		cfg.size = req.size;
	else
		cfg.size = raw.size;

	V4L2DeviceFormat v4l2Format;
	v4l2Format.fourcc = output_->toV4L2PixelFormat(cfg.pixelFormat);
	v4l2Format.size = cfg.size;
	v4l2Format.planes[0].bpl = cfg.stride;
	v4l2Format.planesCount = 1;

	int ret = output_->tryFormat(&v4l2Format);
	if (ret < 0) {
		LOG(Camss, Error) << "OPE output try format error: " << strerror(-ret);
		return {};
	}

	cfg.stride = v4l2Format.planes[0].bpl;
	cfg.frameSize = v4l2Format.planes[0].size;

	if (!camssV4L2DeviceFormatMatchesStreamConfig(v4l2Format, cfg, "OPE output try format"))
		return {};

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

	ret = trySetCfg(input_.get(), inputCfg, true, "OPE input set format");
	if (ret < 0)
		return ret;

	ret = trySetCfg(output_.get(), outputCfg, true, "OPE output set format");
	if (ret < 0)
		return ret;

	input_->bufferReady.connect(this, [&](FrameBuffer *f) {
		inputBufferReady.emit(f);
	});
	output_->bufferReady.connect(this, [&](FrameBuffer *f) {
		outputBufferReady.emit(f);
	});

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	return 0;
}

int CamssIspOpe::exportOutputBuffers([[maybe_unused]] const Stream *stream, unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

void CamssIspOpe::queueBuffers(Request *request, FrameBuffer *inputBuffer)
{
	ipa_->queueRequest(request->sequence(), request->controls());
	/* Calculate stats for the whole frame */
	stats_->processFrame(request->sequence(), 0, inputBuffer);

	FrameBuffer *outputBuffer = nullptr;
	std::map<const Stream *, FrameBuffer *> outputs;
	for (const auto &[stream, outbuffer] : request->buffers()) {
		if (stream == &outStream_)
			outputBuffer = outbuffer;
	}

	if (!outputBuffer) {
		LOG(Camss, Error) << "Request does not contain OPE output buffer";
		return;
	}

	output_->queueBuffer(outputBuffer);
	input_->queueBuffer(inputBuffer);
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

	ret = input_->importBuffers(inputBufferCount_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(outputBufferCount_);
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = input_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = output_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	return 0;
}

void CamssIspOpe::stop()
{
	output_->streamOff();
	input_->streamOff();
	output_->releaseBuffers();
	input_->releaseBuffers();
	ipa_->stop();
}

std::unique_ptr<CamssIspOpe> CamssIspOpe::match(PipelineHandler *pipe,
						DeviceEnumerator *enumerator,
						const CameraSensor *sensor,
						ControlInfoMap *ispControls)
{
	std::unique_ptr<CamssIspOpe> ope = std::make_unique<CamssIspOpe>(pipe, sensor, ispControls);
	MediaEntity *paramsEnt, *inputEnt, *outputEnt;
	int ret;

	DeviceMatch opeDm("qcom-camss-ope");

	opeDm.add("params");
	opeDm.add("frame-input");
	opeDm.add("frame-output");

	ope->opeMediaDev_ = pipe->acquireMediaDevice(enumerator, opeDm);
	if (!ope->opeMediaDev_) {
		LOG(Camss, Info) << "No OPE match for " << sensor->entity()->name();
		return nullptr;
	}

	paramsEnt = ope->opeMediaDev_->getEntityByName("params");
	inputEnt = ope->opeMediaDev_->getEntityByName("frame-input");
	outputEnt = ope->opeMediaDev_->getEntityByName("frame-output");

	if (!paramsEnt || !inputEnt || !outputEnt) {
		LOG(Camss, Error) << "Did not find expected entities";
		return nullptr;
	}

	ope->params_ = std::make_unique<V4L2VideoDevice>(paramsEnt);
	ret = ope->params_->open();
	if (ret)
		return nullptr;

	ope->input_ = std::make_unique<V4L2VideoDevice>(inputEnt);
	ret = ope->input_->open();
	if (ret)
		return nullptr;

	ope->output_ = std::make_unique<V4L2VideoDevice>(outputEnt);
	ret = ope->output_->open();
	if (ret)
		return nullptr;

	LOG(Camss, Info) << "Using OPE for " << sensor->entity()->name();

	return ope;
}

} /* namespace libcamera */
