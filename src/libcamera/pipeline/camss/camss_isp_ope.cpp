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

#include "camss_frames.h"
#include "camss_params.h"
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
 * \param[in] opeMediaDev The OPE MediaDevice
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
CamssIspOpe::CamssIspOpe(PipelineHandler *pipe, std::shared_ptr<MediaDevice> opeMediaDev,
			 const CameraSensor *sensor, CamssFrames *frameInfos,
			 ControlInfoMap *ispControls)
	: opeMediaDev_(std::move(opeMediaDev)), sensor_(sensor), frameInfos_(frameInfos)
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

	stats_->statsReady.connect(this, &CamssIspOpe::statsReady);

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

	ipa_->paramsComputed.connect(this, &CamssIspOpe::paramsComputed);
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

int CamssIspOpe::init()
{
	MediaEntity *paramsEnt, *inputEnt, *outputEnt;
	int ret;

	if (!isValid())
		return -ENOMEM;

	paramsEnt = opeMediaDev_->getEntityByName("params");
	inputEnt = opeMediaDev_->getEntityByName("frame-input");
	outputEnt = opeMediaDev_->getEntityByName("frame-output");

	if (!paramsEnt || !inputEnt || !outputEnt) {
		LOG(Camss, Error) << "Did not find expected entities";
		return -EINVAL;
	}

	params_ = std::make_unique<V4L2VideoDevice>(paramsEnt);
	ret = params_->open();
	if (ret)
		return ret;

	params_->bufferReady.connect(this, &CamssIspOpe::paramsBufferReady);

	input_ = std::make_unique<V4L2VideoDevice>(inputEnt);
	ret = input_->open();
	if (ret)
		return ret;

	input_->bufferReady.connect(this, [&](FrameBuffer *f) {
		inputBufferReady.emit(f);
	});

	output_ = std::make_unique<V4L2VideoDevice>(outputEnt);
	ret = output_->open();
	if (ret)
		return ret;

	output_->bufferReady.connect(this, [&](FrameBuffer *f) {
		outputBufferReady.emit(f);
	});

	return 0;
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

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	return 0;
}

int CamssIspOpe::allocateBuffers([[maybe_unused]] unsigned int bufferCount)
{
	int ret;

	ret = input_->importBuffers(inputBufferCount_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(outputBufferCount_);
	if (ret < 0) {
		freeBuffers();
		return ret;
	}

	ret = params_->allocateBuffers(bufferCount, &paramBuffers_);
	if (ret < 0) {
		freeBuffers();
		return ret;
	}

	/* Map buffers to the IPA. */
	unsigned int ipaBufferId = 1;

	/* Lambda function for future re-use with statsBuffers. */
	auto pushBuffers = [&](const std::vector<std::unique_ptr<FrameBuffer>> &buffers) {
		for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
			Span<const FrameBuffer::Plane> planes = buffer->planes();

			buffer->setCookie(ipaBufferId++);
			ipaBuffers_.emplace_back(buffer->cookie(),
						 std::vector<FrameBuffer::Plane>{ planes.begin(),
										  planes.end() });
		}
	};

	pushBuffers(paramBuffers_);

	/* FIXME drop once using own camss IPA, which will map on IPA side */
	for (const IPABuffer &buffer : ipaBuffers_) {
		const FrameBuffer fb(buffer.planes);
		mappedIpaBuffers_.emplace(buffer.id,
					  MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}

	frameInfos_->init(paramBuffers_, {}, true);
	return 0;
}

void CamssIspOpe::freeBuffers()
{
	frameInfos_->clear();

	/* FIXME drop once using own camss IPA, which will map on IPA side */
	mappedIpaBuffers_.clear();

	ipaBuffers_.clear();
	paramBuffers_.clear();
	params_->releaseBuffers();
	output_->releaseBuffers();
	input_->releaseBuffers();
}

int CamssIspOpe::exportOutputBuffers([[maybe_unused]] const Stream *stream, unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

void CamssIspOpe::queueBuffers(Request *request, FrameBuffer *inputBuffer)
{
	uint32_t frame = request->sequence();

	ipa_->queueRequest(frame, request->controls());
	/* Calculate stats for the whole frame. */
	stats_->processFrame(frame, 0, inputBuffer);
	ipa_->computeParams(frame);
	/* paramsComputed() will queue all the buffers once the params are known. */
}

void CamssIspOpe::paramsComputed(uint32_t frame)
{
	CamssFrames::Info *info = frameInfos_->find(frame);
	if (!info)
		return;

	FrameBuffer *outputBuffer = nullptr;
	for (const auto &[stream, buffer] : info->request->buffers()) {
		if (stream == &outStream_)
			outputBuffer = buffer;
	}
	if (!outputBuffer) {
		LOG(Camss, Error) << "Missing output buffer";
		return;
	}

	/*
	 * FIXME drop once using own camss IPA, which will do this on IPA side.
	 * IPA paramsComputed signal should also pass a bytesused value back.
	 */
	uint32_t bufferId = info->paramBuffer->cookie();
	ipa::camss::CamssParams params(mappedIpaBuffers_.at(bufferId).planes()[0]);
	auto awbGains = params.block<ipa::camss::CamssBlocks::AwbGains>();
	DebayerParams debayerParams = *sharedParams_;
	/* Convert to 15uQ10 and store */
	awbGains->r_gain = std::clamp(debayerParams.gains.r(), 0.0f, 31.0f) * 1024;
	awbGains->g_gain = std::clamp(debayerParams.gains.g(), 0.0f, 31.0f) * 1024;
	awbGains->b_gain = std::clamp(debayerParams.gains.b(), 0.0f, 31.0f) * 1024;

	info->paramBuffer->_d()->metadata().planes()[0].bytesused = params.bytesused();
	params_->queueBuffer(info->paramBuffer);
	output_->queueBuffer(outputBuffer);
	input_->queueBuffer(info->rawBuffer);
}

void CamssIspOpe::paramsBufferReady(FrameBuffer *f)
{
	CamssFrames::Info *info = frameInfos_->find(f);
	if (!info)
		return;

	info->paramDequeued = true;
}

void CamssIspOpe::statsReady(uint32_t frame, uint32_t statsBufferId)
{
	CamssFrames::Info *info = frameInfos_->find(frame);
	if (!info)
		return;

	ipa_->processStats(frame, statsBufferId, info->effectiveSensorControls);
}

int CamssIspOpe::start()
{
	int ret = ipa_->start();
	if (ret)
		return ret;

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

	ret = params_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	return 0;
}

void CamssIspOpe::stop()
{
	params_->streamOff();
	output_->streamOff();
	input_->streamOff();
	ipa_->stop();
}

/**
 * \brief Match media devices and create OPE instance if found
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
std::unique_ptr<CamssIspOpe> CamssIspOpe::match(PipelineHandler *pipe,
						DeviceEnumerator *enumerator,
						const CameraSensor *sensor,
						CamssFrames *frameInfos,
						ControlInfoMap *ispControls)
{
	std::shared_ptr<MediaDevice> opeMediaDev;
	DeviceMatch opeDm("qcom-camss-ope");

	opeDm.add("params");
	opeDm.add("frame-input");
	opeDm.add("frame-output");

	opeMediaDev = pipe->acquireMediaDevice(enumerator, opeDm);
	if (!opeMediaDev) {
		LOG(Camss, Info) << "No OPE match for " << sensor->entity()->name();
		return nullptr;
	}

	std::unique_ptr<CamssIspOpe> ope =
		std::make_unique<CamssIspOpe>(pipe, std::move(opeMediaDev), sensor,
					      frameInfos, ispControls);

	if (ope->init() != 0)
		return nullptr;

	LOG(Camss, Info) << "Using OPE for " << sensor->entity()->name();

	return ope;
}

} /* namespace libcamera */
