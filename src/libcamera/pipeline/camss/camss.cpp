/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Pipeline handler for Qualcomm CAMSS
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Partially based on other pipeline-handlers which are:
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 * Copyright (C) 2019, Google Inc.
 */

#include <algorithm>
#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"

#include "camss_csi.h"
#include "camss_frames.h"
#include "camss_isp.h"
#include "camss_isp_soft.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Camss)

class CamssCameraData : public Camera::Private
{
public:
	CamssCameraData(PipelineHandler *pipe);

	void csiBufferReady(FrameBuffer *buffer);
	void ispOutputBufferReady(FrameBuffer *buffer);
	void ispParameterBufferReady(FrameBuffer *buffer);
	void frameStart(uint32_t sequence);
	void statsReady(uint32_t frame, uint32_t statsBufferId);
	void metadataReady(unsigned int id, const ControlList &metadata);
	void setSensorControls(const ControlList &sensorControls);

	void queuePendingRequests();
	void cancelPendingRequests();

	std::unique_ptr<CamssCsiCamera> csi_;
	std::unique_ptr<CamssIsp> isp_;
	std::unique_ptr<DelayedControls> delayedCtrls_;
	CamssFrames frameInfos_;

	/* Requests for which no buffer has been queued to the CSI receiver yet. */
	std::queue<Request *> pendingRequests_;
};

class CamssCameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;
	static constexpr unsigned int kMaxStreams = 2;

	CamssCameraConfiguration(CamssCameraData *data);
	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;
	StreamConfiguration csiConfig_;
	StreamConfiguration ispConfig_;

private:
	/*
	 * The CamssCameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	const CamssCameraData *data_;
};

class PipelineHandlerCamss : public PipelineHandler
{
public:
	PipelineHandlerCamss(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	CamssCameraData *cameraData(Camera *camera)
	{
		return static_cast<CamssCameraData *>(camera->_d());
	}

	bool acquireDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;
	int allocateBuffers(Camera *camera);
	void freeBuffers(Camera *camera);

	CamssCsi csi_;
};

CamssCameraConfiguration::CamssCameraConfiguration(CamssCameraData *data)
	: CameraConfiguration()
{
	data_ = data;
}

CameraConfiguration::Status CamssCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/*
	 * Validate the requested transform against the sensor capabilities and
	 * rotation and store the final combined transform that configure() will
	 * need to apply to the sensor to save us working it out again.
	 */
	Orientation requestedOrientation = orientation;
	combinedTransform_ = data_->csi_->sensor()->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/* Max. 1 RAW + 1 processed stream is supported (for now). */
	StreamConfiguration rawConfig;
	StreamConfiguration processedConfig;
	unsigned int rawCount = 0;
	unsigned int processedCount = 0;

	for (const StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			if (rawCount) {
				LOG(Camss, Debug) << "Multiple raw streams not supported";
				return Invalid;
			}
			rawConfig = cfg;
			rawCount++;
		} else {
			if (processedCount) {
				LOG(Camss, Debug) << "Multiple processed streams not supported";
				return Invalid;
			}
			processedConfig = cfg;
			processedCount++;
		}
	}

	if (!processedCount) {
		/*
		 * \todo allow this, add dummyISP ISP class which only
		 * calls CPU stats on ready raw output buffers + runs the result
		 * through the softIPA to get sensor-control + metadata-info
		 */
		LOG(Camss, Debug)
			<< "Camera configuration cannot support raw-only streams";
		return Invalid;
	}

	if (!rawCount) {
		rawConfig.size = processedConfig.size;
		rawConfig.bufferCount = processedConfig.bufferCount;
	}

	csiConfig_ = data_->csi_->validate(rawConfig);
	if (!csiConfig_.pixelFormat.isValid())
		return Invalid;

	LOG(Camss, Debug) << "CSI configuration: " << csiConfig_.toString()
			  << " stride " << csiConfig_.stride
			  << " frameSize " << csiConfig_.frameSize;

	ispConfig_ = data_->isp_->validate(csiConfig_, processedConfig);
	if (!ispConfig_.pixelFormat.isValid())
		return Invalid;

	LOG(Camss, Debug) << "ISP configuration: " << ispConfig_.toString()
			  << " stride " << ispConfig_.stride
			  << " frameSize " << ispConfig_.frameSize;

	for (unsigned int i = 0; i < config_.size(); ++i) {
		const PixelFormatInfo &info = PixelFormatInfo::info(config_[i].pixelFormat);
		const StreamConfiguration *hwCfg, originalCfg = config_[i];
		Stream *stream;

		LOG(Camss, Debug) << "Validating stream: " << config_[i].toString();

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			hwCfg = &csiConfig_;
			stream = &data_->csi_->rawStream_;
		} else {
			hwCfg = &ispConfig_;
			stream = &data_->isp_->outStream_;
		}

		StreamConfiguration &cfg = config_[i];
		cfg.size = hwCfg->size;
		cfg.pixelFormat = hwCfg->pixelFormat;
		cfg.stride = hwCfg->stride;
		cfg.frameSize = hwCfg->frameSize;
		cfg.bufferCount = hwCfg->bufferCount;
		cfg.setStream(stream);

		if (cfg.pixelFormat != originalCfg.pixelFormat ||
		    cfg.size != originalCfg.size) {
			LOG(Camss, Debug)
				<< "Stream " << i << " configuration adjusted to "
				<< cfg.toString();
			status = Adjusted;
		}

		if (originalCfg.bufferCount && cfg.bufferCount != originalCfg.bufferCount) {
			LOG(Camss, Debug)
				<< "Adjusting bufferCount from " << originalCfg.bufferCount
				<< " to " << cfg.bufferCount;
			status = Adjusted;
		}

		/*
		 * \todo copy-pasted from src/libcamera/pipeline/simple/simple.cpp turn
		 * this into a generic helper?
		 * Best effort to fix the color space. If the color space is not set,
		 * set it according to the pixel format, which may not be correct (pixel
		 * formats and color spaces are different things, although somewhat
		 * related) but we don't have a better option at the moment. Then in any
		 * case, perform the standard pixel format based color space adjustment.
		 */
		if (!cfg.colorSpace) {
			const PixelFormatInfo &pfi = PixelFormatInfo::info(cfg.pixelFormat);
			switch (pfi.colourEncoding) {
			case PixelFormatInfo::ColourEncodingRGB:
				cfg.colorSpace = ColorSpace::Srgb;
				break;
			case PixelFormatInfo::ColourEncodingYUV:
				cfg.colorSpace = ColorSpace::Sycc;
				break;
			default:
				cfg.colorSpace = ColorSpace::Raw;
			}
			/*
			 * Adjust the assigned color space to make sure everything is OK.
			 * Since this is assigning an unspecified color space rather than
			 * adjusting a requested one, changes here shouldn't set the status
			 * to Adjusted.
			 */
			cfg.colorSpace->adjust(cfg.pixelFormat);
			LOG(Camss, Debug)
				<< "Unspecified color space set to "
				<< cfg.colorSpace.value().toString();
		} else {
			if (cfg.colorSpace->adjust(cfg.pixelFormat)) {
				LOG(Camss, Debug)
					<< "Color space adjusted to "
					<< cfg.colorSpace.value().toString();
				status = Adjusted;
			}
		}
	}

	return status;
}

PipelineHandlerCamss::PipelineHandlerCamss(CameraManager *manager)
	: PipelineHandler(manager), csi_()
{
	lockOnAcquire_ = false;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerCamss::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	CamssCameraData *data = cameraData(camera);
	std::unique_ptr<CamssCameraConfiguration> config =
		std::make_unique<CamssCameraConfiguration>(data);
	StreamConfiguration cfg, csiConfig;

	if (roles.empty())
		return config;

	csiConfig = data->csi_->generateConfiguration();
	if (!csiConfig.pixelFormat.isValid())
		return nullptr;

	LOG(Camss, Debug) << "Generated CSI cfg " << csiConfig;

	bool processedRequested = false;
	bool rawRequested = false;
	for (const auto &role : roles) {
		if (role == StreamRole::Raw)
			rawRequested = true;
		else
			processedRequested = true;
	}

	if (rawRequested)
		config->addConfiguration(csiConfig);

	if (processedRequested) {
		cfg = data->isp_->generateConfiguration(csiConfig);
		if (!cfg.pixelFormat.isValid())
			return nullptr;

		LOG(Camss, Debug) << "Generated ISP cfg " << cfg;
		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return nullptr;

	return config;
}

int PipelineHandlerCamss::configure(Camera *camera, CameraConfiguration *c)
{
	CamssCameraConfiguration *config =
		static_cast<CamssCameraConfiguration *>(c);
	CamssCameraData *data = cameraData(camera);
	int ret;

	ret = data->csi_->configure(config->csiConfig_, config->combinedTransform_);
	if (ret)
		return ret;

	return data->isp_->configure(config->csiConfig_, config->ispConfig_);
}

int PipelineHandlerCamss::exportFrameBuffers(Camera *camera, Stream *stream,
					     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	CamssCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->csi_->rawStream_)
		return data->csi_->exportBuffers(count, buffers);
	else if (stream == &data->isp_->outStream_)
		return data->isp_->exportOutputBuffers(stream, count, buffers);

	return -EINVAL;
}

int PipelineHandlerCamss::allocateBuffers(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);
	unsigned int bufferCount;

	bufferCount = std::max({
		data->csi_->rawStream_.configuration().bufferCount,
		data->isp_->outStream_.configuration().bufferCount,
	});

	return data->isp_->allocateBuffers(bufferCount);
}

void PipelineHandlerCamss::freeBuffers(Camera *camera)
{
	cameraData(camera)->isp_->freeBuffers();
}

bool PipelineHandlerCamss::acquireDevice(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);

	return data->csi_->acquireDevice();
}

void PipelineHandlerCamss::releaseDevice(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);

	data->csi_->releaseDevice();
}

int PipelineHandlerCamss::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	CamssCameraData *data = cameraData(camera);
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	data->delayedCtrls_->reset();

	ret = data->csi_->start();
	if (ret)
		goto freebuffers;

	ret = data->isp_->start();
	if (ret)
		goto stop;

	return 0;

stop:
	data->csi_->stop();
freebuffers:
	freeBuffers(camera);

	LOG(Camss, Error) << "Failed to start camera " << camera->id();
	return ret;
}

void PipelineHandlerCamss::stopDevice(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);

	data->cancelPendingRequests();

	data->isp_->stop();
	data->csi_->stop();

	freeBuffers(camera);
}

CamssCameraData::CamssCameraData(PipelineHandler *pipe)
	: Camera::Private(pipe)
{
	frameInfos_.bufferAvailable.connect(this, &CamssCameraData::queuePendingRequests);
}

void CamssCameraData::cancelPendingRequests()
{
	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		for (const auto &[stream, buffer] : request->buffers()) {
			buffer->_d()->cancel();
			pipe()->completeBuffer(request, buffer);
		}

		pipe()->completeRequest(request);
		pendingRequests_.pop();
	}
}

void CamssCameraData::queuePendingRequests()
{
	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		CamssFrames::Info *info = frameInfos_.create(request);
		if (!info)
			break;

		/*
		 * Queue a buffer on the CSI, using the raw stream buffer
		 * provided in the request, if any, or a CIO2 internal buffer
		 * otherwise.
		 */
		FrameBuffer *reqRawBuffer = request->findBuffer(&csi_->rawStream_);
		FrameBuffer *rawBuffer = csi_->queueBuffer(request, reqRawBuffer);
		/*
		 * \todo If queueBuffer fails in queuing a buffer to the device,
		 * report the request as error by cancelling the request and
		 * calling PipelineHandler::completeRequest().
		 */
		if (!rawBuffer) {
			frameInfos_.remove(info);
			break;
		}

		info->rawBuffer = rawBuffer;

		pendingRequests_.pop();
	}
}

int PipelineHandlerCamss::queueRequestDevice(Camera *camera, Request *request)
{
	CamssCameraData *data = cameraData(camera);

	data->pendingRequests_.push(request);
	data->queuePendingRequests();

	return 0;
}

bool PipelineHandlerCamss::match(DeviceEnumerator *enumerator)
{
	CamssCsi::Cameras csiCams;

	csiCams = csi_.match(this, enumerator);
	if (csiCams.empty())
		return false;

	unsigned int numCameras = 0;
	for (unsigned int i = 0; i < csiCams.size(); i++) {
		std::unique_ptr<CamssCameraData> data =
			std::make_unique<CamssCameraData>(this);
		data->csi_ = std::move(csiCams[i]);

		data->csi_->frameStart().connect(data.get(),
						 &CamssCameraData::frameStart);
		data->csi_->bufferReady().connect(data.get(),
						  &CamssCameraData::csiBufferReady);
		data->csi_->bufferAvailable.connect(data.get(),
						    &CamssCameraData::queuePendingRequests);

		CameraSensor *sensor = data->csi_->sensor();

		/* Initialize the camera properties. */
		data->properties_ = sensor->properties();

		const CameraSensorProperties::SensorDelays &delays = sensor->sensorDelays();
		std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
			{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
			{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		};

		data->delayedCtrls_ =
			std::make_unique<DelayedControls>(sensor->device(), params);

		data->isp_ = std::make_unique<CamssIspSoft>(this, sensor,
							    &data->frameInfos_,
							    &data->controlInfo_);
		if (!data->isp_->isValid()) {
			LOG(Camss, Error) << "Failed to create software ISP";
			continue;
		}

		data->isp_->inputBufferReady.connect(data->csi_.get(),
						     &CamssCsiCamera::tryReturnBuffer);
		data->isp_->outputBufferReady.connect(data.get(),
						      &CamssCameraData::ispOutputBufferReady);
		data->isp_->paramBufferReady.connect(data.get(),
						     &CamssCameraData::ispParameterBufferReady);
		data->isp_->statsReady.connect(data.get(), &CamssCameraData::statsReady);
		data->isp_->metadataReady.connect(data.get(), &CamssCameraData::metadataReady);
		data->isp_->setSensorControls.connect(data.get(), &CamssCameraData::setSensorControls);

		/* Create and register the Camera instance. */
		std::set<Stream *> streams = {
			&data->isp_->outStream_,
			&data->csi_->rawStream_,
		};
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), sensor->id(), streams);

		registerCamera(std::move(camera));
		numCameras++;
	}

	return numCameras != 0;
}

void CamssCameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);

	/*
	 * Directly apply controls now if there is no frameStart signal.
	 *
	 * \todo Applying controls directly not only increases the risk of
	 * applying them to the wrong frame (or across a frame boundary),
	 * but it also bypasses delayedCtrls_, creating AGC regulation issues.
	 * Both problems should be fixed.
	 */
	if (!csi_->supportsFrameStart()) {
		ControlList ctrls(sensorControls);
		csi_->sensor()->setControls(&ctrls);
	}
}

void CamssCameraData::metadataReady(unsigned int id, const ControlList &metadata)
{
	CamssFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	/* tryComplete(info) will free info on success! */
	Request *request = info->request;
	request->_d()->metadata().merge(metadata);

	info->metadataProcessed = true;
	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle buffers completion at the ISP output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the ISP output are directed to the application.
 */
void CamssCameraData::ispOutputBufferReady(FrameBuffer *buffer)
{
	CamssFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	/* tryComplete(info) will free info on success! */
	Request *request = info->request;

	pipe()->completeBuffer(request, buffer);
	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
}

void CamssCameraData::ispParameterBufferReady(FrameBuffer *buffer)
{
	CamssFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	/* tryComplete(info) will free info on success! */
	Request *request = info->request;

	/* Parameter buffer is internal, no pipe()->completeBuffer() */
	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
}

/**
 * \brief Handle buffers completion at the CSI-receiver output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the CSI-receiver are immediately queued to the ISP
 * for further processing.
 */
void CamssCameraData::csiBufferReady(FrameBuffer *buffer)
{
	CamssFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	/* On error / cancellation force a complete of the whole request. */
	if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
		for (const auto &[stream, b] : request->buffers()) {
			b->_d()->cancel();
			pipe()->completeBuffer(request, b);
		}

		frameInfos_.remove(info);
		pipe()->cancelRequest(request);
		return;
	}

	/*
	 * Record the sensor's timestamp in the request metadata.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal.
	 */
	request->_d()->metadata().set(controls::SensorTimestamp,
				      buffer->metadata().timestamp);

	if (request->findBuffer(&csi_->rawStream_))
		pipe()->completeBuffer(request, buffer);

	isp_->queueBuffers(request, buffer);
}

/*
 * \brief Handle the start of frame exposure signal
 * \param[in] sequence The sequence number of frame
 */
void CamssCameraData::frameStart(uint32_t sequence)
{
	delayedCtrls_->applyControls(sequence);
}

void CamssCameraData::statsReady(uint32_t frame, uint32_t statsBufferId)
{
	isp_->processStats(frame, statsBufferId, delayedCtrls_->get(frame));
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerCamss, "camss")

} /* namespace libcamera */
