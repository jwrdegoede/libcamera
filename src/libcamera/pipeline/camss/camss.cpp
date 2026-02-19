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
//#include "camss_isp.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Camss)

class CamssCameraData : public Camera::Private
{
public:
	CamssCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	void csiBufferReady(FrameBuffer *buffer);
	void ispOutputBufferReady(FrameBuffer *buffer);
	/* This is unused atm, this is here for future parameterbuf support */
	void paramBufferReady(FrameBuffer *buffer);
	void queuePendingRequests();
	void cancelPendingRequests();
	void frameStart(uint32_t sequence);

	std::unique_ptr<CamssCsiCamera> csi_;
//	std::unique_ptr<CamssIsp> isp_;
	std::unique_ptr<DelayedControls> delayedCtrls_;
	Stream outStream_;
	Stream rawStream_;
	CamssFrames frameInfos_;

	/* Requests for which no buffer has been queued to the CSI receiver yet. */
	std::queue<Request *> pendingRequests_;
	/* Requests queued to the CSI receiver but not yet processed by the ISP. */
	std::queue<Request *> processingRequests_;

private:
	void metadataReady(unsigned int id, const ControlList &metadata);
//	void paramsComputed(unsigned int id);
	void setSensorControls(const ControlList &sensorControls);
};

class CamssCameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;
	static constexpr unsigned int kMaxStreams = 2;

	CamssCameraConfiguration(CamssCameraData *data);

#if 0
	Status validate() override;

	const StreamConfiguration &csiFormat() const { return csiConfiguration_; }

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;
#endif

private:
	/*
	 * The CamssCameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	const CamssCameraData *data_;

//	StreamConfiguration csiConfiguration_;
//	ImgUDevice::PipeConfig pipeConfig_;
};

class PipelineHandlerCamss : public PipelineHandler
{
public:
	PipelineHandlerCamss(CameraManager *manager);

#if 0
	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

#endif
	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	CamssCameraData *cameraData(Camera *camera)
	{
		return static_cast<CamssCameraData *>(camera->_d());
	}

#if 0
	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	
	ImgUDevice isp0_;
	ImgUDevice isp1_;
	std::shared_ptr<MediaDevice> csiMediaDev_;
	std::shared_ptr<MediaDevice> ispMediaDev_;

	std::vector<IPABuffer> ipaBuffers_;
#endif
	CamssCsi csi_;
};

CamssCameraConfiguration::CamssCameraConfiguration(CamssCameraData *data)
	: CameraConfiguration()
{
	data_ = data;
}

#if 0
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
	combinedTransform_ = data_->csi_.sensor()->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > kMaxStreams) {
		config_.resize(kMaxStreams);
		status = Adjusted;
	}

	/*
	 * Validate the requested stream configuration and select the sensor
	 * format by collecting the maximum RAW stream width and height and
	 * picking the closest larger match.
	 *
	 * If no RAW stream is requested use the one of the largest YUV stream,
	 * plus margin pixels for the IF and BDS rectangle to downscale.
	 *
	 * \todo Clarify the IF and BDS margins requirements.
	 */
	unsigned int rawCount = 0;
	unsigned int yuvCount = 0;
	Size rawRequirement;
	Size maxYuvSize;
	Size rawSize;

	for (const StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			rawCount++;
			// FIXME no need for max, should be only 1 raw stream
			rawSize = std::max(rawSize, cfg.size);
			// FIXME save raw format, convert to mbus-code, pass to
			// csi_.generateConfiguration() / getSensorFormat()
			// as preferred mbus-code to allow selecting between e.g.
			// 8 bpp vs 10 bpp raw bayer on imx219
		} else {
			yuvCount++;
			maxYuvSize = std::max(maxYuvSize, cfg.size);
			rawRequirement.expandTo(cfg.size);
		}
	}

	if (rawCount > 1 || yuvCount > 2) {
		LOG(Camss, Debug) << "Camera configuration not supported";
		return Invalid;
	} else if (rawCount && !yuvCount) {
		// FIXME allow this, add dummyISP ISP class which only
		// calls CPU stats on ready raw output buffers + runs the result
		// through the softIPA to get sensor-control + metadatainfo
		// info
		 *
		 * \todo Configure the ImgU with internal buffers which will enable
		 * the IPA to get configured for the raw-only camera configuration.
		 */
		LOG(Camss, Debug)
			<< "Camera configuration cannot support raw-only streams";
		return Invalid;
	}

	/*
	 * Generate raw configuration from CIO2.
	 *
	 * The output YUV streams will be limited in size to the maximum frame
	 * size requested for the RAW stream, if present.
	 *
	 * If no raw stream is requested, generate a size from the largest YUV
	 * stream, aligned to the ImgU constraints and bound
	 * by the sensor's maximum resolution. See
	 * https://bugs.libcamera.org/show_bug.cgi?id=32
	 */
	if (rawSize.isNull())
		rawSize = rawRequirement.expandedTo({ ImgUDevice::kIFMaxCropWidth,
						      ImgUDevice::kIFMaxCropHeight })
				  .grownBy({ ImgUDevice::kOutputMarginWidth,
					     ImgUDevice::kOutputMarginHeight })
				  .boundedTo(data_->csi_.sensor()->resolution());

	csiConfiguration_ = data_->csi_.generateConfiguration(rawSize);
	if (!csiConfiguration_.pixelFormat.isValid())
		return Invalid;

	LOG(Camss, Debug) << "CIO2 configuration: " << csiConfiguration_.toString();

	ImgUDevice::Pipe pipe{};
	pipe.input = csiConfiguration_.size;

	/*
	 * Adjust the configurations if needed and assign streams while
	 * iterating them.
	 */
	bool mainOutputAvailable = true;
	for (unsigned int i = 0; i < config_.size(); ++i) {
		const PixelFormatInfo &info = PixelFormatInfo::info(config_[i].pixelFormat);
		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		LOG(Camss, Debug) << "Validating stream: " << config_[i].toString();

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			/* Initialize the RAW stream with the CIO2 configuration. */
			cfg->size = csiConfiguration_.size;
			cfg->pixelFormat = csiConfiguration_.pixelFormat;
			cfg->bufferCount = csiConfiguration_.bufferCount;
			cfg->stride = info.stride(cfg->size.width, 0, 64);
			cfg->frameSize = info.frameSize(cfg->size, 64);
			cfg->setStream(const_cast<Stream *>(&data_->rawStream_));

			LOG(Camss, Debug) << "Assigned " << cfg->toString()
					 << " to the raw stream";
		} else {
			/* Assign and configure the main and viewfinder outputs. */

			/*
			 * Clamp the size to match the ImgU size limits and the
			 * margins from the CIO2 output frame size.
			 *
			 * The ImgU outputs needs to be strictly smaller than
			 * the CIO2 output frame and rounded down to 64 pixels
			 * in width and 32 pixels in height. This assumption
			 * comes from inspecting the pipe configuration script
			 * results and the available suggested configurations in
			 * the ChromeOS BSP .xml camera tuning files and shall
			 * be validated.
			 *
			 * \todo Clarify what are the hardware constraints
			 * that require this alignements, if any. It might
			 * depend on the BDS scaling factor of 1/32, as the main
			 * output has no YUV scaler as the viewfinder output has.
			 */
			unsigned int limit;
			limit = utils::alignDown(csiConfiguration_.size.width - 1,
						 ImgUDevice::kOutputMarginWidth);
			cfg->size.width = std::clamp(cfg->size.width,
						     ImgUDevice::kOutputMinSize.width,
						     limit);

			limit = utils::alignDown(csiConfiguration_.size.height - 1,
						 ImgUDevice::kOutputMarginHeight);
			cfg->size.height = std::clamp(cfg->size.height,
						      ImgUDevice::kOutputMinSize.height,
						      limit);

			cfg->size.alignDownTo(ImgUDevice::kOutputAlignWidth,
					      ImgUDevice::kOutputAlignHeight);

			cfg->pixelFormat = formats::NV12;
			cfg->bufferCount = kBufferCount;
			cfg->stride = info.stride(cfg->size.width, 0, 1);
			cfg->frameSize = info.frameSize(cfg->size, 1);

			/*
			 * Use the main output stream in case only one stream is
			 * requested or if the current configuration is the one
			 * with the maximum YUV output size.
			 */
			if (mainOutputAvailable &&
			    (originalCfg.size == maxYuvSize || yuvCount == 1)) {
				cfg->setStream(const_cast<Stream *>(&data_->outStream_));
				mainOutputAvailable = false;

				pipe.main = cfg->size;
				if (yuvCount == 1)
					pipe.viewfinder = pipe.main;

				LOG(Camss, Debug) << "Assigned " << cfg->toString()
						 << " to the main output";
			} else {
				cfg->setStream(const_cast<Stream *>(&data_->vfStream_));
				pipe.viewfinder = cfg->size;

				LOG(Camss, Debug) << "Assigned " << cfg->toString()
						 << " to the viewfinder output";
			}
		}

		if (cfg->pixelFormat != originalCfg.pixelFormat ||
		    cfg->size != originalCfg.size) {
			LOG(Camss, Debug)
				<< "Stream " << i << " configuration adjusted to "
				<< cfg->toString();
			status = Adjusted;
		}
	}

	/* Only compute the ImgU configuration if a YUV stream has been requested. */
	if (yuvCount) {
		pipeConfig_ = data_->isp_->calculatePipeConfig(&pipe);
		if (pipeConfig_.isNull()) {
			LOG(Camss, Error) << "Failed to calculate pipe configuration: "
					 << "unsupported resolutions.";
			return Invalid;
		}
	}

	return status;
}
#endif

PipelineHandlerCamss::PipelineHandlerCamss(CameraManager *manager)
	: PipelineHandler(manager), csi_()
{
}

#if 0
std::unique_ptr<CameraConfiguration>
PipelineHandlerCamss::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	CamssCameraData *data = cameraData(camera);
	std::unique_ptr<CamssCameraConfiguration> config =
		std::make_unique<CamssCameraConfiguration>(data);

	if (roles.empty())
		return config;

	bool processedRequested = false;
	bool rawRequested = false;
	for (const auto &role : roles) {
		if (role == StreamRole::Raw)
			rawRequested = true;
		else
			processedRequested = true;
	}

	if (rawRequested) {
		// FIXME move this to a data->csi_->generateConfiguration() call:
		// data->csi_->generateConfiguration(config.get());
		std::map<PixelFormat, std::vector<SizeRange>> formats;
		for (unsigned int code : data->sensor()->mbusCodes()) {
			PixelFormat pixelFormat = data->csi_->mbusCodeToPixelFormat(code);
			if (!pixelFormat)
				continue;

			std::vector<SizeRange> sizes;
			for (const Size &sz : sensor_->sizes(code))
				sizes.emplace_back(sz);

			formats[pixelFormat] = sizes;
		}

		V4L2SubdeviceFormat sensorFormat = data->csi_->getSensorFormat(data->sensor_->resolution());
		StreamConfiguration cfg{ StreamFormats{ formats } };
		cfg.size = sensorFormat.size;
		cfg.pixelFormat = data->csi->mbusCodeToPixelFormat(sensorFormat.code);

		config->addConfiguration(cfg);
	}

	if (processedRequested) {
#if 0
		// FIXME this sbould be the problem of the ISP class, just call:
		// data->isp_->generateConfiguration(data->csi_, config.get());
		if (data->isp_->canDownscale()) {
			/*
			 * Use as default full-frame configuration a value
			 * strictly smaller than the sensor resolution (limited
			 * to the ImgU  maximum output size) and aligned down to
			 * the required frame margin.
			 *
			 * \todo Clarify the alignment constraints as explained
			 * in validate()
			 */
			size = sensorResolution.boundedTo(ImgUDevice::kOutputMaxSize)
					       .shrunkBy({ 1, 1 })
					       .alignedDownTo(ImgUDevice::kOutputMarginWidth,
							      ImgUDevice::kOutputMarginHeight);
			pixelFormat = formats::NV12;
			bufferCount = CamssCameraConfiguration::kBufferCount;
			streamFormats[pixelFormat] = { { ImgUDevice::kOutputMinSize, size } };
			StreamFormats formats(streamFormats);
			StreamConfiguration cfg(formats);
			cfg.size = size;
			cfg.pixelFormat = pixelFormat;
			cfg.bufferCount = bufferCount;
			config->addConfiguration(cfg);
		} else {
			// TODO iterate over sensor sizes and add them to streamFormats[pixelFormat]
			// 1 by one, call into isp to do this for us, so it can take
			// any omitted border or some such into effect
		}		
#endif
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerCamss::configure(Camera *camera, CameraConfiguration *c)
{
	CamssCameraConfiguration *config =
		static_cast<CamssCameraConfiguration *>(c);
	CamssCameraData *data = cameraData(camera);
	Stream *outStream = &data->outStream_;
	Stream *vfStream = &data->vfStream_;
	CIO2Device *csi = &data->csi_;
	ImgUDevice *isp = data->isp_;
	V4L2DeviceFormat outputFormat;
	int ret;

	/*
	 * FIXME: enabled links in one ImgU pipe interfere with capture
	 * operations on the other one. This can be easily triggered by
	 * capturing from one camera and then trying to capture from the other
	 * one right after, without disabling media links on the first used
	 * pipe.
	 *
	 * The tricky part here is where to disable links on the ImgU instance
	 * which is currently not in use:
	 * 1) Link enable/disable cannot be done at start()/stop() time as video
	 * devices needs to be linked first before format can be configured on
	 * them.
	 * 2) As link enable has to be done at the least in configure(),
	 * before configuring formats, the only place where to disable links
	 * would be 'stop()', but the Camera class state machine allows
	 * start()<->stop() sequences without any configure() in between.
	 *
	 * As of now, disable all links in the ImgU media graph before
	 * configuring the device, to allow alternate the usage of the two
	 * ImgU pipes.
	 *
	 * As a consequence, a Camera using an ImgU shall be configured before
	 * any start()/stop() sequence. An application that wants to
	 * pre-configure all the camera and then start/stop them alternatively
	 * without going through any re-configuration (a sequence that is
	 * allowed by the Camera state machine) would now fail on the Camss.
	 */
	ret = ispMediaDev_->disableLinks();
	if (ret)
		return ret;

	/*
	 * \todo Enable links selectively based on the requested streams.
	 * As of now, enable all links unconditionally.
	 * \todo Don't configure the ImgU at all if we only have a single
	 * stream which is for raw capture, in which case no buffers will
	 * ever be queued to the ImgU.
	 */
	ret = data->isp_->enableLinks(true);
	if (ret)
		return ret;

	/*
	 * Pass the requested stream size to the CIO2 unit and get back the
	 * adjusted format to be propagated to the ImgU output devices.
	 */
	const Size &sensorSize = config->csiFormat().size;
	V4L2DeviceFormat csiFormat;
	ret = csi->configure(sensorSize, config->combinedTransform_, &csiFormat);
	if (ret)
		return ret;

	IPACameraSensorInfo sensorInfo;
	csi->sensor()->sensorInfo(&sensorInfo);
	data->cropRegion_ = sensorInfo.analogCrop;

	/*
	 * If the ImgU gets configured, its driver seems to expect that
	 * buffers will be queued to its outputs, as otherwise the next
	 * capture session that uses the ImgU fails when queueing
	 * buffers to its input.
	 *
	 * If no ImgU configuration has been computed, it means only a RAW
	 * stream has been requested: return here to skip the ImgU configuration
	 * part.
	 */
	ImgUDevice::PipeConfig ispConfig = config->ispConfig();
	if (ispConfig.isNull())
		return 0;

	ret = isp->configure(ispConfig, &csiFormat);
	if (ret)
		return ret;

	/* Apply the format to the configured streams output devices. */
	StreamConfiguration *mainCfg = nullptr;
	StreamConfiguration *vfCfg = nullptr;

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == outStream) {
			mainCfg = &cfg;
			ret = isp->configureOutput(cfg, &outputFormat);
			if (ret)
				return ret;
		} else if (stream == vfStream) {
			vfCfg = &cfg;
			ret = isp->configureViewfinder(cfg, &outputFormat);
			if (ret)
				return ret;
		}
	}

	/*
	 * As we need to set format also on the non-active streams, use
	 * the configuration of the active one for that purpose (there should
	 * be at least one active stream in the configuration request).
	 */
	if (!vfCfg) {
		ret = isp->configureViewfinder(*mainCfg, &outputFormat);
		if (ret)
			return ret;
	}

	/* Apply the "pipe_mode" control to the ImgU subdevice. */
	ControlList ctrls(isp->isp_->controls());
	/*
	 * Set the ImgU pipe mode to 'Video' unconditionally to have statistics
	 * generated.
	 *
	 * \todo Figure out what the 'Still Capture' mode is meant for, and use
	 * it accordingly.
	 */
	ctrls.set(V4L2_CID_Camss_PIPE_MODE,
		  static_cast<int32_t>(CamssPipeModeVideo));
	ret = isp->isp_->setControls(&ctrls);
	if (ret) {
		LOG(Camss, Error) << "Unable to set pipe_mode control";
		return ret;
	}

	ipa::ipu3::IPAConfigInfo configInfo;
	configInfo.sensorControls = data->csi_.sensor()->controls();

	CameraLens *lens = data->csi_.sensor()->focusLens();
	if (lens)
		configInfo.lensControls = lens->controls();

	configInfo.sensorInfo = sensorInfo;
	configInfo.bdsOutputSize = config->ispConfig().bds;
	configInfo.iif = config->ispConfig().iif;

	ret = data->ipa_->configure(configInfo, &data->ipaControls_);
	if (ret) {
		LOG(Camss, Error) << "Failed to configure IPA: "
				 << strerror(-ret);
		return ret;
	}

	return updateControls(data);
}

int PipelineHandlerCamss::exportFrameBuffers(Camera *camera, Stream *stream,
					    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	CamssCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->outStream_)
		return data->isp_->output_->exportBuffers(count, buffers);
	else if (stream == &data->vfStream_)
		return data->isp_->viewfinder_->exportBuffers(count, buffers);
	else if (stream == &data->rawStream_)
		return data->csi_.exportBuffers(count, buffers);

	return -EINVAL;
}

/**
 * \todo Clarify if 'viewfinder' and 'stat' nodes have to be set up and
 * started even if not in use. As of now, if not properly configured and
 * enabled, the ImgU processing pipeline stalls.
 *
 * In order to be able to start the 'viewfinder' and 'stat' nodes, we need
 * memory to be reserved.
 */
int PipelineHandlerCamss::allocateBuffers(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);
	ImgUDevice *isp = data->isp_;
	unsigned int bufferCount;
	int ret;

	bufferCount = std::max({
		data->outStream_.configuration().bufferCount,
		data->vfStream_.configuration().bufferCount,
		data->rawStream_.configuration().bufferCount,
	});

	ret = isp->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	/* Map buffers to the IPA. */
	unsigned int ipaBufferId = 1;

	auto pushBuffers = [&](const std::vector<std::unique_ptr<FrameBuffer>> &buffers) {
		for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
			Span<const FrameBuffer::Plane> planes = buffer->planes();

			buffer->setCookie(ipaBufferId++);
			ipaBuffers_.emplace_back(buffer->cookie(),
						 std::vector<FrameBuffer::Plane>{ planes.begin(),
										  planes.end() });
		}
	};

	pushBuffers(isp->paramBuffers_);
	pushBuffers(isp->statBuffers_);

	data->ipa_->mapBuffers(ipaBuffers_);

	data->frameInfos_.init(isp->paramBuffers_, isp->statBuffers_);
	data->frameInfos_.bufferAvailable.connect(
		data, &CamssCameraData::queuePendingRequests);

	return 0;
}

int PipelineHandlerCamss::freeBuffers(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);

	data->frameInfos_.clear();

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : ipaBuffers_)
		ids.push_back(ipabuf.id);

	data->ipa_->unmapBuffers(ids);
	ipaBuffers_.clear();

	data->isp_->freeBuffers();

	return 0;
}

int PipelineHandlerCamss::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	CamssCameraData *data = cameraData(camera);
	CIO2Device *csi = &data->csi_;
	ImgUDevice *isp = data->isp_;
	int ret;

	/* Disable test pattern mode on the sensor, if any. */
	ret = csi->sensor()->setTestPatternMode(
		controls::draft::TestPatternModeEnum::TestPatternModeOff);
	if (ret)
		return ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	ret = data->ipa_->start();
	if (ret)
		goto error;

	data->delayedCtrls_->reset();

	/*
	 * Start the ImgU video devices, buffers will be queued to the
	 * ImgU output and viewfinder when requests will be queued.
	 */
	ret = csi->start();
	if (ret)
		goto error;

	ret = isp->start();
	if (ret)
		goto error;

	return 0;

error:
	isp->stop();
	csi->stop();
	data->ipa_->stop();
	freeBuffers(camera);
	LOG(Camss, Error) << "Failed to start camera " << camera->id();

	return ret;
}

void PipelineHandlerCamss::stopDevice(Camera *camera)
{
	CamssCameraData *data = cameraData(camera);
	int ret = 0;

	data->cancelPendingRequests();

	data->ipa_->stop();

	ret |= data->isp_->stop();
	ret |= data->csi_.stop();
	if (ret)
		LOG(Camss, Warning) << "Failed to stop camera " << camera->id();

	freeBuffers(camera);
}

void CamssCameraData::cancelPendingRequests()
{
	processingRequests_ = {};

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
		 * Queue a buffer on the CIO2, using the raw stream buffer
		 * provided in the request, if any, or a CIO2 internal buffer
		 * otherwise.
		 */
		FrameBuffer *reqRawBuffer = request->findBuffer(&rawStream_);
		FrameBuffer *rawBuffer = csi_.queueBuffer(request, reqRawBuffer);
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

		ipa_->queueRequest(info->id, request->controls());

		pendingRequests_.pop();
		processingRequests_.push(request);
	}
}
#endif

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
	for (unsigned int i = 0 ; i < csiCams.size(); i++) {
		std::unique_ptr<CamssCameraData> data =
			std::make_unique<CamssCameraData>(this);
		std::set<Stream *> streams = {
			&data->outStream_,
			&data->rawStream_,
		};
		data->csi_ = std::move(csiCams[i]);

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

		data->csi_->frameStart().connect(data.get(),
						 &CamssCameraData::frameStart);
		data->csi_->bufferReady().connect(data.get(),
						  &CamssCameraData::csiBufferReady);
		data->csi_->bufferAvailable.connect(data.get(),
						    &CamssCameraData::queuePendingRequests);


		// FIXME start with assigning softISP, extend with OPE support
		// data->isp_ = bla();

#if 0
		data->isp_->input_->bufferReady.connect(&data->csi_,
							 &CIO2Device::tryReturnBuffer);
		data->isp_->output_->bufferReady.connect(data.get(),
							  &CamssCameraData::ispOutputBufferReady);
		data->isp_->viewfinder_->bufferReady.connect(data.get(),
							      &CamssCameraData::ispOutputBufferReady);
		data->isp_->param_->bufferReady.connect(data.get(),
							 &CamssCameraData::paramBufferReady);
		data->isp_->stat_->bufferReady.connect(data.get(),
							&CamssCameraData::statBufferReady);
#endif
		/* Create and register the Camera instance. */
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), sensor->id(), streams);

		registerCamera(std::move(camera));
		numCameras++;		
	}

	return numCameras != 0;
}

#if 0
int CamssCameraData::loadIPA()
{
	ipa_ = IPAManager::createIPA<ipa::ipu3::IPAProxyCamss>(pipe(), 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &CamssCameraData::setSensorControls);
	ipa_->paramsComputed.connect(this, &CamssCameraData::paramsComputed);
	ipa_->metadataReady.connect(this, &CamssCameraData::metadataReady);

	/*
	 * Pass the sensor info to the IPA to initialize controls.
	 *
	 * \todo Find a way to initialize IPA controls without basing their
	 * limits on a particular sensor mode. We currently pass sensor
	 * information corresponding to the largest sensor resolution, and the
	 * IPA uses this to compute limits for supported controls. There's a
	 * discrepancy between the need to compute IPA control limits at init
	 * time, and the fact that those limits may depend on the sensor mode.
	 * Research is required to find out to handle this issue.
	 */
	CameraSensor *sensor = csi_.sensor();
	V4L2SubdeviceFormat sensorFormat = {};
	sensorFormat.size = sensor->resolution();
	int ret = sensor->setFormat(&sensorFormat);
	if (ret)
		return ret;

	IPACameraSensorInfo sensorInfo{};
	ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor->model() + ".yaml", "uncalibrated.yaml");

	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 sensorInfo, sensor->controls(), &ipaControls_);
	if (ret) {
		LOG(Camss, Error) << "Failed to initialise the Camss IPA";
		return ret;
	}

	return 0;
}
#endif

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

#if 0
void CamssCameraData::paramsComputed(unsigned int id)
{
	CamssFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	/* Queue all buffers from the request aimed for the ImgU. */
	for (const auto &[stream, outbuffer] : info->request->buffers()) {
		if (stream == &outStream_)
			isp_->output_->queueBuffer(outbuffer);
		else if (stream == &vfStream_)
			isp_->viewfinder_->queueBuffer(outbuffer);
	}

	info->paramBuffer->_d()->metadata().planes()[0].bytesused =
		sizeof(struct ipu3_uapi_params);
	isp_->param_->queueBuffer(info->paramBuffer);
	isp_->stat_->queueBuffer(info->statBuffer);
	isp_->input_->queueBuffer(info->rawBuffer);
}
#endif

void CamssCameraData::metadataReady(unsigned int id, const ControlList &metadata)
{
	CamssFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

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

	Request *request = info->request;

	pipe()->completeBuffer(request, buffer);
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

	/* If the buffer is cancelled force a complete of the whole request. */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		for (const auto &[stream, b] : request->buffers()) {
			b->_d()->cancel();
			pipe()->completeBuffer(request, b);
		}

		frameInfos_.remove(info);
		pipe()->completeRequest(request);
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

	info->effectiveSensorControls = delayedCtrls_->get(buffer->metadata().sequence);

	if (request->findBuffer(&rawStream_))
		pipe()->completeBuffer(request, buffer);

// FIXME
//	ipa_->computeParams(info->id, info->paramBuffer->cookie());
}

void CamssCameraData::paramBufferReady(FrameBuffer *buffer)
{
	CamssFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	info->paramDequeued = true;

	/*
	 * tryComplete() will delete info if it completes the CamssFrame.
	 * In that event, we must have obtained the Request before hand.
	 *
	 * \todo Improve the FrameInfo API to avoid this type of issue
	 */
	Request *request = info->request;

	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
}

/*
 * \brief Handle the start of frame exposure signal
 * \param[in] sequence The sequence number of frame
 */
void CamssCameraData::frameStart(uint32_t sequence)
{
	delayedCtrls_->applyControls(sequence);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerCamss, "camss")

} /* namespace libcamera */
