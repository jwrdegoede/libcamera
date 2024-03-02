/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * atomisp.cpp - Pipeline handler for atomisp devices
 *
 * The atomisp ISP needs some extra lines/columns when debayering and also has
 * some max resolution limitations, this causes the available output resolutions
 * to differ from the sensor resolutions.
 *
 * The atomisp driver's Android heritage means that it mostly works as a non
 * media-controller centric v4l2 device, primarily controlled through its
 * /dev/video# node. The driver takes care of setting up the pipeline itself
 * propagating try / set fmt calls down from its single /dev/video# node to
 * the selected sensor taking the necessary padding, etc. into account.
 *
 * Therefor things like getting the list of support formats / sizes and tryFmt()
 * / setFmt() calls are all done on the /dev/video# node instead of on subdevs,
 * this avoids having to duplicate the padding, etc. logic here.
 * Note this requires enabling the ISP <-> CSI receiver for the current sensor
 * even when only querying / trying formats.
 *
 * Copyright (C) 2024, Hans de Goede <hansg@kernel.org>
 *
 * Partially based on simple.cpp and uvcvideo.cpp which are:
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 * Copyright (C) 2019, Google Inc.
 */

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_proxy.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Atomisp)

class AtomispCameraData : public Camera::Private
{
public:
	AtomispCameraData(PipelineHandler *pipe, V4L2VideoDevice *video)
		: Camera::Private(pipe), video_(video)
	{
	}

	int init(MediaEntity *sensor);
	void bufferReady(FrameBuffer *buffer);
	void statsReady(uint32_t frame, uint32_t bufferId);
	void setSensorControls(const ControlList &sensorControls);

	/* This is owned by AtomispPipelineHandler and shared by the cameras */
	V4L2VideoDevice *video_;
	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<DelayedControls> delayedCtrls_;
	std::unique_ptr<SwStatsCpu> stats_;
	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
	SharedMemObject<DebayerParams> debayerParams_;
	Stream stream_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
	MediaLink *csiReceiverIspLink_;
};

class AtomispCameraConfiguration : public CameraConfiguration
{
public:
	AtomispCameraConfiguration()
		: CameraConfiguration() {}

	Status validate() override;
};

class AtomispPipelineHandler : public PipelineHandler
{
public:
	AtomispPipelineHandler(CameraManager *manager)
		: PipelineHandler(manager) {}

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
	bool acquireDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	AtomispCameraData *cameraData(Camera *camera)
	{
		return static_cast<AtomispCameraData *>(camera->_d());
	}

	std::unique_ptr<V4L2VideoDevice> video_;
};

bool AtomispPipelineHandler::acquireDevice(Camera *camera)
{
	AtomispCameraData *data = cameraData(camera);

	/* atomisp can run only 1 sensor / camera at a time */
	if (data->video_->isOpen())
		return false;

	int ret = data->video_->open();
	if (ret != 0)
		return false;

	ret = data->csiReceiverIspLink_->setEnabled(true);
	if (ret) {
		data->video_->close();
		return false;
	}

	return true;
}

void AtomispPipelineHandler::releaseDevice(Camera *camera)
{
	AtomispCameraData *data = cameraData(camera);

	data->csiReceiverIspLink_->setEnabled(false);
	data->video_->close();
}

CameraConfiguration::Status AtomispCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (orientation != Orientation::Rotate0) {
		orientation = Orientation::Rotate0;
		status = Adjusted;
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];
	const StreamFormats &formats = cfg.formats();
	const PixelFormat pixelFormat = cfg.pixelFormat;
	const Size size = cfg.size;

	const std::vector<PixelFormat> pixelFormats = formats.pixelformats();
	auto iter = std::find(pixelFormats.begin(), pixelFormats.end(), pixelFormat);
	if (iter == pixelFormats.end()) {
		cfg.pixelFormat = pixelFormats.front();
		LOG(Atomisp, Debug)
			<< "Adjusting pixel format from " << pixelFormat
			<< " to " << cfg.pixelFormat;
		status = Adjusted;
	}

	const std::vector<Size> &formatSizes = formats.sizes(cfg.pixelFormat);
	cfg.size = formatSizes.front();
	for (const Size &formatsSize : formatSizes) {
		if (formatsSize > size)
			break;

		cfg.size = formatsSize;
	}

	if (cfg.size != size) {
		LOG(Atomisp, Debug)
			<< "Adjusting size from " << size << " to " << cfg.size;
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	switch (cfg.pixelFormat) {
	case formats::YUV420:
		/* atomisp stride must be a multiple of 32 */
		cfg.stride = (cfg.size.width + 31) & ~31;
		cfg.frameSize = cfg.stride * cfg.size.height * 3 / 2;
		break;
	default:
		LOG(Atomisp, Error)
			<< "Unknown pixel-format " << cfg.pixelFormat;
		return Invalid;
	}

	if (cfg.colorSpace != ColorSpace::Rec709) {
		cfg.colorSpace = ColorSpace::Rec709;
		status = Adjusted;
	}

	return status;
}

std::unique_ptr<CameraConfiguration>
AtomispPipelineHandler::generateConfiguration(Camera *camera,
					      Span<const StreamRole> roles)
{
	AtomispCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<AtomispCameraConfiguration>();

	if (roles.empty())
		return config;

	StreamFormats formats(data->formats_);
	StreamConfiguration cfg(formats);

	cfg.pixelFormat = formats.pixelformats().front();
	cfg.size = formats.sizes(cfg.pixelFormat).back();
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int AtomispPipelineHandler::configure(Camera *camera, CameraConfiguration *config)
{
	AtomispCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);

	int ret = data->stats_->configure(cfg);
	if (ret)
		return ret;

	ipa::soft::IPAConfigInfo configInfo;
	configInfo.sensorControls = data->sensor_->controls();

	ret = data->ipa_->configure(configInfo);
	if (ret)
		return ret;

	V4L2PixelFormat fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	V4L2DeviceFormat format;

	format.size = cfg.size;
	format.fourcc = fourcc;
	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size) {
		LOG(Atomisp, Error)
			<< "format mismatch req " << cfg.size << " got " << format.size;
		return -EINVAL;
	}

	if (format.fourcc != fourcc) {
		LOG(Atomisp, Error)
			<< "format mismatch req " << fourcc << " got " << format.fourcc;
		return -EINVAL;
	}

	data->stats_->setWindow(Rectangle(cfg.size));

	cfg.setStream(&data->stream_);

	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { 2, false } },
		{ V4L2_CID_EXPOSURE, { 2, false } },
	};
	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(data->sensor_->device(),
						  params);
	data->video_->frameStart.connect(data->delayedCtrls_.get(),
					 &DelayedControls::applyControls);
	return 0;
}

int AtomispPipelineHandler::exportFrameBuffers(Camera *camera, Stream *stream,
					       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int AtomispPipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;
	int ret;

	ret = data->video_->importBuffers(count);
	if (ret)
		return ret;

	video_->bufferReady.connect(data, &AtomispCameraData::bufferReady);

	ret = data->video_->streamOn();
	if (ret) {
		video_->bufferReady.disconnect(data, &AtomispCameraData::bufferReady);
		data->video_->releaseBuffers();
		return ret;
	}

	ret = data->ipa_->start();
	if (ret) {
		data->video_->streamOff();
		video_->bufferReady.disconnect(data, &AtomispCameraData::bufferReady);
		data->video_->releaseBuffers();
		return ret;
	}

	return 0;
}

void AtomispPipelineHandler::stopDevice(Camera *camera)
{
	AtomispCameraData *data = cameraData(camera);

	data->video_->streamOff();
	video_->bufferReady.disconnect(data, &AtomispCameraData::bufferReady);
	data->video_->releaseBuffers();
	data->ipa_->stop();
}

int AtomispPipelineHandler::queueRequestDevice(Camera *camera, Request *request)
{
	AtomispCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(Atomisp, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = data->video_->queueBuffer(buffer);
	if (ret)
		return ret;

	data->ipa_->queueRequest(request->sequence(), request->controls());
	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */
bool AtomispPipelineHandler::match(DeviceEnumerator *enumerator)
{
	std::vector<MediaEntity *> sensors;
	MediaEntity *videoEntity = NULL;
	DeviceMatch dm("atomisp-isp2");
	MediaDevice *media;

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	for (MediaEntity *entity : media->entities()) {
		switch (entity->function()) {
		case MEDIA_ENT_F_CAM_SENSOR:
			sensors.push_back(entity);
			break;
		case MEDIA_ENT_F_IO_V4L:
			videoEntity = entity;
			break;
		}
	}

	if (!videoEntity) {
		LOG(Atomisp, Error) << "Could not find the video device";
		return false;
	}
	if (sensors.empty()) {
		LOG(Atomisp, Error) << "No sensor found";
		return false;
	}

	/* Create and open the video device. */
	video_ = std::make_unique<V4L2VideoDevice>(videoEntity);
	int ret = video_->open();
	if (ret)
		return false;

	/* Create and register a camera for each sensor */
	bool registered = false;
	for (MediaEntity *sensor : sensors) {
		std::unique_ptr<AtomispCameraData> data =
			std::make_unique<AtomispCameraData>(this, video_.get());

		if (data->init(sensor))
			continue;

		const std::string &id = data->sensor_->id();
		std::set<Stream *> streams{ &data->stream_ };
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), id, streams);
		registerCamera(std::move(camera));
		registered = true;
	}

	/*
	 * atomisp cameras share a single /dev/video# node. The shared node
	 * gets opened from acquireDevice() to allow only one camera to be
	 * acquired at a time.
	 * This also works around a kernel bug (which needs to be fixed) where
	 * the node needs to be closed for the ISP to runtime-suspend.
	 */
	video_->close();

	return registered;
}

int AtomispCameraData::init(MediaEntity *sensor)
{
	sensor_ = std::make_unique<CameraSensor>(sensor);
	MediaEntity *csi_receiver;
	const MediaPad *source_pad;
	const MediaLink *link;

	int ret = sensor_->init();
	if (ret)
		return ret;

	debayerParams_ = SharedMemObject<DebayerParams>("debayer_params");
	if (!debayerParams_) {
		LOG(Atomisp, Error) << "Failed to create shared memory for parameters";
		return -ENOMEM;
	}

	stats_ = std::make_unique<SwStatsCpu>();
	if (!stats_->isValid()) {
		LOG(Atomisp, Error) << "Failed to create SwStatsCpu object";
		return -ENOMEM;
	}

	ipa_ = IPAManager::createIPA<ipa::soft::IPAProxySoft>(pipe(), 0, 0, "simple");
	if (!ipa_) {
		LOG(Atomisp, Error) << "Creating IPA failed";
		return -ENOMEM;
	}

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor_->model() + "_atomisp.yaml",
					"uncalibrated_atomisp.yaml");

	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor_->model() },
			 stats_->getStatsFD(), debayerParams_.fd(),
			 sensor_->controls());
	if (ret) {
		LOG(Atomisp, Error) << "IPA init failed";
		return ret;
	}

	source_pad = sensor->getPadByIndex(0);
	if (source_pad == nullptr) {
		LOG(Atomisp, Error)
			<< "Camera " << sensor_->model() << " doesn't have pad0";
		return -ENODEV;
	}

	link = source_pad->links()[0];
	csi_receiver = link->sink()->entity();

	source_pad = csi_receiver->getPadByIndex(1);
	if (source_pad == nullptr) {
		LOG(Atomisp, Error)
			<< "CSI receiver for " << sensor_->model() << " doesn't have pad1";
		return -ENODEV;
	}

	csiReceiverIspLink_ = source_pad->links()[0];

	ret = csiReceiverIspLink_->setEnabled(true);
	if (ret)
		return ret;

	/*
	 * The atomisp supports many different output formats but SwStatsCpu,
	 * used for 3A due to atomisp statistics being undocumented,
	 * only supports a few formats.
	 */
	std::vector<PixelFormat> supported({ formats::YUV420 });

	for (const auto &format : video_->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();

		if (std::find(supported.begin(), supported.end(), pixelFormat) == supported.end())
			continue;

		formats_[pixelFormat] = format.second;
	}

	csiReceiverIspLink_->setEnabled(false);

	if (formats_.empty()) {
		LOG(Atomisp, Error)
			<< "Camera " << sensor_->model()
			<< " doesn't expose any supported format";
		return -EINVAL;
	}

	properties_ = sensor_->properties();

	stats_->statsReady.connect(this, &AtomispCameraData::statsReady);
	ipa_->setSensorControls.connect(this, &AtomispCameraData::setSensorControls);

	return 0;
}

void AtomispCameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	if (buffer->metadata().status == FrameMetadata::FrameSuccess) {
		ipa_->fillParamsBuffer(request->sequence());

		/*
		 * Buffer ids are currently not used, so pass zero as buffer id.
		 *
		 * \todo Pass real bufferId once stats buffer passing is changed.
		 */
		stats_->processFrame(request->sequence(), 0, buffer);

		request->metadata().set(controls::SensorTimestamp,
					buffer->metadata().timestamp);
	}

	pipe()->completeBuffer(request, buffer);
	pipe()->completeRequest(request);
}

void AtomispCameraData::statsReady(uint32_t frame, uint32_t bufferId)
{
	ipa_->processStats(frame, bufferId, delayedCtrls_->get(frame));
}

void AtomispCameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
	ControlList ctrls(sensorControls);
	sensor_->setControls(&ctrls);
}

REGISTER_PIPELINE_HANDLER(AtomispPipelineHandler, "atomisp")

} /* namespace libcamera */
