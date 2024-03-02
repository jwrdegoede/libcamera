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
#include <fstream>
#include <iomanip>
#include <math.h>
#include <memory>
#include <tuple>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
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

	int acquireSensor();
	void releaseSensor();
	int tryFormat(V4L2DeviceFormat *format);

	/* This is owned by AtomispPipelineHandler and shared by the cameras */
	V4L2VideoDevice *video_;
	std::unique_ptr<CameraSensor> sensor_;
	Stream stream_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
	MediaLink *csiReceiverIspLink_;
	V4L2DeviceFormat format_;
};

class AtomispCameraConfiguration : public CameraConfiguration
{
public:
	AtomispCameraConfiguration(AtomispCameraData *data);

	Status validate() override;

private:
	AtomispCameraData *data_;
};

class AtomispPipelineHandler : public PipelineHandler
{
public:
	AtomispPipelineHandler(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	int acquireVideoDevice(AtomispCameraData *data);
	void releaseVideoDevice(AtomispCameraData *data);

private:
	AtomispCameraData *cameraData(Camera *camera)
	{
		return static_cast<AtomispCameraData *>(camera->_d());
	}

	std::unique_ptr<V4L2VideoDevice> video_;
	AtomispCameraData *videoOwner_;
};

AtomispCameraConfiguration::AtomispCameraConfiguration(AtomispCameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status AtomispCameraConfiguration::validate()
{
	Status status = Valid;
	int ret;

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

	V4L2DeviceFormat format;
	format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data_->tryFormat(&format);;
	if (ret)
		return Invalid;

	cfg.stride = format.planes[0].bpl;
	cfg.frameSize = format.planes[0].size;

	if (cfg.colorSpace != format.colorSpace) {
		cfg.colorSpace = format.colorSpace;
		status = Adjusted;
	}

	return status;
}

AtomispPipelineHandler::AtomispPipelineHandler(CameraManager *manager)
	: PipelineHandler(manager), videoOwner_(nullptr)
{
}

std::unique_ptr<CameraConfiguration>
AtomispPipelineHandler::generateConfiguration(Camera *camera,
					      Span<const StreamRole> roles)
{
	AtomispCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<AtomispCameraConfiguration>(data);

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
	int ret;

	V4L2DeviceFormat format;
	format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->tryFormat(&format);;
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
		return -EINVAL;

	data->format_ = format;

	cfg.setStream(&data->stream_);

	return 0;
}

int AtomispPipelineHandler::exportFrameBuffers(Camera *camera, Stream *stream,
					       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int AtomispPipelineHandler::acquireVideoDevice(AtomispCameraData *data)
{
	if (videoOwner_ && videoOwner_ != data) {
		LOG(Atomisp, Info)
			<< "Failed to acquire pipeline "
			<< video_->deviceName() << " is in use";
		return -EBUSY;
	}

	videoOwner_ = data;
	return 0;
}

void AtomispPipelineHandler::releaseVideoDevice(AtomispCameraData *data)
{
	ASSERT(videoOwner_ == data);
	videoOwner_ = NULL;
}

int AtomispPipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;
	int ret;

	ret = data->acquireSensor();
	if (ret)
		return ret;

	ret = data->video_->setFormat(&data->format_);
	if (ret) {
		data->releaseSensor();
		return ret;
	}

	ret = data->video_->importBuffers(count);
	if (ret) {
		data->releaseSensor();
		return ret;
	}

	video_->bufferReady.connect(data, &AtomispCameraData::bufferReady);

	ret = data->video_->streamOn();
	if (ret) {
		video_->bufferReady.disconnect(data, &AtomispCameraData::bufferReady);
		data->video_->releaseBuffers();
		data->releaseSensor();
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
	data->releaseSensor();
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

	ret = acquireSensor();
	if (ret)
		return ret;

	/*
	 * The atomisp supports many different output formats but SwStatsCpu,
	 * used for 3A due to atomisp statistics being undocumented,
	 * only supports a few formats.
	 */
	std::vector<PixelFormat> supported({ formats::YUV420 });
//	Size resolution;

	for (const auto &format : video_->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();

		if (std::find(supported.begin(), supported.end(), pixelFormat) == supported.end())
			continue;

		formats_[pixelFormat] = format.second;

/* FIXME do we nee this to override properties::PixelArrayActiveAreas ?
		const std::vector<SizeRange> &sizeRanges = format.second;
		for (const SizeRange &sizeRange : sizeRanges) {
			if (sizeRange.max > resolution)
				resolution = sizeRange.max;
		} */
	}

	releaseSensor();

	if (formats_.empty()) {
		LOG(Atomisp, Error)
			<< "Camera " << sensor_->model()
			<< " doesn't expose any supported format";
		return -EINVAL;
	}

	properties_ = sensor_->properties();

	return 0;
}

void AtomispCameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	/* \todo Use the Atomisp metadata to calculate a more precise timestamp */
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	pipe()->completeBuffer(request, buffer);
	pipe()->completeRequest(request);
}

int AtomispCameraData::acquireSensor()
{
	AtomispPipelineHandler *pipe =
		static_cast<AtomispPipelineHandler *>(Camera::Private::pipe());
	int ret;

	ret = pipe->acquireVideoDevice(this);
	if (ret)
		return ret;

	ret = csiReceiverIspLink_->setEnabled(true);
	if (ret) {
		pipe->releaseVideoDevice(this);
		return ret;
	}

	return 0;
}

void AtomispCameraData::releaseSensor()
{
	AtomispPipelineHandler *pipe =
		static_cast<AtomispPipelineHandler *>(Camera::Private::pipe());

	csiReceiverIspLink_->setEnabled(false);
	pipe->releaseVideoDevice(this);
}

int AtomispCameraData::tryFormat(V4L2DeviceFormat *format)
{
	int ret;

	/*
	 * tryFmt() goes through the shared V4L2VideoDevice, make sure
	 * the tryFmt() is propagated to the right sensor.
	 */
	ret = acquireSensor();
	if (ret)
		return ret;

	ret = video_->tryFormat(format);
	releaseSensor();

	return ret;
}

REGISTER_PIPELINE_HANDLER(AtomispPipelineHandler)

} /* namespace libcamera */
