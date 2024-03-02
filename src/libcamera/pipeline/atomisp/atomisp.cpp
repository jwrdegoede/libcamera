/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Hans de Goede <hansg@kernel.org>
 *
 * atomisp.cpp - Pipeline handler for atomisp devices
 *
 * Based on uvcvideo.cpp which is:
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
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
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
	AtomispCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe), id_("atomisp")
	{
	}

	int init(MediaDevice *media);
	void bufferReady(FrameBuffer *buffer);

	const std::string &id() const { return id_; }

	std::unique_ptr<V4L2VideoDevice> video_;
	Stream stream_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
private:
	std::string id_;
};

class AtomispCameraConfiguration : public CameraConfiguration
{
public:
	AtomispCameraConfiguration(AtomispCameraData *data);

	Status validate() override;

private:
	AtomispCameraData *data_;
};

class PipelineHandlerAtomisp : public PipelineHandler
{
public:
	PipelineHandlerAtomisp(CameraManager *manager);

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
	AtomispCameraData *cameraData(Camera *camera)
	{
		return static_cast<AtomispCameraData *>(camera->_d());
	}
};

AtomispCameraConfiguration::AtomispCameraConfiguration(AtomispCameraData *data)
	: CameraConfiguration(), data_(data)
{
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

	V4L2DeviceFormat format;
	format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	int ret = data_->video_->tryFormat(&format);
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

PipelineHandlerAtomisp::PipelineHandlerAtomisp(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerAtomisp::generateConfiguration(Camera *camera,
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

int PipelineHandlerAtomisp::configure(Camera *camera, CameraConfiguration *config)
{
	AtomispCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format;
	format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
		return -EINVAL;

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerAtomisp::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int PipelineHandlerAtomisp::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;

	int ret = data->video_->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = data->video_->streamOn();
	if (ret < 0) {
		data->video_->releaseBuffers();
		return ret;
	}

	return 0;
}

void PipelineHandlerAtomisp::stopDevice(Camera *camera)
{
	AtomispCameraData *data = cameraData(camera);
	data->video_->streamOff();
	data->video_->releaseBuffers();
}

int PipelineHandlerAtomisp::queueRequestDevice(Camera *camera, Request *request)
{
	AtomispCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(Atomisp, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	return 0;
}

bool PipelineHandlerAtomisp::match(DeviceEnumerator *enumerator)
{
	MediaDevice *media;
	DeviceMatch dm("atomisp-isp2");

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<AtomispCameraData> data = std::make_unique<AtomispCameraData>(this);

	if (data->init(media))
		return false;

	/* Create and register the camera. */
	std::string id = data->id();
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	registerCamera(std::move(camera));

	return true;
}

int AtomispCameraData::init(MediaDevice *media)
{
	int ret;

	/* Locate and initialise the camera data with the default video node. */
	const std::vector<MediaEntity *> &entities = media->entities();
	auto entity = std::find_if(entities.begin(), entities.end(),
				   [](MediaEntity *e) {
					   return e->flags() & MEDIA_ENT_FL_DEFAULT;
				   });
	if (entity == entities.end()) {
		LOG(Atomisp, Error) << "Could not find a default video device";
		return -ENODEV;
	}

	/* Create and open the video device. */
	video_ = std::make_unique<V4L2VideoDevice>(*entity);
	ret = video_->open();
	if (ret)
		return ret;

	video_->bufferReady.connect(this, &AtomispCameraData::bufferReady);

	/*
	 * Populate the map of supported formats, and infer the camera sensor
	 * resolution from the largest size it advertises.
	 */
	Size resolution;
	for (const auto &format : video_->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (!pixelFormat.isValid())
			continue;

		formats_[pixelFormat] = format.second;

		const std::vector<SizeRange> &sizeRanges = format.second;
		for (const SizeRange &sizeRange : sizeRanges) {
			if (sizeRange.max > resolution)
				resolution = sizeRange.max;
		}
	}

	if (formats_.empty()) {
		LOG(Atomisp, Error)
			<< "Camera " << id_ << " (" << media->model()
			<< ") doesn't expose any supported format";
		return -EINVAL;
	}

	/* Populate the camera properties. */
	properties_.set(properties::Model, utils::toAscii(media->model()));
	properties_.set(properties::PixelArraySize, resolution);
	properties_.set(properties::PixelArrayActiveAreas, { Rectangle(resolution) });

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

REGISTER_PIPELINE_HANDLER(PipelineHandlerAtomisp)

} /* namespace libcamera */
