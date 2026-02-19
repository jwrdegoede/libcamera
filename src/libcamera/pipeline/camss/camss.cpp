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
#include <stdio.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Camss)

class CamssCameraData : public Camera::Private
{
public:
	CamssCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	std::unique_ptr<DelayedControls> delayedCtrls_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
	Stream stream_;
private:
	void setSensorControls(unsigned int id, const ControlList &sensorControls);
};

class CamssCameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;

	CamssCameraConfiguration(CamssCameraData *data)
		: CameraConfiguration(), data_(data) {}

	Status validate() override;

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
	PipelineHandlerCamss(CameraManager *manager)
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
	CamssCameraData *cameraData(Camera *camera)
	{
		return static_cast<CamssCameraData *>(camera->_d());
	}

	void getEntities(std::vector<MediaEntity *> &ents, const char *fmt, unsigned int max);

	static constexpr unsigned int kMinCsiPhys = 2;
	static constexpr unsigned int kMaxCsiPhys = 2;
	static constexpr unsigned int kMinCsiDecoders = 2;
	static constexpr unsigned int kMaxCsiDecoders = 2;
	static constexpr unsigned int kMinVfes = 2;
	static constexpr unsigned int kMaxVfes = 2;
	std::shared_ptr<MediaDevice> camssMediaDev_;
	std::vector<MediaEntity *> csiPhys_;
	std::vector<MediaEntity *> csiDecoders_;
	std::vector<MediaEntity *> VfeRdi0s_;
};

CameraConfiguration::Status CamssCameraConfiguration::validate()
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
		LOG(Camss, Debug)
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
		LOG(Camss, Debug)
			<< "Adjusting size from " << size << " to " << cfg.size;
		status = Adjusted;
	}

	return status;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerCamss::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	CamssCameraData *data = cameraData(camera);
	std::unique_ptr<CamssCameraConfiguration> config =
		std::make_unique<CamssCameraConfiguration>(data);

	if (roles.empty())
		return config;

	StreamFormats formats(data->formats_);
	StreamConfiguration cfg(formats);

	cfg.pixelFormat = formats.pixelformats().front();
	cfg.size = formats.sizes(cfg.pixelFormat).back();
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerCamss::configure([[maybe_unused]] Camera *camera, [[maybe_unused]] CameraConfiguration *c)
{
#if 0
	CamssCameraConfiguration *config =
		static_cast<CamssCameraConfiguration *>(c);
	CamssCameraData *data = cameraData(camera);
#endif

	return 0;
}

int PipelineHandlerCamss::exportFrameBuffers([[maybe_unused]] Camera *camera, [[maybe_unused]] Stream *stream,
					     [[maybe_unused]] std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
//	unsigned int count = stream->configuration().bufferCount;

	return -EINVAL;
}

int PipelineHandlerCamss::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	CamssCameraData *data = cameraData(camera);

	data->delayedCtrls_->reset();
	return 0;
}

void PipelineHandlerCamss::stopDevice([[maybe_unused]] Camera *camera)
{
//	CamssCameraData *data = cameraData(camera);
}

int PipelineHandlerCamss::queueRequestDevice([[maybe_unused]] Camera *camera, [[maybe_unused]] Request *request)
{
//	CamssCameraData *data = cameraData(camera);

	return 0;
}

void PipelineHandlerCamss::getEntities(std::vector<MediaEntity *> &ents, const char *fmt, unsigned int max)
{
	for (unsigned int i = 0; i < max; i++) {
		char name[16];
		snprintf(name, sizeof(name), fmt, i);
		MediaEntity *ent = camssMediaDev_->getEntityByName(name);
		if (ent)
			ents.push_back(ent);
	}
}

bool PipelineHandlerCamss::match(DeviceEnumerator *enumerator)
{
	DeviceMatch camssDm("qcom-camss");

	for (unsigned int i = 0; i < kMinCsiPhys; i++)
		camssDm.add("msm_csiphy" + std::to_string(i));

	for (unsigned int i = 0; i < kMinCsiDecoders; i++)
		camssDm.add("msm_csid" + std::to_string(i));

	for (unsigned int i = 0; i < kMinVfes; i++) {
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi0");
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi1");
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi2");
		camssDm.add("msm_vfe" + std::to_string(i) + "_pix");
	}

	camssMediaDev_ = acquireMediaDevice(enumerator, camssDm);
	if (!camssMediaDev_)
		return false;

	/*
	 * Disable all links that are enabled to start with a clean state,
	 * camera creation enables all valid links it finds.
	 */
	if (camssMediaDev_->disableLinks())
		return false;

	getEntities(csiPhys_, "msm_csiphy%d", kMaxCsiPhys);
	getEntities(csiDecoders_, "msm_csid%d", kMaxCsiDecoders);
	/* Only RDI0 is used for now */
	getEntities(VfeRdi0s_, "msm_vfe%d_rdi0", kMaxVfes);

	LOG(Camss, Info) << "Found "
		<< csiPhys_.size() << " CSI phys "
		<< csiDecoders_.size() << " CSI decoders "
		<< VfeRdi0s_.size() << " VFEs";

	return 0;
}

#if 0
/**
 * \brief Initialise CAMSS devices associated with cameras
 *
 * \return 0 on success or a negative error code for error or if no camera
 * has been created
 * \retval -ENODEV no camera has been created
 */
int PipelineHandlerCamss::registerCameras()
{
	int ret;

	/*
	 * For each CSI-2 receiver on the Camss, create a Camera if an
	 * image sensor is connected to it and the sensor can produce images
	 * in a compatible format.
	 */
	unsigned int numCameras = 0;
	for (unsigned int id = 0; id < 4 && numCameras < 2; ++id) {
		std::unique_ptr<CamssCameraData> data =
			std::make_unique<CamssCameraData>(this);
		std::set<Stream *> streams = {
			&data->outStream_,
			&data->vfStream_,
			&data->rawStream_,
		};
		CIO2Device *cio2 = &data->cio2_;

		ret = cio2->init(cio2MediaDev_, id);
		if (ret)
			continue;

		ret = data->loadIPA();
		if (ret)
			continue;

		/* Initialize the camera properties. */
		data->properties_ = cio2->sensor()->properties();

		ret = initControls(data.get());
		if (ret)
			continue;

		const CameraSensorProperties::SensorDelays &delays = cio2->sensor()->sensorDelays();
		std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
			{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
			{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		};

		data->delayedCtrls_ =
			std::make_unique<DelayedControls>(cio2->sensor()->device(),
							  params);
		data->cio2_.frameStart().connect(data.get(),
						 &CamssCameraData::frameStart);

		/* Convert the sensor rotation to a transformation */
		const auto &rotation = data->properties_.get(properties::Rotation);
		if (!rotation)
			LOG(Camss, Warning) << "Rotation control not exposed by "
					   << cio2->sensor()->id()
					   << ". Assume rotation 0";

		/**
		 * \todo Dynamically assign ImgU and output devices to each
		 * stream and camera; as of now, limit support to two cameras
		 * only, and assign imgu0 to the first one and imgu1 to the
		 * second.
		 */
		data->imgu_ = numCameras ? &imgu1_ : &imgu0_;

		/*
		 * Connect video devices' 'bufferReady' signals to their
		 * slot to implement the image processing pipeline.
		 *
		 * Frames produced by the CIO2 unit are passed to the
		 * associated ImgU input where they get processed and
		 * returned through the ImgU main and secondary outputs.
		 */
		data->cio2_.bufferReady().connect(data.get(),
						  &CamssCameraData::cio2BufferReady);
		data->cio2_.bufferAvailable.connect(
			data.get(), &CamssCameraData::queuePendingRequests);
		data->imgu_->input_->bufferReady.connect(&data->cio2_,
							 &CIO2Device::tryReturnBuffer);
		data->imgu_->output_->bufferReady.connect(data.get(),
							  &CamssCameraData::imguOutputBufferReady);
		data->imgu_->viewfinder_->bufferReady.connect(data.get(),
							      &CamssCameraData::imguOutputBufferReady);
		data->imgu_->param_->bufferReady.connect(data.get(),
							 &CamssCameraData::paramBufferReady);
		data->imgu_->stat_->bufferReady.connect(data.get(),
							&CamssCameraData::statBufferReady);

		/* Create and register the Camera instance. */
		const std::string &cameraId = cio2->sensor()->id();
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), cameraId, streams);

		registerCamera(std::move(camera));

		LOG(Camss, Info)
			<< "Registered Camera[" << numCameras << "] \""
			<< cameraId << "\""
			<< " connected to CSI-2 receiver " << id;

		numCameras++;
	}

	return numCameras ? 0 : -ENODEV;
}
#endif

void CamssCameraData::setSensorControls([[maybe_unused]] unsigned int id,
				       const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerCamss, "camss")

} /* namespace libcamera */
