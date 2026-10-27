/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS dummy ISP class for raw only image capture.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_isp_raw.h"

#include <string.h>

#include <libcamera/base/log.h>

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "camss_frames.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

/**
 * \class CamssIspRaw
 * \brief CAMSS dummy ISP class for raw only image capture
 */

/**
 * \brief Constructs CamssIspRaw object
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
CamssIspRaw::CamssIspRaw(PipelineHandler *pipe, const CameraSensor *sensor,
			 CamssFrames *frameInfos, ControlInfoMap *ispControls)
	: sensor_(sensor), frameInfos_(frameInfos)
{
	sharedParams_ = SharedMemObject<DebayerParams>("softIsp_params");
	if (!sharedParams_) {
		LOG(Camss, Error) << "Failed to create shared memory for parameters";
		return;
	}

	const CameraManager &cm = *pipe->cameraManager();

	stats_ = std::make_unique<SwStatsCpu>(cm);
	if (!stats_->isValid()) {
		LOG(Camss, Error) << "Failed to create SwStatsCpu object";
		return;
	}

	stats_->statsReady.connect(this,
				   [&](uint32_t frame, uint32_t statsBufferId) {
					   statsReady.emit(frame, statsBufferId);
				   });

	ipa_ = pipe->createIPA<ipa::softisp::IPAProxySoftIsp>(0, 0);
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
	bool lscEnabled;
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 stats_->getStatsFD(),
			 sharedParams_.fd(),
			 sensorInfo,
			 sensor->controls(),
			 ispControls,
			 &ccmEnabled,
			 &lscEnabled);
	if (ret) {
		LOG(Camss, Error) << "IPA init failed";
		ipa_.reset();
		return;
	}

	ipa_->metadataReady.connect(this,
				    [&](uint32_t frame, const ControlList &metadata) {
					    metadataReady.emit(frame, metadata);
				    });
	ipa_->setSensorControls.connect(this,
					[&](const ControlList &sensorControls) {
						setSensorControls.emit(sensorControls);
					});
}

CamssIspRaw::~CamssIspRaw() = default;

bool CamssIspRaw::isValid()
{
	return !!ipa_;
}

Size CamssIspRaw::getMargins([[maybe_unused]] PixelFormat inputFormat)
{
	return Size();
}

StreamConfiguration CamssIspRaw::generateConfiguration(const StreamConfiguration &raw) const
{
	return raw;
}

StreamConfiguration CamssIspRaw::validate(const StreamConfiguration &raw,
					  [[maybe_unused]] const StreamConfiguration &req) const
{
	return raw;
}

int CamssIspRaw::configure(const StreamConfiguration &inputCfg,
			   const StreamConfiguration &outputCfg,
			   ControlInfoMap *ispControls)
{
	ipa::softisp::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor_->controls();

	if (inputCfg.pixelFormat != outputCfg.pixelFormat ||
	    inputCfg.size != outputCfg.size)
		return -EINVAL;

	int ret = ipa_->configure(configInfo, ispControls);
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

	return 0;
}

int CamssIspRaw::allocateBuffers([[maybe_unused]] unsigned int bufferCount)
{
	frameInfos_->init({}, {});
	return 0;
}

void CamssIspRaw::freeBuffers()
{
	frameInfos_->clear();
}

int CamssIspRaw::exportOutputBuffers([[maybe_unused]] const Stream *stream,
				     [[maybe_unused]] unsigned int count,
				     [[maybe_unused]] std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* Must never be called when there only is a raw stream */
	return -EINVAL;
}

void CamssIspRaw::queueBuffers(Request *request, FrameBuffer *inputBuffer)
{
	uint32_t frame = request->sequence();

	DmaSyncer inputDmaSync(inputBuffer->planes()[0].fd, DmaSyncer::SyncType::Read);

	MappedFrameBuffer input(inputBuffer, MappedFrameBuffer::MapFlag::Read);
	if (!input.isValid()) {
		LOG(Camss, Error) << "mmap-ing buffer(s) failed";
		return;
	}

	ipa_->queueRequest(frame, request->controls());
	/*
	 * This emits statsReady which calls ipa_->processStats(), which emits
	 * metadataReady which makes camss.cpp mark the request as completed.
	 */
	stats_->processFrame(frame, 0, input);
}

void CamssIspRaw::processStats(const uint32_t frame, const uint32_t statsBufferId,
			       const ControlList &sensorControls)
{
	ipa_->processStats(frame, statsBufferId, sensorControls);
}

int CamssIspRaw::start()
{
	return ipa_->start();
}

void CamssIspRaw::stop()
{
	ipa_->stop();
}

} /* namespace libcamera */
