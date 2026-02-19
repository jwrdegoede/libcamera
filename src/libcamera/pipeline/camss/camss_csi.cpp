/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS CSI phy/decoder and VFE handling
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Partially based on other pipeline-handlers which are:
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 * Copyright (C) 2019, Google Inc.
 */

#include "camss_csi.h"

#include <cmath>
#include <limits>

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>
#include <libcamera/transform.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

CamssCsiCamera::CamssCsiCamera()
{
}

/**
 * \brief Get output V4L2PixelFormat for media bus code
 *
 * Get output video node V4L2PixelFormat for the given media bus code.
 * \param[in] code The media bus code
 *
 * \return V4L2PixelFormat
 */
V4L2PixelFormat CamssCsiCamera::mbusCodeToV4L2PixelFormat(unsigned int code) const
{
	V4L2VideoDevice::Formats formats = output_->formats(code);

	if (formats.empty()) {
		LOG(Camss, Error)
			<< "No formats for media bus code " << code;
		return V4L2PixelFormat();
	}

	/*
	 * camss supports only 1 V4L2 output format per media bus code and not
	 * multiple (e.g. not mipi-packed + sparse for raw-bayer).
	 */
	return formats.begin()->first;
}

/**
 * \brief Get output PixelFormat for media bus code
 *
 * Get output video node PixelFormat for the given media bus code.
 * \param[in] code The media bus code
 *
 * \return PixelFormat
 */
PixelFormat CamssCsiCamera::mbusCodeToPixelFormat(unsigned int code) const
{
	V4L2PixelFormat v4l2Format = mbusCodeToV4L2PixelFormat(code);
	if (!v4l2Format.isValid())
		return PixelFormat();

	return v4l2Format.toPixelFormat(true);
}

/**
 * \brief Get media bus code for desired output PixelFormat
 *
 * Get the media bus code for a desired output video node PixelFormat.
 * \param[in] format The PixelFormat
 *
 * \return media bus code or 0 if no matching code is found
 */
unsigned int CamssCsiCamera::PixelFormatToMbusCode(const PixelFormat &format) const
{
	for (unsigned int code : sensor_->mbusCodes()) {
		PixelFormat pixelFormat = mbusCodeToPixelFormat(code);
		if (pixelFormat == format)
			return code;
	}

	return 0;
}

// TODO maybe make size + format in,out and drop outputFormat ?
/**
 * \brief Configure the CamssCsi unit
 * \param[in] size The requested CamssCsi output frame size
 * \param[in] format The requested CamssCsi output PixelFormat
 * \param[in] transform The transformation to be applied on the image sensor
 * \param[out] outputFormat The CamssCsi output V4L2DeviceFormat format
 * \return 0 on success or a negative error code otherwise
 */
int CamssCsiCamera::configure(const Size &size, const PixelFormat &format,
			      const Transform &transform, V4L2DeviceFormat *outputFormat)
{
	V4L2SubdeviceFormat sensorFormat;
	int ret;

	sensorFormat = getSensorFormat(size, format);
	/* This updates format with the actual established format */
	ret = sensor_->setFormat(&sensorFormat, transform);
	if (ret)
		return ret;

	for (auto &link : links_) {
		if (!(link.link->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = link.link->setEnabled(true);
			if (ret)
				return ret;
		}

		MediaPad *sink = link.link->sink();
		ret = link.sinkSubdev->setFormat(sink->index(), &sensorFormat,
						 V4L2Subdevice::ActiveFormat);
		if (ret)
			return ret;
	}

	outputFormat->fourcc = mbusCodeToV4L2PixelFormat(sensorFormat.code);
	outputFormat->size = sensorFormat.size;
	outputFormat->planesCount = 1;

	ret = output_->setFormat(outputFormat);
	if (ret)
		return ret;

	LOG(Camss, Info) << "CamssCsi output format " << *outputFormat;

	return 0;
}

StreamConfiguration CamssCsiCamera::generateConfiguration(Size size) const
{
	StreamConfiguration cfg;

	/* Query the sensor static information for closest match. */
	V4L2SubdeviceFormat sensorFormat = getSensorFormat(size);

	cfg.size = sensorFormat.size;
	cfg.pixelFormat = mbusCodeToPixelFormat(sensorFormat.code);
	cfg.bufferCount = kBufferCount;

	return cfg;
}

/**
 * \brief Retrieve the best sensor format for a desired output size and format
 * \param[in] size The desired size
 * \param[in] format The desired PixelFormat
 *
 * \a size indicates the desired size at the output of the sensor. This method
 * selects the best media bus code and size supported by the sensor according
 * to the following criteria.
 *
 * - The desired \a size shall fit in the sensor output size to avoid the need
 *   to up-scale.
 * - The aspect ratio of sensor output size shall be as close as possible to
 *   the sensor's native resolution field of view.
 * - The sensor output size shall be as small as possible to lower the required
 *   bandwidth.
 *
 * When \a format is empty and multiple media bus codes can produce the same
 * size, the media bus code with the highest bits-per-pixel is selected.
 *
 * The returned sensor output format is guaranteed to be acceptable by the
 * setFormat() method without any modification.
 *
 * \return The best sensor output format matching the desired size and format
 * on success, or an empty format otherwise.
 */
V4L2SubdeviceFormat CamssCsiCamera::getSensorFormat(Size size, const PixelFormat &format) const
{
	unsigned int desiredArea = size.width * size.height;
	unsigned int bestArea = std::numeric_limits<unsigned int>::max();
	const Size &resolution = sensor_->resolution();
	std::vector<unsigned int> mbusCodes;
	float desiredRatio = static_cast<float>(resolution.width) /
			     resolution.height;
	float bestRatio = std::numeric_limits<float>::max();
	unsigned int desiredCode = 0;
	uint32_t bestCode = 0;
	uint8_t bestDepth = 0;
	Size bestSize;

	/* If no desired size use the sensor resolution. */
	if (size.isNull())
		size = resolution;

	if (format.isValid())
		desiredCode = PixelFormatToMbusCode(format);

	if (desiredCode)	
		mbusCodes.push_back(desiredCode);
	else
		mbusCodes = sensor_->mbusCodes();

	for (unsigned int code : mbusCodes) {
		PixelFormat pixelFormat = mbusCodeToPixelFormat(code);
		BayerFormat bayerFormat = BayerFormat::fromPixelFormat(pixelFormat);

		/* Only Bayer formats are supported for now */
		if (!bayerFormat.isValid())
			continue;

		const auto sizes = sensor_->sizes(code);
		if (!sizes.size())
			continue;

		for (const Size &sz : sizes) {
			if (sz.width < size.width || sz.height < size.height)
				continue;

			float ratio = static_cast<float>(sz.width) / sz.height;
			/*
			 * Ratios can differ by small mantissa difference which
			 * can affect the selection of the sensor output size
			 * wildly. We are interested in selection of the closest
			 * size with respect to the desired output size, hence
			 * comparing it with a single precision digit is enough.
			 */
			ratio = static_cast<unsigned int>(ratio * 10) / 10.0;
			float ratioDiff = std::abs(ratio - desiredRatio);
			unsigned int area = sz.width * sz.height;
			unsigned int areaDiff = area - desiredArea;

			if (ratioDiff > bestRatio)
				continue;

			if ((ratioDiff < bestRatio || areaDiff < bestArea) ||
			    (ratioDiff == bestRatio && areaDiff == bestArea &&
			     bayerFormat.bitDepth > bestDepth)) {
				bestRatio = ratioDiff;
				bestArea = areaDiff;
				bestSize = sz;
				bestCode = code;
				bestDepth = bayerFormat.bitDepth;
			}
		}
	}

	if (bestSize.isNull()) {
		LOG(Camss, Warning) << "No supported format or size found";
		return {};
	}

	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.code = bestCode;
	sensorFormat.size = bestSize;

	return sensorFormat;
}

int CamssCsiCamera::exportBuffers(unsigned int count,
			      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

int CamssCsiCamera::start()
{
	int ret = output_->exportBuffers(kBufferCount, &buffers_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(kBufferCount);
	if (ret)
		LOG(Camss, Error) << "Failed to import CamssCsi buffers";

	for (std::unique_ptr<FrameBuffer> &buffer : buffers_)
		availableBuffers_.push(buffer.get());

	ret = output_->streamOn();
	if (ret) {
		freeBuffers();
		return ret;
	}

	return 0;
}

int CamssCsiCamera::stop()
{
	int ret;

	ret = output_->streamOff();

	freeBuffers();

	return ret;
}

FrameBuffer *CamssCsiCamera::queueBuffer(Request *request, FrameBuffer *rawBuffer)
{
	FrameBuffer *buffer = rawBuffer;

	/* If no buffer is provided in the request, use an internal one. */
	if (!buffer) {
		if (availableBuffers_.empty()) {
			LOG(Camss, Debug) << "CamssCsi buffer underrun";
			return nullptr;
		}

		buffer = availableBuffers_.front();
		availableBuffers_.pop();
		buffer->_d()->setRequest(request);
	}

	int ret = output_->queueBuffer(buffer);
	if (ret)
		return nullptr;

	return buffer;
}

void CamssCsiCamera::tryReturnBuffer(FrameBuffer *buffer)
{
	/*
	 * \todo Once more pipelines deal with buffers that may be allocated
	 * internally or externally this pattern might become a common need. At
	 * that point this check should be moved to something clever in
	 * FrameBuffer.
	 */
	for (const std::unique_ptr<FrameBuffer> &buf : buffers_) {
		if (buf.get() == buffer) {
			availableBuffers_.push(buffer);
			break;
		}
	}

	bufferAvailable.emit();
}

void CamssCsiCamera::freeBuffers()
{
	availableBuffers_ = {};
	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(Camss, Error) << "Failed to release CamssCsi buffers";
}

CamssCsi::CamssCsi()
{
}

void CamssCsi::getEntities(std::vector<MediaEntity *> &ents, const char *fmt, unsigned int max)
{
	for (unsigned int i = 0; i < max; i++) {
		char name[16];
		snprintf(name, sizeof(name), fmt, i);
		MediaEntity *ent = camssMediaDev_->getEntityByName(name);
		/* Reverse sort so that [pop_]back() gets the first ent. */
		if (ent)
			ents.insert(ents.begin(), ent);
	}
}

CamssCsi::Cameras CamssCsi::match(PipelineHandler *pipe, DeviceEnumerator *enumerator)
{
	DeviceMatch camssDm("qcom-camss");
	Cameras cameras;

	/*
	 * On SoCs where the CSI-phy is a separate dt-node (e.g. x1e), only
	 * actually used phys are there. So no match on "msm_csiphy%d".
	 */

	for (unsigned int i = 0; i < kMinCsiDecoders; i++)
		camssDm.add("msm_csid" + std::to_string(i));

	for (unsigned int i = 0; i < kMinVfes; i++) {
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi0");
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi1");
		camssDm.add("msm_vfe" + std::to_string(i) + "_rdi2");
		camssDm.add("msm_vfe" + std::to_string(i) + "_pix");
	}

	camssMediaDev_ = pipe->acquireMediaDevice(enumerator, camssDm);
	if (!camssMediaDev_)
		return {};

	/*
	 * Disable all links that are enabled to start with a clean state,
	 * CamssCsiCamera::configure() enables links as necessary.
	 * TODO: instead only disable links on used entities, to allow
	 * 2 separate libcamera instances to drive 2 different sensors.
	 * This will also require changes to PipelineHandler::acquire() to
	 * allow a more fine grained version of that locking a list of
	 * subdevs associated with a Camera instead of the mediactl node.
	 */
	if (camssMediaDev_->disableLinks())
		return {};

	getEntities(phys_, "msm_csiphy%d", kMaxCsiPhys);
	getEntities(csids_, "msm_csid%d", kMaxCsiDecoders);
	/* Only RDI0 is used for now */
	getEntities(vfes_, "msm_vfe%d_rdi0", kMaxVfes);

	LOG(Camss, Info) << "Found "
		<< phys_.size() << " CSI phy(s) "
		<< csids_.size() << " CSI decoders "
		<< vfes_.size() << " VFEs";

	for (auto &phy : phys_) {
		std::unique_ptr<CamssCsiCamera> camera = enumCamera(phy);
		if (camera)
			cameras.push_back(std::move(camera));
	}

	return cameras;
}

std::unique_ptr<CamssCsiCamera> CamssCsi::enumCamera(MediaEntity *phy)
{
	std::unique_ptr<CamssCsiCamera> cam = std::make_unique<CamssCsiCamera>();
	int ret;

	/* CSI phy has a sink pad for the sensor at index 0. */
	if (phy->pads().empty() || phy->pads()[0]->links().empty())
		return nullptr;

	MediaEntity *sensor =
		phy->pads()[0]->links()[0]->source()->entity();
	cam->sensor_ = CameraSensorFactoryBase::create(sensor);
	if (!cam->sensor_)
		return nullptr;

	if (csids_.empty()) {
		LOG(Camss, Warning)
			<< "Not enough CSI decoders to enumerate all cameras\n";
		return nullptr;
	}

	if (vfes_.empty()) {
		LOG(Camss, Warning)
			<< "Not enough VFEs to enumerate all cameras\n";
		return nullptr;
	}

	MediaEntity *csid = csids_.back();
	MediaEntity *vfe = vfes_.back();

	for (unsigned int i = 0; i < CamssCsiCamera::LinkCount; i++) {
		auto &link = cam->links_[i];

		switch (i) {
		case CamssCsiCamera::SensorPhyLink:
			link.link = camssMediaDev_->link(sensor, 0, phy, 0);
			link.sinkSubdev = std::make_unique<V4L2Subdevice>(phy);
			break;
		case CamssCsiCamera::PhyCsidLink:
			link.link = camssMediaDev_->link(phy, 1, csid, 0);
			link.sinkSubdev = std::make_unique<V4L2Subdevice>(csid);
			break;
		case CamssCsiCamera::CsidVfeLink:
			link.link = camssMediaDev_->link(csid, 1, vfe, 0);
			link.sinkSubdev = std::make_unique<V4L2Subdevice>(vfe);
			break;
		}

		if (!link.link) {
			LOG(Camss, Error) << "Error enumerating links";
			return nullptr;
		}

		ret = link.sinkSubdev->open();
		if (ret)
			return nullptr;
	}

	/* VFE has a source pad to its /dev/video# node at index 1. */
	if (vfe->pads().size() < 2 || vfe->pads()[1]->links().empty())
		return nullptr;

	MediaEntity *output =
		vfe->pads()[1]->links()[0]->sink()->entity();
	cam->output_ = std::make_unique<V4L2VideoDevice>(output);
	ret = cam->output_->open();
	if (ret)
		return nullptr;

#if 0
	// FIXME this is just a test, drop me
	ret = cam->configure();
	if (ret)
		LOG(Camss, Error) << "Error configure() failed";
#endif

	LOG(Camss, Info)
		<< "Sensor " << cam->sensor_->entity()->name()
		<< " decoder " << csid->name()
		<< " VFE " << vfe->name();

	csids_.pop_back();
	vfes_.pop_back();
	return cam;
}

} /* namespace libcamera */
