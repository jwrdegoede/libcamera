/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright 2022 NXP
 *
 * V4L2 M2M Format converter
 */

#include "libcamera/internal/converter/converter_v4l2_m2m.h"

#include <limits.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/utils.h>

#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

/**
 * \file converter/converter_v4l2_m2m.h
 * \brief V4L2 M2M based converter
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Converter)

/* -----------------------------------------------------------------------------
 * V4L2M2MConverter::V4L2M2MStream
 */

V4L2M2MConverter::V4L2M2MStream::V4L2M2MStream(V4L2M2MConverter *converter, const Stream *stream)
	: converter_(converter), stream_(stream)
{
	m2m_ = std::make_unique<V4L2M2MDevice>(converter->deviceNode());

	m2m_->output()->bufferReady.connect(this, &V4L2M2MStream::outputBufferReady);
	m2m_->capture()->bufferReady.connect(this, &V4L2M2MStream::captureBufferReady);

	int ret = m2m_->open();
	if (ret < 0)
		m2m_.reset();
}

int V4L2M2MConverter::V4L2M2MStream::configure(const StreamConfiguration &inputCfg,
					       const StreamConfiguration &outputCfg)
{
	V4L2PixelFormat videoFormat =
		m2m_->output()->toV4L2PixelFormat(inputCfg.pixelFormat);

	V4L2DeviceFormat format;
	format.fourcc = videoFormat;
	format.size = inputCfg.size;
	format.planesCount = 1;
	format.planes[0].bpl = inputCfg.stride;

	int ret = m2m_->output()->setFormat(&format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set input format: " << strerror(-ret);
		return ret;
	}

	if (format.fourcc != videoFormat || format.size != inputCfg.size ||
	    format.planes[0].bpl != inputCfg.stride) {
		LOG(Converter, Error)
			<< "Input format not supported (requested "
			<< inputCfg.size << "-" << videoFormat
			<< ", got " << format << ")";
		return -EINVAL;
	}

	/* Set the pixel format and size on the output. */
	videoFormat = m2m_->capture()->toV4L2PixelFormat(outputCfg.pixelFormat);
	format = {};
	format.fourcc = videoFormat;
	format.size = outputCfg.size;

	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set output format: " << strerror(-ret);
		return ret;
	}

	if (format.fourcc != videoFormat || format.size != outputCfg.size) {
		LOG(Converter, Error)
			<< "Output format not supported";
		return -EINVAL;
	}

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	if (converter_->features() & Feature::InputCrop) {
		Rectangle minCrop;
		Rectangle maxCrop;

		/* Find crop bounds */
		minCrop.width = 1;
		minCrop.height = 1;
		maxCrop.width = UINT_MAX;
		maxCrop.height = UINT_MAX;

		ret = setInputSelection(V4L2_SEL_TGT_CROP, &minCrop);
		if (ret) {
			LOG(Converter, Error)
				<< "Could not query minimum selection crop: "
				<< strerror(-ret);
			return ret;
		}

		ret = getInputSelection(V4L2_SEL_TGT_CROP_BOUNDS, &maxCrop);
		if (ret) {
			LOG(Converter, Error)
				<< "Could not query maximum selection crop: "
				<< strerror(-ret);
			return ret;
		}

		/* Reset the crop to its maximum */
		ret = setInputSelection(V4L2_SEL_TGT_CROP, &maxCrop);
		if (ret) {
			LOG(Converter, Error)
				<< "Could not reset selection crop: "
				<< strerror(-ret);
			return ret;
		}

		inputCropBounds_ = { minCrop, maxCrop };
	}

	return 0;
}

int V4L2M2MConverter::V4L2M2MStream::exportBuffers(unsigned int count,
						   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return m2m_->capture()->exportBuffers(count, buffers);
}

int V4L2M2MConverter::V4L2M2MStream::start()
{
	int ret = m2m_->output()->importBuffers(inputBufferCount_);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->importBuffers(outputBufferCount_);
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = m2m_->output()->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = m2m_->capture()->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	return 0;
}

void V4L2M2MConverter::V4L2M2MStream::stop()
{
	m2m_->capture()->streamOff();
	m2m_->output()->streamOff();
	m2m_->capture()->releaseBuffers();
	m2m_->output()->releaseBuffers();
}

int V4L2M2MConverter::V4L2M2MStream::queueBuffers(FrameBuffer *input, FrameBuffer *output)
{
	int ret = m2m_->output()->queueBuffer(input);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->queueBuffer(output);
	if (ret < 0)
		return ret;

	return 0;
}

int V4L2M2MConverter::V4L2M2MStream::getInputSelection(unsigned int target, Rectangle *rect)
{
	return m2m_->output()->getSelection(target, rect);
}

int V4L2M2MConverter::V4L2M2MStream::setInputSelection(unsigned int target, Rectangle *rect)
{
	return m2m_->output()->setSelection(target, rect);
}

std::pair<Rectangle, Rectangle> V4L2M2MConverter::V4L2M2MStream::inputCropBounds()
{
	return inputCropBounds_;
}

std::string V4L2M2MConverter::V4L2M2MStream::logPrefix() const
{
	return stream_->configuration().toString();
}

void V4L2M2MConverter::V4L2M2MStream::outputBufferReady(FrameBuffer *buffer)
{
	auto it = converter_->queue_.find(buffer);
	if (it == converter_->queue_.end())
		return;

	if (!--it->second) {
		converter_->inputBufferReady.emit(buffer);
		converter_->queue_.erase(it);
	}
}

void V4L2M2MConverter::V4L2M2MStream::captureBufferReady(FrameBuffer *buffer)
{
	converter_->outputBufferReady.emit(buffer);
}

/* -----------------------------------------------------------------------------
 * V4L2M2MConverter
 */

/**
 * \class libcamera::V4L2M2MConverter
 * \brief The V4L2 M2M converter implements the converter interface based on
 * V4L2 M2M device.
*/

/**
 * \fn V4L2M2MConverter::V4L2M2MConverter
 * \brief Construct a V4L2M2MConverter instance
 * \param[in] media The media device implementing the converter
 */

V4L2M2MConverter::V4L2M2MConverter(MediaDevice *media)
	: Converter(media)
{
	if (deviceNode().empty())
		return;

	m2m_ = std::make_unique<V4L2M2MDevice>(deviceNode());
	int ret = m2m_->open();
	if (ret < 0) {
		m2m_.reset();
		return;
	}

	/* Discover Feature::InputCrop */
	Rectangle maxCrop;
	maxCrop.width = UINT_MAX;
	maxCrop.height = UINT_MAX;

	ret = m2m_->output()->setSelection(V4L2_SEL_TGT_CROP, &maxCrop);
	if (ret)
		return;

	/*
	 * Rectangles for cropping targets are defined even if the device
	 * does not support cropping. Their sizes and positions will be
	 * fixed in such cases.
	 *
	 * Set and inspect a crop equivalent to half of the maximum crop
	 * returned earlier. Use this to determine whether the crop on
	 * input is really supported.
	 */
	Rectangle halfCrop(maxCrop.size() / 2);
	ret = m2m_->output()->setSelection(V4L2_SEL_TGT_CROP, &halfCrop);
	if (!ret && halfCrop != maxCrop) {
		features_ |= Feature::InputCrop;

		LOG(Converter, Info)
			<< "Converter supports cropping on its input";
	}
}

/**
 * \fn libcamera::V4L2M2MConverter::loadConfiguration
 * \details \copydetails libcamera::Converter::loadConfiguration
 */

/**
 * \fn libcamera::V4L2M2MConverter::isValid
 * \details \copydetails libcamera::Converter::isValid
 */

/**
 * \fn libcamera::V4L2M2MConverter::formats
 * \details \copydetails libcamera::Converter::formats
 */
std::vector<PixelFormat> V4L2M2MConverter::formats(PixelFormat input)
{
	if (!m2m_)
		return {};

	/*
	 * Set the format on the input side (V4L2 output) of the converter to
	 * enumerate the conversion capabilities on its output (V4L2 capture).
	 */
	V4L2DeviceFormat v4l2Format;
	v4l2Format.fourcc = m2m_->output()->toV4L2PixelFormat(input);
	v4l2Format.size = { 1, 1 };

	int ret = m2m_->output()->setFormat(&v4l2Format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	if (v4l2Format.fourcc != m2m_->output()->toV4L2PixelFormat(input)) {
		LOG(Converter, Debug)
			<< "Input format " << input << " not supported.";
		return {};
	}

	std::vector<PixelFormat> pixelFormats;

	for (const auto &format : m2m_->capture()->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (pixelFormat)
			pixelFormats.push_back(pixelFormat);
	}

	return pixelFormats;
}

/**
 * \copydoc libcamera::Converter::sizes
 */
SizeRange V4L2M2MConverter::sizes(const Size &input)
{
	if (!m2m_)
		return {};

	/*
	 * Set the size on the input side (V4L2 output) of the converter to
	 * enumerate the scaling capabilities on its output (V4L2 capture).
	 */
	V4L2DeviceFormat format;
	format.fourcc = V4L2PixelFormat();
	format.size = input;

	int ret = m2m_->output()->setFormat(&format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	SizeRange sizes;

	format.size = { 1, 1 };
	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	sizes.min = format.size;

	format.size = { UINT_MAX, UINT_MAX };
	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(Converter, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	sizes.max = format.size;

	return sizes;
}

/**
 * \copydoc libcamera::Converter::strideAndFrameSize
 */
std::tuple<unsigned int, unsigned int>
V4L2M2MConverter::strideAndFrameSize(const PixelFormat &pixelFormat,
				     const Size &size)
{
	V4L2DeviceFormat format;
	format.fourcc = m2m_->capture()->toV4L2PixelFormat(pixelFormat);
	format.size = size;

	int ret = m2m_->capture()->tryFormat(&format);
	if (ret < 0)
		return std::make_tuple(0, 0);

	return std::make_tuple(format.planes[0].bpl, format.planes[0].size);
}

/**
 * \copydoc libcamera::Converter::configure
 */
int V4L2M2MConverter::configure(const StreamConfiguration &inputCfg,
				const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	int ret = 0;

	streams_.clear();

	for (unsigned int i = 0; i < outputCfgs.size(); ++i) {
		const StreamConfiguration &cfg = outputCfgs[i];
		std::unique_ptr<V4L2M2MStream> stream =
			std::make_unique<V4L2M2MStream>(this, cfg.stream());

		if (!stream->isValid()) {
			LOG(Converter, Error)
				<< "Failed to create stream " << i;
			ret = -EINVAL;
			break;
		}

		ret = stream->configure(inputCfg, cfg);
		if (ret < 0)
			break;

		streams_.emplace(cfg.stream(), std::move(stream));
	}

	if (ret < 0) {
		streams_.clear();
		return ret;
	}

	return 0;
}

/**
 * \copydoc libcamera::Converter::exportBuffers
 */
int V4L2M2MConverter::exportBuffers(const Stream *stream, unsigned int count,
				    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	auto iter = streams_.find(stream);
	if (iter == streams_.end())
		return -EINVAL;

	return iter->second->exportBuffers(count, buffers);
}

/**
 * \copydoc libcamera::Converter::setInputCrop
 */
int V4L2M2MConverter::setInputCrop(const Stream *stream, Rectangle *rect)
{
	if (!(features_ & Feature::InputCrop))
		return -ENOTSUP;

	auto iter = streams_.find(stream);
	if (iter == streams_.end()) {
		LOG(Converter, Error) << "Invalid output stream";
		return -EINVAL;
	}

	return iter->second->setInputSelection(V4L2_SEL_TGT_CROP, rect);
}

/**
 * \copydoc libcamera::Converter::inputCropBounds
 */
std::pair<Rectangle, Rectangle>
V4L2M2MConverter::inputCropBounds(const Stream *stream)
{
	auto iter = streams_.find(stream);
	if (iter == streams_.end())
		return {};

	return iter->second->inputCropBounds();
}

/**
 * \copydoc libcamera::Converter::start
 */
int V4L2M2MConverter::start()
{
	int ret;

	for (auto &iter : streams_) {
		ret = iter.second->start();
		if (ret < 0) {
			stop();
			return ret;
		}
	}

	return 0;
}

/**
 * \copydoc libcamera::Converter::stop
 */
void V4L2M2MConverter::stop()
{
	for (auto &iter : streams_)
		iter.second->stop();
}

/**
 * \copydoc libcamera::Converter::queueBuffers
 */
int V4L2M2MConverter::queueBuffers(FrameBuffer *input,
				   const std::map<const Stream *, FrameBuffer *> &outputs)
{
	std::set<FrameBuffer *> outputBufs;
	int ret;

	/*
	 * Validate the outputs as a sanity check: at least one output is
	 * required, all outputs must reference a valid stream and no two
	 * streams can reference same output framebuffers.
	 */
	if (outputs.empty())
		return -EINVAL;

	for (auto [stream, buffer] : outputs) {
		if (!buffer)
			return -EINVAL;

		outputBufs.insert(buffer);
	}

	if (outputBufs.size() != streams_.size())
		return -EINVAL;

	/* Queue the input and output buffers to all the streams. */
	for (auto [stream, buffer] : outputs) {
		ret = streams_.at(stream)->queueBuffers(input, buffer);
		if (ret < 0)
			return ret;
	}

	/*
	 * Add the input buffer to the queue, with the number of streams as a
	 * reference count. Completion of the input buffer will be signalled by
	 * the stream that releases the last reference.
	 */
	queue_.emplace(std::piecewise_construct,
		       std::forward_as_tuple(input),
		       std::forward_as_tuple(outputs.size()));

	return 0;
}

/*
 * \todo: This should be extended to include Feature::Flag to denote
 * what each converter supports feature-wise.
 */
static std::initializer_list<std::string> compatibles = {
	"mtk-mdp",
	"pxp",
};

REGISTER_CONVERTER("v4l2_m2m", V4L2M2MConverter, compatibles)

} /* namespace libcamera */
