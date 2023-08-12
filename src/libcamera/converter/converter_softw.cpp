/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * converter_softw.h - interface of software converter (runs 100% on CPU)
 */

#include "libcamera/internal/converter/converter_softw.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Converter)

std::vector<PixelFormat> SwConverter::formats(PixelFormat input)
{
	std::vector<PixelFormat> pixelFormats;
	BayerFormat inputFormat = BayerFormat::fromPixelFormat(input);

	/* Only RAW10P is currently supported */
	if (inputFormat.bitDepth == 10 && inputFormat.packing == BayerFormat::Packing::CSI2)
		pixelFormats.push_back(formats::RGB888);

	if (pixelFormats.empty())
		LOG(Converter, Info)
			<< "Unsupported input format " << input.toString();

	return pixelFormats;
}

SizeRange SwConverter::sizes(const Size &input)
{
	if (input.width < 2 || input.height < 2) {
		LOG(Converter, Error)
			<< "Input format size too small: " << input.toString();
		return {};
	}

	return SizeRange(Size(input.width - 2, input.height - 2));
}

std::tuple<unsigned int, unsigned int>
SwConverter::strideAndFrameSize(const PixelFormat &pixelFormat,
				const Size &size)
{
	/* Only RGB888 output is currently supported */
	if (pixelFormat == formats::RGB888) {
		int stride = size.width * 3;
		return std::make_tuple(stride, stride * (size.height));
	}

	return std::make_tuple(0, 0);
}

int SwConverter::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	if (outputCfgs.size() != 1) {
		LOG(Converter, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	isp_ = std::make_unique<SwConverter::Isp>(this);

	return isp_->configure(inputCfg, outputCfgs[0]);
}

int SwConverter::Isp::configure(const StreamConfiguration &inputCfg,
				const StreamConfiguration &outputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);
	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;
	stride_ = inputCfg.stride;

	if (bayerFormat.bitDepth != 10 ||
	    bayerFormat.packing != BayerFormat::Packing::CSI2 ||
	    width_ < 2 || height_ < 2) {
		LOG(Converter, Error) << "Input format "
				      << inputCfg.size << "-"
				      << inputCfg.pixelFormat
				      << "not supported";
		return -EINVAL;
	}

	switch (bayerFormat.order) {
	case BayerFormat::BGGR:
		red_shift_ = Point(0, 0);
		break;
	case BayerFormat::GBRG:
		red_shift_ = Point(1, 0);
		break;
	case BayerFormat::GRBG:
		red_shift_ = Point(0, 1);
		break;
	case BayerFormat::RGGB:
	default:
		red_shift_ = Point(1, 1);
		break;
	}

	if (outputCfg.size.width != width_ - 2 ||
	    outputCfg.size.height != height_ - 2 ||
	    outputCfg.stride != (width_ - 2) * 3 ||
	    outputCfg.pixelFormat != formats::RGB888) {
		LOG(Converter, Error)
			<< "Output format not supported";
		return -EINVAL;
	}

	LOG(Converter, Info) << "SwConverter configuration: "
			     << inputCfg.size << "-" << inputCfg.pixelFormat
			     << " -> "
			     << outputCfg.size << "-" << outputCfg.pixelFormat;

	/* set r/g/b gains to 1.0 until frame data collected */
	rNumerat_ = rDenomin_ = 1;
	bNumerat_ = bDenomin_ = 1;
	gNumerat_ = gDenomin_ = 1;

	return 0;
}

int SwConverter::exportBuffers(unsigned int output, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* single output for now */
	if (output >= 1)
		return -EINVAL;

	return isp_->exportBuffers(count, buffers);
}

int SwConverter::Isp::exportBuffers(unsigned int count,
				    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* V4L2_PIX_FMT_BGR24 aka 'BGR3' for output: */
	unsigned int bufSize = (height_ - 2) * (width_ - 2) * 3;

	for (unsigned int i = 0; i < count; i++) {
		std::string name = "frame-" + std::to_string(i);

		const int ispFd = memfd_create(name.c_str(), 0);
		int ret = ftruncate(ispFd, bufSize);
		if (ret < 0) {
			LOG(Converter, Error) << "ftruncate() for memfd failed "
					      << strerror(-ret);
			return ret;
		}

		FrameBuffer::Plane outPlane;
		outPlane.fd = SharedFD(std::move(ispFd));
		outPlane.offset = 0;
		outPlane.length = bufSize;

		std::vector<FrameBuffer::Plane> planes{ outPlane };
		buffers->emplace_back(std::make_unique<FrameBuffer>(std::move(planes)));
	}

	return count;
}

int SwConverter::Isp::start()
{
	moveToThread(&thread_);
	thread_.start();
	return 0;
}

void SwConverter::Isp::stop()
{
	thread_.exit();
	thread_.wait();
}

int SwConverter::start()
{
	return isp_->start();
}

void SwConverter::stop()
{
	return isp_->stop();
}

int SwConverter::queueBuffers(FrameBuffer *input,
			      const std::map<unsigned int, FrameBuffer *> &outputs)
{
	unsigned int mask = 0;

	/*
	 * Validate the outputs as a sanity check: at least one output is
	 * required, all outputs must reference a valid stream and no two
	 * outputs can reference the same stream.
	 */
	if (outputs.empty())
		return -EINVAL;

	for (auto [index, buffer] : outputs) {
		if (!buffer)
			return -EINVAL;
		if (index >= 1) /* only single stream atm */
			return -EINVAL;
		if (mask & (1 << index))
			return -EINVAL;

		mask |= 1 << index;
	}

	process(input, outputs.at(0));

	return 0;
}

void SwConverter::process(FrameBuffer *input, FrameBuffer *output)
{
	isp_->invokeMethod(&SwConverter::Isp::process,
			   ConnectionTypeQueued, input, output);
}

void SwConverter::Isp::process(FrameBuffer *input, FrameBuffer *output)
{
	const uint8_t *rawData;
	uint8_t *rgbData;

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	const FrameBuffer::Plane &plane = input->planes()[0];
	/* TODO: use MappedBuffer class for mapping the frame buffers */
	rawData = (const uint8_t *)mmap(NULL, plane.length, PROT_READ,
					MAP_SHARED, plane.fd.get(), 0);
	const FrameBuffer::Plane &rgbPlane = output->planes()[0];
	rgbData = (uint8_t *)mmap(NULL, rgbPlane.length, PROT_READ | PROT_WRITE,
				  MAP_SHARED, rgbPlane.fd.get(), 0);
	LOG(Converter, Debug) << "raw length = " << plane.length
			      << ", rgb length = " << rgbPlane.length;
	if (rawData == MAP_FAILED || rgbData == MAP_FAILED) {
		LOG(Converter, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		converter_->outputBufferReady.emit(output);
		converter_->inputBufferReady.emit(input);
		return;
	}

	debayer(rgbData, rawData);

	metadata.planes()[0].bytesused = rgbPlane.length;

	converter_->outputBufferReady.emit(output);
	converter_->inputBufferReady.emit(input);
}

void SwConverter::Isp::debayer(uint8_t *dst, const uint8_t *src)
{
	/* RAW10P input format is assumed */

	/* output buffer is in BGR24 format and is of (width-2)*(height-2) */

	int w_out = width_ - 2;
	int h_out = height_ - 2;

	unsigned long sumR = 0;
	unsigned long sumB = 0;
	unsigned long sumG = 0;

	for (int y = 0; y < h_out; y++) {
		const uint8_t *pin_base = src + (y + 1) * stride_;
		uint8_t *pout = dst + y * w_out * 3;
		int phase_y = (y + red_shift_.y) % 2;

		for (int x = 0; x < w_out; x++) {
			int phase_x = (x + red_shift_.x) % 2;
			int phase = 2 * phase_y + phase_x;

			/* x part of the offset in the input buffer: */
			int x_m1 = x + x / 4;		/* offset for (x-1) */
			int x_0 = x + 1 + (x + 1) / 4;	/* offset for x */
			int x_p1 = x + 2 + (x + 2) / 4;	/* offset for (x+1) */
			/* the colour component value to write to the output */
			unsigned val;

			switch (phase) {
			case 0: /* at R pixel */
				/* blue: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: (0,0) */
				val = *(pin_base + x_0);
				sumR += val;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 1: /* at Gr pixel */
				/* blue: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 2: /* at Gb pixel */
				/* blue: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			default: /* at B pixel */
				/* blue: (0,0) */
				val = *(pin_base + x_0);
				sumB += val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
			}
		}
	}

	/* calculate red and blue gains for simple AWB */
	LOG(Converter, Debug) << "sumR = " << sumR
			      << ", sumB = " << sumB << ", sumG = " << sumG;

	sumG /= 2; /* the number of G pixels is twice as big vs R and B ones */

	/* normalize red, blue, and green sums to fit into 22-bit value */
	unsigned long fRed = sumR / 0x400000;
	unsigned long fBlue = sumB / 0x400000;
	unsigned long fGreen = sumG / 0x400000;
	unsigned long fNorm = std::max({ 1UL, fRed, fBlue, fGreen });
	sumR /= fNorm;
	sumB /= fNorm;
	sumG /= fNorm;

	LOG(Converter, Debug) << "fNorm = " << fNorm;
	LOG(Converter, Debug) << "Normalized: sumR = " << sumR
			      << ", sumB= " << sumB << ", sumG = " << sumG;

	/* make sure red/blue gains never exceed approximately 256 */
	unsigned long minDenom;
	rNumerat_ = (sumR + sumB + sumG) / 3;
	minDenom = rNumerat_ / 0x100;
	rDenomin_ = std::max(minDenom, sumR);
	bNumerat_ = rNumerat_;
	bDenomin_ = std::max(minDenom, sumB);
	gNumerat_ = rNumerat_;
	gDenomin_ = std::max(minDenom, sumG);

	LOG(Converter, Debug) << "rGain = [ "
			      << rNumerat_ << " / " << rDenomin_
			      << " ], bGain = [ " << bNumerat_ << " / " << bDenomin_
			      << " ], gGain = [ " << gNumerat_ << " / " << gDenomin_
			      << " (minDenom = " << minDenom << ")";
}

} /* namespace libcamera */
