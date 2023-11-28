/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_linaro.cpp - software ISP implementation by Linaro
 */

#include "libcamera/internal/software_isp/swisp_linaro.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(SoftwareIsp)

SwIspLinaro::SwIspLinaro(const std::string &name)
	: SoftwareIsp(name)
{
	ispWorker_ = std::make_unique<SwIspLinaro::IspWorker>(this);
	if (!ispWorker_) {
		LOG(SoftwareIsp, Error)
			<< "Failed to create ISP worker";
	} else {
		sharedStats_ = SharedMemObject<SwIspStats>("softIsp_stats");
		if (!sharedStats_.fd().isValid()) {
			LOG(SoftwareIsp, Error)
				<< "Failed to create shared memory for statistics";
			ispWorker_.reset();
		} else {
			ispWorker_->moveToThread(&ispWorkerThread_);
		}
	}
}

bool SwIspLinaro::isValid() const
{
	return !!ispWorker_;
}

void SwIspLinaro::IspWorker::debayerRaw10P(uint8_t *dst, const uint8_t *src)
{
	/* for brightness values in the 0 to 255 range: */
	static const unsigned int BRIGHT_LVL = 200U << 8;
	static const unsigned int TOO_BRIGHT_LVL = 240U << 8;

	static const unsigned int RED_Y_MUL = 77;	/* 0.30 * 256 */
	static const unsigned int GREEN_Y_MUL = 150;	/* 0.59 * 256 */
	static const unsigned int BLUE_Y_MUL = 29;	/* 0.11 * 256 */

	unsigned long sumR = 0;
	unsigned long sumB = 0;
	unsigned long sumG = 0;

	unsigned long bright_sum = 0;
	unsigned long too_bright_sum = 0;

	for (unsigned int y = 0; y < outHeight_; y++) {
		const uint8_t *pin_base = src + (y + 1) * stride_;
		uint8_t *pout = dst + y * outStride_;
		int phase_y = (y + redShift_.y) % 2;

		for (unsigned int x = 0; x < outWidth_; x++) {
			int phase_x = (x + redShift_.x) % 2;
			int phase = 2 * phase_y + phase_x;

			/* x part of the offset in the input buffer: */
			int x_m1 = x + x / 4;		/* offset for (x-1) */
			int x_0 = x + 1 + (x + 1) / 4;	/* offset for x */
			int x_p1 = x + 2 + (x + 2) / 4;	/* offset for (x+1) */
			/* the colour component value to write to the output */
			unsigned val;
			/* Y value times 256 */
			unsigned y_val;

			switch (phase) {
			case 0: /* at R pixel */
				/* blue: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				val = val * gNumerat_ / gDenomin_;
				y_val += GREEN_Y_MUL * val;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: (0,0) */
				val = *(pin_base + x_0);
				sumR += val;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 1: /* at Gr pixel */
				/* blue: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 2: /* at Gb pixel */
				/* blue: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			default: /* at B pixel */
				/* blue: (0,0) */
				val = *(pin_base + x_0);
				sumB += val;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
			}
		}
	}

	/* calculate the fractions of "bright" and "too bright" pixels */
	stats_.bright_ratio = (float)bright_sum / (outHeight_ * outWidth_);
	stats_.too_bright_ratio = (float)too_bright_sum / (outHeight_ * outWidth_);

	/* calculate red and blue gains for simple AWB */
	LOG(SoftwareIsp, Debug)
		<< "sumR = " << sumR << ", sumB = " << sumB << ", sumG = " << sumG;

	sumG /= 2; /* the number of G pixels is twice as big vs R and B ones */

	/* normalize red, blue, and green sums to fit into 22-bit value */
	unsigned long fRed = sumR / 0x400000;
	unsigned long fBlue = sumB / 0x400000;
	unsigned long fGreen = sumG / 0x400000;
	unsigned long fNorm = std::max({ 1UL, fRed, fBlue, fGreen });
	sumR /= fNorm;
	sumB /= fNorm;
	sumG /= fNorm;

	LOG(SoftwareIsp, Debug) << "fNorm = " << fNorm;
	LOG(SoftwareIsp, Debug)
		<< "Normalized: sumR = " << sumR
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

	LOG(SoftwareIsp, Debug)
		<< "rGain = [ " << rNumerat_ << " / " << rDenomin_
		<< " ], bGain = [ " << bNumerat_ << " / " << bDenomin_
		<< " ], gGain = [ " << gNumerat_ << " / " << gDenomin_
		<< " (minDenom = " << minDenom << ")";
}

SizeRange SwIspLinaro::IspWorker::outSizesRaw10P(const Size &inSize)
{
	if (inSize.width < 2 || inSize.height < 2) {
		LOG(SoftwareIsp, Error)
			<< "Input format size too small: " << inSize.toString();
		return {};
	}

	return SizeRange(Size(inSize.width - 2, inSize.height - 2));
}

unsigned int SwIspLinaro::IspWorker::outStrideRaw10P(const Size &outSize)
{
	return outSize.width * 3;
}

SwIspLinaro::IspWorker::IspWorker(SwIspLinaro *swIsp)
	: swIsp_(swIsp)
{
	debayerInfos_[formats::SBGGR10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerRaw10P,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGBRG10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerRaw10P,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGRBG10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerRaw10P,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SRGGB10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerRaw10P,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
}

int SwIspLinaro::IspWorker::setDebayerInfo(PixelFormat format)
{
	const auto it = debayerInfos_.find(format);
	if (it == debayerInfos_.end())
		return -1;

	debayerInfo_ = &it->second;
	return 0;
}

std::vector<PixelFormat> SwIspLinaro::IspWorker::formats(PixelFormat input)
{
	std::vector<PixelFormat> pixelFormats;

	const auto it = debayerInfos_.find(input);
	if (it == debayerInfos_.end())
		LOG(SoftwareIsp, Info)
			<< "Unsupported input format " << input.toString();
	else
		pixelFormats.push_back(it->second.outPixelFmt);

	return pixelFormats;
}

SizeRange SwIspLinaro::IspWorker::sizes(PixelFormat inputFormat,
					const Size &inputSize)
{
	const auto it = debayerInfos_.find(inputFormat);
	if (it == debayerInfos_.end()) {
		LOG(SoftwareIsp, Info)
			<< "Unsupported input format " << inputFormat.toString();
		return {};
	}

	return (*it->second.getOutSizes)(inputSize);
}

unsigned int SwIspLinaro::IspWorker::outStride(const PixelFormat &outputFormat,
					       const Size &outSize)
{
	/*
	 * Assuming that the output stride depends only on the outputFormat,
	 * we use the first debayerInfos_ entry with the matching output format
	 */
	for (auto it = debayerInfos_.begin(); it != debayerInfos_.end(); it++) {
		if (it->second.outPixelFmt == outputFormat)
			return (*it->second.getOutStride)(outSize);
	}

	return 0;
}

int SwIspLinaro::IspWorker::configure(const StreamConfiguration &inputCfg,
				      const StreamConfiguration &outputCfg)
{
	if (setDebayerInfo(inputCfg.pixelFormat) != 0) {
		LOG(SoftwareIsp, Error)
			<< "Input format " << inputCfg.pixelFormat
			<< "not supported";
		return -EINVAL;
	}

	/* check that:
	 * - output format is valid
	 * - output size matches the input size and is valid */
	SizeRange outSizeRange = (*debayerInfo_->getOutSizes)(inputCfg.size);
	if (debayerInfo_->outPixelFmt != outputCfg.pixelFormat ||
	    outputCfg.size.isNull() || !outSizeRange.contains(outputCfg.size) ||
	    (*debayerInfo_->getOutStride)(outputCfg.size) != outputCfg.stride) {
		LOG(SoftwareIsp, Error)
			<< "Invalid output format/size/stride: "
			<< "\n  " << outputCfg.pixelFormat << " ("
			<< debayerInfo_->outPixelFmt << ")"
			<< "\n  " << outputCfg.size << " ("
			<< outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " ("
			<< (*debayerInfo_->getOutStride)(outputCfg.size) << ")";
		return -EINVAL;
	}

	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;
	stride_ = inputCfg.stride;

	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);
	switch (bayerFormat.order) {
	case BayerFormat::BGGR:
		redShift_ = Point(0, 0);
		break;
	case BayerFormat::GBRG:
		redShift_ = Point(1, 0);
		break;
	case BayerFormat::GRBG:
		redShift_ = Point(0, 1);
		break;
	case BayerFormat::RGGB:
	default:
		redShift_ = Point(1, 1);
		break;
	}

	outStride_ = outputCfg.stride;
	outWidth_  = outputCfg.size.width;
	outHeight_ = outputCfg.size.height;

	LOG(SoftwareIsp, Info)
		<< "SoftwareISP configuration: "
		<< inputCfg.size << "-" << inputCfg.pixelFormat << " -> "
		<< outputCfg.size << "-" << outputCfg.pixelFormat;

	/* set r/g/b gains to 1.0 until frame data collected */
	rNumerat_ = rDenomin_ = 1;
	bNumerat_ = bDenomin_ = 1;
	gNumerat_ = gDenomin_ = 1;

	return 0;
}

/* May not be called before SwIspLinaro::IspWorker::configure() */
unsigned int SwIspLinaro::IspWorker::outBufferSize()
{
	return outHeight_ * outStride_;
}

std::vector<PixelFormat> SwIspLinaro::formats(PixelFormat inputFormat)
{
	ASSERT(ispWorker_ != nullptr);

	return ispWorker_->formats(inputFormat);
}

SizeRange SwIspLinaro::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	ASSERT(ispWorker_ != nullptr);

	return ispWorker_->sizes(inputFormat, inputSize);
}

std::tuple<unsigned int, unsigned int>
SwIspLinaro::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	ASSERT(ispWorker_ != nullptr);

	unsigned int stride = ispWorker_->outStride(outputFormat, size);

	return std::make_tuple(stride, stride * size.height);
}

int SwIspLinaro::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	ASSERT(ispWorker_ != nullptr);

	if (outputCfgs.size() != 1) {
		LOG(SoftwareIsp, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	return ispWorker_->configure(inputCfg, outputCfgs[0]);
}

int SwIspLinaro::exportBuffers(unsigned int output, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	ASSERT(ispWorker_ != nullptr);

	/* single output for now */
	if (output >= 1)
		return -EINVAL;

	unsigned int bufSize = ispWorker_->outBufferSize();

	/* TODO: allocate from dma_heap; memfd buffs aren't allowed in FrameBuffer */
	for (unsigned int i = 0; i < count; i++) {
		std::string name = "frame-" + std::to_string(i);

		const int ispFd = memfd_create(name.c_str(), 0);
		int ret = ftruncate(ispFd, bufSize);
		if (ret < 0) {
			LOG(SoftwareIsp, Error)
				<< "ftruncate() for memfd failed "
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

int SwIspLinaro::queueBuffers(FrameBuffer *input,
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

int SwIspLinaro::start()
{
	ispWorkerThread_.start();
	return 0;
}

void SwIspLinaro::stop()
{
	ispWorkerThread_.exit();
	ispWorkerThread_.wait();
}

void SwIspLinaro::IspWorker::process(FrameBuffer *input, FrameBuffer *output)
{
	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(SoftwareIsp, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		swIsp_->outputBufferReady.emit(output);
		swIsp_->inputBufferReady.emit(input);
		return;
	}

	(this->*debayerInfo_->debayer)(out.planes()[0].data(), in.planes()[0].data());
	metadata.planes()[0].bytesused = out.planes()[0].size();

	*swIsp_->sharedStats_ = stats_;
	swIsp_->ispStatsReady.emit(0);

	swIsp_->outputBufferReady.emit(output);
	swIsp_->inputBufferReady.emit(input);
}

void SwIspLinaro::process(FrameBuffer *input, FrameBuffer *output)
{
	ispWorker_->invokeMethod(&SwIspLinaro::IspWorker::process,
				 ConnectionTypeQueued, input, output);
}

} /* namespace libcamera */
