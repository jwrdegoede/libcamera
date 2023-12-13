/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_linaro.cpp - software ISP implementation by Linaro
 */

#include "libcamera/internal/software_isp/swisp_linaro.h"

#include <math.h>
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
	: SoftwareIsp(name), ispWorker_(nullptr)
{
	std::unique_ptr<SwStatsCpu> stats;

	stats = std::make_unique<SwStatsCpu>();
	if (!stats || !stats->isValid()) {
		LOG(SoftwareIsp, Error) << "Failed to create SwStatsCPU object";
		return;
	}

	stats->statsReady.connect(this, &SwIspLinaro::statsReady);

	ispWorker_ = std::make_unique<SwIspLinaro::IspWorker>(this, std::move(stats));
	if (!ispWorker_) {
		LOG(SoftwareIsp, Error) << "Failed to create ISP worker";
		return;
	}

	ispWorker_->moveToThread(&ispWorkerThread_);
}

bool SwIspLinaro::isValid() const
{
	return !!ispWorker_;
}

void SwIspLinaro::IspWorker::debayerBGGR10PLine0(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src + stride_;

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 */
	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * BGBG line even pixel: RGR
		 *                       GBG
		 *                       RGR
		 * Write BGR
		 */
		*dst++ = blue_[curr[x]];
		*dst++ = green_[(prev[x] + curr[x - 2] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[(prev[x - 2] + prev[x + 1] + next[x - 2]  + next[x + 1]) / 4];
		x++;

		/*
		 * BGBG line odd pixel: GRG
		 *                      BGB
		 *                      GRG
		 * Write BGR
		 */
		*dst++ = blue_[(curr[x - 1] + curr[x + 1]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(prev[x] + next[x]) / 2];
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = blue_[curr[x]];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[(prev[x - 1] + prev[x + 1] + next[x - 1]  + next[x + 1]) / 4];
		x++;

		*dst++ = blue_[(curr[x - 1] + curr[x + 2]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(prev[x] + next[x]) / 2];
	}
}

void SwIspLinaro::IspWorker::debayerBGGR10PLine1(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src + stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * GRGR line even pixel: GBG
		 *                       RGR
		 *                       GBG
		 * Write BGR
		 */
		*dst++ = blue_[(prev[x] + next[x]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(curr[x - 2] + curr[x + 1]) / 2];
		x++;

		/*
		 * GRGR line odd pixel: BGB
		 *                      GRG
		 *                      BGB
		 * Write BGR
		 */
		*dst++ = blue_[(prev[x - 1] + prev[x + 1] + next[x - 1] + next[x + 1]) / 4];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[curr[x]];
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = blue_[(prev[x] + next[x]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(curr[x - 1] + curr[x + 1]) / 2];
		x++;

		*dst++ = blue_[(prev[x - 1] + prev[x + 2] + next[x - 1] + next[x + 2]) / 4];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 2] + next[x]) / 4];
		*dst++ = red_[curr[x]];
	}
}

void SwIspLinaro::IspWorker::debayerGBRG10PLine0(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src + stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * GBGB line even pixel: GRG
		 *                       BGB
		 *                       GRG
		 * Write BGR
		 */
		*dst++ = blue_[(curr[x - 2] + curr[x + 1]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(prev[x] + next[x]) / 2];
		x++;

		/*
		 * GBGB line odd pixel: RGR
		 *                      GBG
		 *                      RGR
		 * Write BGR
		 */
		*dst++ = blue_[curr[x]];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[(prev[x - 1] + prev[x + 1] + next[x - 1]  + next[x + 1]) / 4];
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = blue_[(curr[x - 1] + curr[x + 1]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(prev[x] + next[x]) / 2];
		x++;

		*dst++ = blue_[curr[x]];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 2] + next[x]) / 4];
		*dst++ = red_[(prev[x - 1] + prev[x + 2] + next[x - 1]  + next[x + 2]) / 4];
	}
}

void SwIspLinaro::IspWorker::debayerGBRG10PLine1(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src + stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * RGRG line even pixel: BGB
		 *                       GRG
		 *                       BGB
		 * Write BGR
		 */
		*dst++ = blue_[(prev[x - 2] + prev[x + 1] + next[x - 2] + next[x + 1]) / 4];
		*dst++ = green_[(prev[x] + curr[x - 2] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[curr[x]];
		x++;

		/*
		 * RGRG line odd pixel: GBG
		 *                      RGR
		 *                      GBG
		 * Write BGR
		 */
		*dst++ = blue_[(prev[x] + next[x]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(curr[x - 1] + curr[x + 1]) / 2];
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = blue_[(prev[x - 1] + prev[x + 1] + next[x - 1] + next[x + 1]) / 4];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 4];
		*dst++ = red_[curr[x]];
		x++;

		*dst++ = blue_[(prev[x] + next[x]) / 2];
		*dst++ = green_[curr[x]];
		*dst++ = red_[(curr[x - 1] + curr[x + 2]) / 2];
	}
}

SizeRange SwIspLinaro::IspWorker::outSizesRaw10P(const Size &inSize)
{
	if (inSize.width < 2 || inSize.height < 2) {
		LOG(SoftwareIsp, Error)
			<< "Input format size too small: " << inSize.toString();
		return {};
	}

	/*
	 * Debayering is done on 4x4 blocks because:
	 * 1. Some RGBI patterns repeat on a 4x4 basis
	 * 2. 10 bit packed bayer data packs 4 pixels in every 5 bytes
	 *
	 * For the width 1 extra column is needed for interpolation on each side
	 * and to keep the debayer code simple on the left side an entire block
	 * is skipped reducing the available width by 5 pixels.
	 *
	 * For the height 2 extra rows are needed for RGBI interpolation
	 * and to keep the debayer code simple on the top an entire block is
         * skipped reducing the available height by 6 pixels.
         *
         * As debayering is done in 4x4 blocks both must be a multiple of 4.
         */
	return SizeRange(Size(4, 4),
			 Size((inSize.width - 2 * 4) & ~(4 - 1),
			      (inSize.height - 2 * 4) & ~(4 - 1)),
			 4, 4);
}

unsigned int SwIspLinaro::IspWorker::outStrideRaw10P(const Size &outSize)
{
	return outSize.width * 3;
}

SwIspLinaro::IspWorker::IspWorker(SwIspLinaro *swIsp, std::unique_ptr<SwStatsCpu> stats)
	: swIsp_(swIsp), stats_(std::move(stats))
{
	debayerInfos_[formats::SBGGR10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerBGGR10PLine0,
						  &SwIspLinaro::IspWorker::debayerBGGR10PLine1,
						  NULL,
						  NULL,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGBRG10_CSI2P] = { formats::RGB888,
						  &SwIspLinaro::IspWorker::debayerGBRG10PLine0,
						  &SwIspLinaro::IspWorker::debayerGBRG10PLine1,
						  NULL,
						  NULL,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGRBG10_CSI2P] = { formats::RGB888,
						  /* GRBG is BGGR with the lines swapped */
						  &SwIspLinaro::IspWorker::debayerBGGR10PLine1,
						  &SwIspLinaro::IspWorker::debayerBGGR10PLine0,
						  NULL,
						  NULL,
						  &SwIspLinaro::IspWorker::outSizesRaw10P,
						  &SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SRGGB10_CSI2P] = { formats::RGB888,
						  /* RGGB is GBRG with the lines swapped */
						  &SwIspLinaro::IspWorker::debayerGBRG10PLine1,
						  &SwIspLinaro::IspWorker::debayerGBRG10PLine0,
						  NULL,
						  NULL,
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

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

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

	stats_->setWindow(Rectangle(0, 0, outWidth_, outHeight_));

	LOG(SoftwareIsp, Info)
		<< "SoftwareISP configuration: "
		<< inputCfg.size << "-" << inputCfg.pixelFormat << " -> "
		<< outputCfg.size << "-" << outputCfg.pixelFormat;

	const float gamma_correction = 0.5;

	/*
	 * Store gamma curve in green lookup and copy to red + blue
	 * until frame data collected
	 */
	for (int i = 0; i < 256; i++) {
		green_[i] = 255 * powf(i / 255.0, gamma_correction);
		red_[i] = green_[i];
		blue_[i] = green_[i];
	}

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

	stats_->startFrame();

	const uint8_t *src = in.planes()[0].data();
	uint8_t *dst = out.planes()[0].data();

	if (debayerInfo_->debayer2) {
		/* Skip first 4 lines for debayer interpolation purposes */
		src += stride_ * 4;
		for (unsigned int y = 0; y < outHeight_; y += 4) {
			stats_->processLine0(y, src, stride_);
			(this->*debayerInfo_->debayer0)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer1)(dst, src);
			src += stride_;
			dst += outStride_;

			stats_->processLine2(y, src, stride_);
			(this->*debayerInfo_->debayer2)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer3)(dst, src);
			src += stride_;
			dst += outStride_;
		}
	} else {
		/* Skip first 2 lines for debayer interpolation purposes */
		src += stride_ * 2;
		for (unsigned int y = 0; y < outHeight_; y += 2) {
			stats_->processLine0(y, src, stride_);
			(this->*debayerInfo_->debayer0)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer1)(dst, src);
			src += stride_;
			dst += outStride_;
		}
	}

	metadata.planes()[0].bytesused = out.planes()[0].size();

	stats_->finishFrame();

	/* calculate red and blue gains for simple AWB, FIXME move to IPA */
	struct SwIspStats stats = stats_->getStats();
	unsigned int rNumerat;
	unsigned int bNumerat;

	/* Clamp max gain at 4.0, this also avoids 0 division */
	if (stats.sumR_ <= stats.sumG_ / 4)
		rNumerat = 1024;
	else
		rNumerat = 256 * stats.sumG_ / stats.sumR_;

	if (stats.sumB_ <= stats.sumG_ / 4)
		bNumerat = 1024;
	else
		bNumerat = 256 * stats.sumG_ / stats.sumB_;

	for (int i = 0; i < 256; i++) {
		int idx;

		/* Use gamma curve stored in green lookup, apply gamma after gain! */
		idx = std::min({ i * rNumerat / 256U, 255U });
		red_[i] = green_[idx];

		idx = std::min({ i * bNumerat / 256U, 255U });
		blue_[i] = green_[idx];
	}

	swIsp_->outputBufferReady.emit(output);
	swIsp_->inputBufferReady.emit(input);
}

void SwIspLinaro::process(FrameBuffer *input, FrameBuffer *output)
{
	ispWorker_->invokeMethod(&SwIspLinaro::IspWorker::process,
				 ConnectionTypeQueued, input, output);
}

void SwIspLinaro::statsReady(int dummy)
{
	ispStatsReady.emit(dummy);
}

} /* namespace libcamera */
