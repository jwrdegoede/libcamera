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
	: SoftwareIsp(name), debayer_(nullptr), debayerParams_{256, 256, 0.5f}
{
	std::unique_ptr<SwStatsCpu> stats;

	stats = std::make_unique<SwStatsCpu>();
	if (!stats || !stats->isValid()) {
		LOG(SoftwareIsp, Error) << "Failed to create SwStatsCpu object";
		return;
	}

	stats->statsReady.connect(this, &SwIspLinaro::statsReady);

	debayer_ = std::make_unique<DebayerCpu>(std::move(stats));
	if (!debayer_) {
		LOG(SoftwareIsp, Error) << "Failed to create DebayerCpu object";
		return;
	}

	debayer_->inputBufferReady.connect(this, &SwIspLinaro::inputReady);
	debayer_->outputBufferReady.connect(this, &SwIspLinaro::outputReady);
	debayer_->moveToThread(&ispWorkerThread_);
}

bool SwIspLinaro::isValid() const
{
	return !!debayer_;
}

std::vector<PixelFormat> SwIspLinaro::formats(PixelFormat inputFormat)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->formats(inputFormat);
}

SizeRange SwIspLinaro::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->sizes(inputFormat, inputSize);
}

std::tuple<unsigned int, unsigned int>
SwIspLinaro::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->strideAndFrameSize(outputFormat, size);
}

int SwIspLinaro::exportBuffers(unsigned int output, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	ASSERT(debayer_ != nullptr);

	/* single output for now */
	if (output >= 1)
		return -EINVAL;

	/* TODO: allocate from dma_heap; memfd buffs aren't allowed in FrameBuffer */
	for (unsigned int i = 0; i < count; i++) {
		std::string name = "frame-" + std::to_string(i);

		const int ispFd = memfd_create(name.c_str(), 0);
		int ret = ftruncate(ispFd, debayer_->frameSize());
		if (ret < 0) {
			LOG(SoftwareIsp, Error)
				<< "ftruncate() for memfd failed "
				<< strerror(-ret);
			return ret;
		}

		FrameBuffer::Plane outPlane;
		outPlane.fd = SharedFD(std::move(ispFd));
		outPlane.offset = 0;
		outPlane.length = debayer_->frameSize();

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

void SwIspLinaro::process(FrameBuffer *input, FrameBuffer *output)
{
	debayer_->invokeMethod(&DebayerCpu::process,
			       ConnectionTypeQueued, input, output, debayerParams_);
}

void SwIspLinaro::statsReady(int dummy)
{
	ispStatsReady.emit(dummy);
}

void SwIspLinaro::inputReady(FrameBuffer *input)
{
	inputBufferReady.emit(input);
}

void SwIspLinaro::outputReady(FrameBuffer *output)
{
	outputBufferReady.emit(output);
}

} /* namespace libcamera */
