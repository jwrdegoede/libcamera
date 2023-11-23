/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_simple.cpp - Simple software ISP implementation
 */

#include "libcamera/internal/software_isp/swisp_simple.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

SwIspSimple::SwIspSimple(PipelineHandler *pipe, const ControlInfoMap &sensorControls)
	: SoftwareIsp(pipe, sensorControls), debayer_(nullptr), debayerParams_{256, 256, 256, 0.5f}
{
	sharedParams_ = SharedMemObject<DebayerParams>("softIsp_params");
	if (!sharedParams_.fd().isValid()) {
		LOG(SoftwareIsp, Error) << "Failed to create shared memory for parameters";
		return;
	}

	std::unique_ptr<SwStatsCpu> stats;

	stats = std::make_unique<SwStatsCpu>();
	if (!stats) {
		LOG(SoftwareIsp, Error) << "Failed to create SwStatsCpu object";
		return;
	}

	stats->statsReady.connect(this, &SwIspSimple::statsReady);

	debayer_ = std::make_unique<DebayerCpu>(std::move(stats));
	if (!debayer_) {
		LOG(SoftwareIsp, Error) << "Failed to create DebayerCpu object";
		return;
	}

	debayer_->inputBufferReady.connect(this, &SwIspSimple::inputReady);
	debayer_->outputBufferReady.connect(this, &SwIspSimple::outputReady);

	ipa_ = IPAManager::createIPA<ipa::soft::IPAProxySoft>(pipe, 0, 0);
	if (!ipa_) {
		LOG(SoftwareIsp, Error)
			<< "Creating IPA for software ISP failed";
		debayer_.reset();
		return;
	}

	int ret = ipa_->init(IPASettings{ "No cfg file", "No sensor model" },
			     debayer_->getStatsFD(),
			     sharedParams_.fd(),
			     sensorControls);
	if (ret) {
		LOG(SoftwareIsp, Error) << "IPA init failed";
		debayer_.reset();
		return;
	}

	ipa_->configure(sensorControls);

	ipa_->setIspParams.connect(this, &SwIspSimple::saveIspParams);

	debayer_->moveToThread(&ispWorkerThread_);
}

void SwIspSimple::processStats(const ControlList &sensorControls)
{
	ASSERT(ipa_);
	ipa_->processStats(sensorControls);
}

Signal<const ControlList &> &SwIspSimple::getSignalSetSensorControls()
{
	ASSERT(ipa_);
	return ipa_->setSensorControls;
}

bool SwIspSimple::isValid() const
{
	return !!debayer_;
}

std::vector<PixelFormat> SwIspSimple::formats(PixelFormat inputFormat)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->formats(inputFormat);
}

SizeRange SwIspSimple::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->sizes(inputFormat, inputSize);
}

std::tuple<unsigned int, unsigned int>
SwIspSimple::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->strideAndFrameSize(outputFormat, size);
}

int SwIspSimple::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	ASSERT(debayer_ != nullptr);

	return debayer_->configure(inputCfg, outputCfgs);
}

int SwIspSimple::exportBuffers(unsigned int output, unsigned int count,
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

int SwIspSimple::queueBuffers(FrameBuffer *input,
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

int SwIspSimple::start()
{
	int ret = ipa_->start();
	if (ret)
		return ret;

	ispWorkerThread_.start();
	return 0;
}

void SwIspSimple::stop()
{
	ispWorkerThread_.exit();
	ispWorkerThread_.wait();

	ipa_->stop();
}

void SwIspSimple::process(FrameBuffer *input, FrameBuffer *output)
{
	debayer_->invokeMethod(&DebayerCpu::process,
			       ConnectionTypeQueued, input, output, debayerParams_);
}

void SwIspSimple::saveIspParams([[maybe_unused]] int dummy)
{
	debayerParams_ = *sharedParams_;
}

void SwIspSimple::statsReady(int dummy)
{
	ispStatsReady.emit(dummy);
}

void SwIspSimple::inputReady(FrameBuffer *input)
{
	inputBufferReady.emit(input);
}

void SwIspSimple::outputReady(FrameBuffer *output)
{
	outputBufferReady.emit(output);
}

REGISTER_SOFTWAREISP(SwIspSimple)

} /* namespace libcamera */
