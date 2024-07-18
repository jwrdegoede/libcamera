/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * Simple builtin benchmark to measure software ISP processing times
 */

#include "libcamera/internal/software_isp/benchmark.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(Benchmark)

/**
 * \class Benchmark
 * \brief Simple builtin benchmark
 *
 * Simple builtin benchmark to measure software ISP processing times.
 */

/**
 * \brief Constructs a Benchmark object
 */
Benchmark::Benchmark()
	: measuredFrames_(0), frameProcessTime_(0)
{
}

Benchmark::~Benchmark()
{
}

static inline int64_t timeDiff(timespec &after, timespec &before)
{
	return (after.tv_sec - before.tv_sec) * 1000000000LL +
	       (int64_t)after.tv_nsec - (int64_t)before.tv_nsec;
}

/**
 * \brief Start measuring process time for a single frame
 *
 * Call this function before processing frame data to start measuring
 * the process time for a frame.
 */
void Benchmark::startFrame(void)
{
	if (measuredFrames_ >= Benchmark::kLastFrameToMeasure)
		return;

	frameStartTime_ = {};
	clock_gettime(CLOCK_MONOTONIC_RAW, &frameStartTime_);
}

/**
 * \brief Finish measuring process time for a single frame
 *
 * Call this function after processing frame data to finish measuring
 * the process time for a frame.
 *
 * This function will log frame processing time information after
 * Benchmark::kLastFrameToMeasure frames have been processed.
 */
void Benchmark::finishFrame(void)
{
	if (measuredFrames_ >= Benchmark::kLastFrameToMeasure)
		return;

	measuredFrames_++;

	if (measuredFrames_ <= Benchmark::kFramesToSkip)
		return;

	timespec frameEndTime = {};
	clock_gettime(CLOCK_MONOTONIC_RAW, &frameEndTime);
	frameProcessTime_ += timeDiff(frameEndTime, frameStartTime_);

	if (measuredFrames_ == Benchmark::kLastFrameToMeasure) {
		const unsigned int measuredFrames = Benchmark::kLastFrameToMeasure -
						    Benchmark::kFramesToSkip;
		LOG(Benchmark, Info)
			<< "Processed " << measuredFrames
			<< " frames in " << frameProcessTime_ / 1000 << "us, "
			<< frameProcessTime_ / (1000 * measuredFrames)
			<< " us/frame";
	}
}

} /* namespace libcamera */
