/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * soft_simple.cpp - Simple Software Image Processing Algorithm module
 */

#include <sys/mman.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

#include "common/soft_base.h"

#define EXPOSURE_OPTIMAL_VALUE 2.5
#define EXPOSURE_SATISFACTORY_OFFSET 0.2

namespace libcamera {

LOG_DECLARE_CATEGORY(IPASoft)

namespace ipa::soft {

class IPASoftSimple final : public IPASoftBase
{
public:
	IPASoftSimple()
		: IPASoftBase(), ignore_updates_(0)
	{
	}

	~IPASoftSimple()
	{
		if (stats_)
			munmap(stats_, sizeof(SwIspStats));
		if (params_)
			munmap(params_, sizeof(DebayerParams));
	}

	int platformInit(const ControlInfoMap &sensorInfoMap) override;
	int platformConfigure(const ControlInfoMap &sensorInfoMap) override;
	int platformStart() override;
	void platformStop() override;
	void platformProcessStats(const ControlList &sensorControls) override;

private:
	void update_exposure(double exposuremsv);

	DebayerParams *params_;
	SwIspStats *stats_;
	int exposure_min_, exposure_max_;
	int again_min_, again_max_;
	int again_, exposure_;
	int ignore_updates_;
};

int IPASoftSimple::platformInit(const ControlInfoMap &sensorInfoMap)
{
	params_ = static_cast<DebayerParams *>(mmap(nullptr, sizeof(DebayerParams),
						    PROT_WRITE, MAP_SHARED,
						    fdParams_.get(), 0));
	if (!params_) {
		LOG(IPASoft, Error) << "Unable to map Parameters";
		return -ENODEV;
	}

	stats_ = static_cast<SwIspStats *>(mmap(nullptr, sizeof(SwIspStats),
						PROT_READ, MAP_SHARED,
						fdStats_.get(), 0));
	if (!stats_) {
		LOG(IPASoft, Error) << "Unable to map Statistics";
		return -ENODEV;
	}

	if (sensorInfoMap.find(V4L2_CID_EXPOSURE) == sensorInfoMap.end()) {
		LOG(IPASoft, Error) << "Don't have exposure control";
		return -EINVAL;
	}

	if (sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN) == sensorInfoMap.end()) {
		LOG(IPASoft, Error) << "Don't have gain control";
		return -EINVAL;
	}

	return 0;
}

int IPASoftSimple::platformConfigure(const ControlInfoMap &sensorInfoMap)
{
	const ControlInfo &exposure_info = sensorInfoMap.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gain_info = sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN)->second;

	exposure_min_ = exposure_info.min().get<int>();
	if (!exposure_min_) {
		LOG(IPASoft, Warning) << "Minimum exposure is zero, that can't be linear";
		exposure_min_ = 1;
	}
	exposure_max_ = exposure_info.max().get<int>();
	again_min_ = gain_info.min().get<int>();
	if (!again_min_) {
		LOG(IPASoft, Warning) << "Minimum gain is zero, that can't be linear";
		again_min_ = 100;
	}
	again_max_ = gain_info.max().get<int>();

	LOG(IPASoft, Info) << "Exposure " << exposure_min_ << "-" << exposure_max_
			   << ", gain " << again_min_ << "-" << again_max_;

	return 0;
}

int IPASoftSimple::platformStart()
{
	return 0;
}

void IPASoftSimple::platformStop()
{
}

void IPASoftSimple::platformProcessStats(const ControlList &sensorControls)
{
	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	if (stats_->sumR_ <= stats_->sumG_ / 4)
		params_->gainR = 1024;
	else
		params_->gainR = 256 * stats_->sumG_ / stats_->sumR_;

	if (stats_->sumB_ <= stats_->sumG_ / 4)
		params_->gainB = 1024;
	else
		params_->gainB = 256 * stats_->sumG_ / stats_->sumB_;

	/* Green gain and gamma values are fixed */
	params_->gainG = 256;
	params_->gamma = 0.5;

	setIspParams.emit(0);

	/*
	 * AE / AGC, use 2 frames delay to make sure that the exposure and
	 * the gain set have applied to the camera sensor.
	 */
	if (ignore_updates_ > 0) {
		--ignore_updates_;
		return;
	}

	unsigned int denom = 0;
	unsigned int num = 0;
	unsigned int y_histogramSmall[5] = {};

	for (int i = 0; i < 16; i++)
		y_histogramSmall[(i - i / 8) / 3] += stats_->y_histogram[i];

	for (int i = 0; i < 5; i++) {
		LOG(IPASoft, Debug) << i << ": " << y_histogramSmall[i];
		denom += y_histogramSmall[i];
		num += y_histogramSmall[i] * (i + 1);
	}

	float exposuremsv = (float)num / denom;

	/* sanity check */
	if (!sensorControls.contains(V4L2_CID_EXPOSURE) ||
	    !sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
		LOG(IPASoft, Error) << "Control(s) missing";
		return;
	}

	ControlList ctrls(sensorControls);

	exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int>();
	again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int>();

	update_exposure(exposuremsv);

	ctrls.set(V4L2_CID_EXPOSURE, exposure_);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

	ignore_updates_ = 2;

	setSensorControls.emit(ctrls);

	LOG(IPASoft, Debug) << "exposuremsv " << exposuremsv
			    << " exp " << exposure_ << " again " << again_
			    << " gain R/B " << params_->gainR << "/" << params_->gainB;
}

/* DENOMINATOR of 10 gives ~10% increment/decrement; DENOMINATOR of 5 - about ~20% */
#define DENOMINATOR 10
#define UP_NUMERATOR (DENOMINATOR + 1)
#define DOWN_NUMERATOR (DENOMINATOR - 1)

void IPASoftSimple::update_exposure(double exposuremsv)
{
	int next;

	if (exposuremsv < EXPOSURE_OPTIMAL_VALUE - EXPOSURE_SATISFACTORY_OFFSET) {
		next = exposure_ * UP_NUMERATOR / DENOMINATOR;
		if (next - exposure_ < 1)
			exposure_ += 1;
		else
			exposure_ = next;
		if (exposure_ >= exposure_max_) {
			next = again_ * UP_NUMERATOR / DENOMINATOR;
			if (next - again_ < 1)
				again_ += 1;
			else
				again_ = next;
		}
	}

	if (exposuremsv > EXPOSURE_OPTIMAL_VALUE + EXPOSURE_SATISFACTORY_OFFSET) {
		if (exposure_ == exposure_max_ && again_ != again_min_) {
			next = again_ * DOWN_NUMERATOR / DENOMINATOR;
			if (again_ - next < 1)
				again_ -= 1;
			else
				again_ = next;
		} else {
			next = exposure_ * DOWN_NUMERATOR / DENOMINATOR;
			if (exposure_ - next < 1)
				exposure_ -= 1;
			else
				exposure_ = next;
		}
	}

	if (exposure_ > exposure_max_)
		exposure_ = exposure_max_;
	else if (exposure_ < exposure_min_)
		exposure_ = exposure_min_;

	if (again_ > again_max_)
		again_ = again_max_;
	else if (again_ < again_min_)
		again_ = again_min_;
}

} /* namespace ipa::soft */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"SimplePipelineHandler",
	"soft/simple",
};

IPAInterface *ipaCreate()
{
	return new ipa::soft::IPASoftSimple();
}

} /* extern "C" */

} /* namespace libcamera */
