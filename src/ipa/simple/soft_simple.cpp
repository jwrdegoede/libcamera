/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * soft_simple.cpp - Simple Software Image Processing Algorithm module
 */

#include <sys/mman.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/soft_ipa_interface.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

#include "black_level.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoft)

namespace ipa::soft {

class IPASoftSimple : public ipa::soft::IPASoftInterface
{
public:
	IPASoftSimple()
		: params_(static_cast<DebayerParams *>(MAP_FAILED)),
		  stats_(static_cast<SwIspStats *>(MAP_FAILED)),
		  blackLevel_(BlackLevel()), ignore_updates_(0)
	{
	}

	~IPASoftSimple()
	{
		if (stats_ != MAP_FAILED)
			munmap(stats_, sizeof(SwIspStats));
		if (params_ != MAP_FAILED)
			munmap(params_, sizeof(DebayerParams));
	}

	int init(const IPASettings &settings,
		 const SharedFD &fdStats,
		 const SharedFD &fdParams,
		 const ControlInfoMap &sensorInfoMap) override;
	int configure(const ControlInfoMap &sensorInfoMap) override;

	int start() override;
	void stop() override;

	void processStats(const ControlList &sensorControls) override;

private:
	void updateExposure(double exposureMSV);

	SharedFD fdStats_;
	SharedFD fdParams_;
	DebayerParams *params_;
	SwIspStats *stats_;
	BlackLevel blackLevel_;

	int32_t exposure_min_, exposure_max_;
	int32_t again_min_, again_max_;
	int32_t again_, exposure_;
	unsigned int ignore_updates_;
};

int IPASoftSimple::init([[maybe_unused]] const IPASettings &settings,
			const SharedFD &fdStats,
			const SharedFD &fdParams,
			const ControlInfoMap &sensorInfoMap)
{
	fdStats_ = fdStats;
	if (!fdStats_.isValid()) {
		LOG(IPASoft, Error) << "Invalid Statistics handle";
		return -ENODEV;
	}

	fdParams_ = fdParams;
	if (!fdParams_.isValid()) {
		LOG(IPASoft, Error) << "Invalid Parameters handle";
		return -ENODEV;
	}

	params_ = static_cast<DebayerParams *>(mmap(nullptr, sizeof(DebayerParams),
						    PROT_WRITE, MAP_SHARED,
						    fdParams_.get(), 0));
	if (params_ == MAP_FAILED) {
		LOG(IPASoft, Error) << "Unable to map Parameters";
		return -errno;
	}

	stats_ = static_cast<SwIspStats *>(mmap(nullptr, sizeof(SwIspStats),
						PROT_READ, MAP_SHARED,
						fdStats_.get(), 0));
	if (stats_ == MAP_FAILED) {
		LOG(IPASoft, Error) << "Unable to map Statistics";
		return -errno;
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

int IPASoftSimple::configure(const ControlInfoMap &sensorInfoMap)
{
	const ControlInfo &exposure_info = sensorInfoMap.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gain_info = sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN)->second;

	exposure_min_ = exposure_info.min().get<int32_t>();
	exposure_max_ = exposure_info.max().get<int32_t>();
	if (!exposure_min_) {
		LOG(IPASoft, Warning) << "Minimum exposure is zero, that can't be linear";
		exposure_min_ = 1;
	}

	again_min_ = gain_info.min().get<int32_t>();
	again_max_ = gain_info.max().get<int32_t>();
	/*
	 * The camera sensor gain (g) is usually not equal to the value written
	 * into the gain register (x). But the way how the AGC algorithm changes
	 * the gain value to make the total exposure closer to the optimum assumes
	 * that g(x) is not too far from linear function. If the minimal gain is 0,
	 * the g(x) is likely to be far from the linear, like g(x) = a / (b * x + c).
	 * To avoid unexpected changes to the gain by the AGC algorithm (abrupt near
	 * one edge, and very small near the other) we limit the range of the gain
	 * values used.
	 */
	if (!again_min_) {
		LOG(IPASoft, Warning) << "Minimum gain is zero, that can't be linear";
		again_min_ = std::min(100, again_min_ / 2 + again_max_ / 2);
	}

	LOG(IPASoft, Info) << "Exposure " << exposure_min_ << "-" << exposure_max_
			   << ", gain " << again_min_ << "-" << again_max_;

	return 0;
}

int IPASoftSimple::start()
{
	return 0;
}

void IPASoftSimple::stop()
{
}

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;
/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;
/*
 * The below value implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

void IPASoftSimple::processStats(const ControlList &sensorControls)
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

	if (ignore_updates_ > 0)
		blackLevel_.update(stats_->yHistogram);
	params_->blackLevel = blackLevel_.get();

	setIspParams.emit(0);

	/*
	 * AE / AGC, use 2 frames delay to make sure that the exposure and
	 * the gain set have applied to the camera sensor.
	 */
	if (ignore_updates_ > 0) {
		--ignore_updates_;
		return;
	}

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const unsigned int blackLevelHistIdx =
		params_->blackLevel / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize = SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int ExposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		ExposureBins[idx] += stats_->yHistogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoft, Debug) << i << ": " << ExposureBins[i];
		denom += ExposureBins[i];
		num += ExposureBins[i] * (i + 1);
	}

	float exposureMSV = (float)num / denom;

	/* sanity check */
	if (!sensorControls.contains(V4L2_CID_EXPOSURE) ||
	    !sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
		LOG(IPASoft, Error) << "Control(s) missing";
		return;
	}

	ControlList ctrls(sensorControls);

	exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();

	updateExposure(exposureMSV);

	ctrls.set(V4L2_CID_EXPOSURE, exposure_);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

	ignore_updates_ = 2;

	setSensorControls.emit(ctrls);

	LOG(IPASoft, Debug) << "exposureMSV " << exposureMSV
			    << " exp " << exposure_ << " again " << again_
			    << " gain R/B " << params_->gainR << "/" << params_->gainB
			    << " black level " << params_->blackLevel;
}

void IPASoftSimple::updateExposure(double exposureMSV)
{
	/* DENOMINATOR of 10 gives ~10% increment/decrement; DENOMINATOR of 5 - about ~20% */
	static constexpr uint8_t kExpDenominator = 10;
	static constexpr uint8_t kExpNumeratorUp = kExpDenominator + 1;
	static constexpr uint8_t kExpNumeratorDown = kExpDenominator - 1;

	int next;

	if (exposureMSV < kExposureOptimal - kExposureSatisfactory) {
		next = exposure_ * kExpNumeratorUp / kExpDenominator;
		if (next - exposure_ < 1)
			exposure_ += 1;
		else
			exposure_ = next;
		if (exposure_ >= exposure_max_) {
			next = again_ * kExpNumeratorUp / kExpDenominator;
			if (next - again_ < 1)
				again_ += 1;
			else
				again_ = next;
		}
	}

	if (exposureMSV > kExposureOptimal + kExposureSatisfactory) {
		if (exposure_ == exposure_max_ && again_ != again_min_) {
			next = again_ * kExpNumeratorDown / kExpDenominator;
			if (again_ - next < 1)
				again_ -= 1;
			else
				again_ = next;
		} else {
			next = exposure_ * kExpNumeratorDown / kExpDenominator;
			if (exposure_ - next < 1)
				exposure_ -= 1;
			else
				exposure_ = next;
		}
	}

	exposure_ = std::clamp(exposure_, exposure_min_, exposure_max_);
	again_ = std::clamp(again_, again_min_, again_max_);
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
	"simple",
};

IPAInterface *ipaCreate()
{
	return new ipa::soft::IPASoftSimple();
}

} /* extern "C" */

} /* namespace libcamera */
