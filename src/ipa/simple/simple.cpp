/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * simple.cpp - Simple SoftwareIsp Image Processing Algorithm module
 */
#include <libcamera/ipa/simple_ipa_interface.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/camera_sensor.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASimple)

namespace ipa::simple {

class IPASimple : public ipa::simple::IPASimpleInterface
{
public:
	IPASimple() : ignore_updates_(0)
	{
	}
	~IPASimple()
	{
	}

	int init(const IPASettings &settings,
		 const ControlList &sensorControls) override;

	int start() override;
	void stop() override;

	void processStats(const ControlList &sensorControls,
			  float bright_ratio, float too_bright_ratio);

private:
	void update_exposure(double ev_adjustment);

	int exposure_min_, exposure_max_;
	int again_min_, again_max_;
	int again_, exposure_;
	int ignore_updates_;
};

int IPASimple::init([[maybe_unused]] const IPASettings &settings,
		    const ControlList &sensorControls)
{
	if (!sensorControls.contains(V4L2_CID_EXPOSURE)) {
		LOG(IPASimple, Error) << "Don't have exposure control";
		return -EINVAL;
	}
	if (!sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
		LOG(IPASimple, Error) << "Don't have gain control";
		return -EINVAL;
	}

	const ControlInfoMap &infoMap = *sensorControls.infoMap();
	const ControlInfo &exposure_info = infoMap.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gain_info = infoMap.find(V4L2_CID_ANALOGUE_GAIN)->second;

	exposure_min_ = exposure_info.min().get<int>();
	if (!exposure_min_) {
		LOG(IPASimple, Warning) << "Minimum exposure is zero, that can't be linear";
		exposure_min_ = 1;
	}
	exposure_max_ = exposure_info.max().get<int>();
	again_min_ = gain_info.min().get<int>();
	if (!again_min_) {
		LOG(IPASimple, Warning) << "Minimum gain is zero, that can't be linear";
		again_min_ = 100;
	}
	again_max_ = gain_info.max().get<int>();

	LOG(IPASimple, Info) << "Exposure" << exposure_min_ << " " << exposure_max_
			     << ", gain" << again_min_ << " " << again_max_;

	return 0;
}

void IPASimple::update_exposure(double ev_adjustment)
{
	double exp = (double)exposure_;
	double gain = (double)again_;
	double ev = ev_adjustment * exp * gain;

	/*
	 * Try to use the minimal possible analogue gain.
	 * The exposure can be any value from exposure_min_ to exposure_max_,
	 * and normally this should keep the frame rate intact.
	 */

	exp = ev / again_min_;
	if (exp > exposure_max_) exposure_ = exposure_max_;
	else if (exp < exposure_min_) exposure_ = exposure_min_;
	else exposure_ = (int)exp;

	gain = ev / exposure_;
	if (gain > again_max_) again_ = again_max_;
	else if (gain < again_min_) again_ = again_min_;
	else again_ = (int)gain;

	LOG(IPASimple, Debug) << "Desired EV = " << ev
			      << ", real EV = " << (double)again_ * exposure_;
}

void IPASimple::processStats(const ControlList &sensorControls,
			     float bright_ratio, float too_bright_ratio)
{
	double ev_adjustment = 0.0;
	ControlList ctrls(sensorControls);

	/*
	 * Use 2 frames delay to make sure that the exposure and the gain set
	 * have applied to the camera sensor
	 */
	if (ignore_updates_ > 0) {
		LOG(IPASimple, Debug) << "Skipping exposure update: "
				      << ignore_updates_;
		--ignore_updates_;
		return;
	}

	if (bright_ratio < 0.01) ev_adjustment = 1.1;
	if (too_bright_ratio > 0.04) ev_adjustment = 0.9;

	if (ev_adjustment != 0.0) {
		/* sanity check */
		if (!sensorControls.contains(V4L2_CID_EXPOSURE) ||
		    !sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
			LOG(IPASimple, Error) << "Control(s) missing";
			return;
		}

		exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int>();
		again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int>();

		update_exposure(ev_adjustment);

		ctrls.set(V4L2_CID_EXPOSURE, exposure_);
		ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

		ignore_updates_ = 2;

		setSensorControls.emit(ctrls);
	}
}

int IPASimple::start()
{
	return 0;
}

void IPASimple::stop()
{
}

} /* namespace ipa::simple */

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
	return new ipa::simple::IPASimple();
}
}

} /* namespace libcamera */
