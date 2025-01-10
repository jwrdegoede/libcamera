#include "af.h"

#include <libcamera/base/log.h>

namespace libcamera {
LOG_DEFINE_CATEGORY(af)

namespace ipa::soft::algorithms {

Af::Af()
	: lensPos(0), highest(0, 0), stable(false), waitFlag(false)
{
}

void Af::process([[maybe_unused]] IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext, [[maybe_unused]] const SwIspStats *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	uint64_t sharpness = stats->sharpnessValue_;

	switch (afState) {
	case 0:
		initState(context);
		break;
	case 1: //Locked
		lockedState(context, sharpness);
		break;
	case 2: // Full Sweep
		fullSweepState(context, sharpness);
		break;
	case 3: // small sweep (hill climb)
		smallSweepState(context, sharpness);
		break;
	}
}

void Af::initState([[maybe_unused]] IPAContext &context)
{
	context.activeState.af.focus = 0;
	if (itt < 255) {
		itt++;
	} else {
		itt = 0;
		afState = 2; // full sweep
	}
}

void Af::lockedState([[maybe_unused]] IPAContext &context, [[maybe_unused]] uint64_t sharpness)
{
	if (stable) {
		itt++;
		if (itt >= 20) {
			stable = false;
			itt = 0;
		}
		return;
	}
	if(sharpnessLock < sharpness) {
		sharpnessLock = sharpness;
		context.activeState.af.sharpnessLock = sharpness;
	}

	else if ((uint64_t)((double)sharpnessLock * 0.6) > sharpness) { // to sweep
		lensPos = 0;
		waitFlag = true;
		context.activeState.af.focus = lensPos;
		afState = 2; // full sweep
	} else if ((uint64_t)((double)sharpnessLock * 0.8) > sharpness) { // to smallsweep
		if (lensPos < 200) {
			lensPos = 0;
		} else {
			lensPos = lensPos - 200;
		}
		waitFlag = true;
		context.activeState.af.focus = lensPos;
		itt = 0;
		afState = 3; // small sweep
	}
}

void Af::fullSweepState([[maybe_unused]] IPAContext &context, [[maybe_unused]] uint64_t sharpness)
{
	if (waitFlag) {
		itt++;
		if (itt >= 20) {
			waitFlag = false;
			itt = 0;
		}
		return;
	}

	int32_t focusMax = context.configuration.af.afocusMax;
    if (lensPos < focusMax && highest.second * 0.5 < sharpness) {
        if (sharpness > highest.second) {
            highest = std::make_pair(lensPos, sharpness);
			LOG(af, Info) << "Highest Sharpness: " << highest.second;
			LOG(af, Info) << "Highest focus pos: " << (int32_t)highest.first;
		}
		lensPos += context.configuration.af.stepValue;
		if (lensPos > focusMax) lensPos = focusMax;
		context.activeState.af.focus = lensPos;
	} else {
		lensPos = highest.first;
		sharpnessLock = highest.second;
		highest = std::make_pair(0, 0);
		context.activeState.af.sharpnessLock = sharpnessLock;
		context.activeState.af.focus = lensPos;
		stable = true;
		itt = 0;
		afState = 1;
	}
}

void Af::smallSweepState([[maybe_unused]] IPAContext &context, [[maybe_unused]] uint64_t sharpness)
{
	if (waitFlag) {
		itt++;
		if (itt >= 20) {
			waitFlag = false;
			itt = 0;
		}
		return;
	}

	if (itt < 400 && highest.second * 0.5 < sharpness) {
		if (sharpness > highest.second) {
			highest = std::make_pair(lensPos, sharpness);
		}
		lensPos++;
		itt++;
		context.activeState.af.focus = lensPos;
	} else {
		lensPos = highest.first;
		sharpnessLock = highest.second;
		highest = std::make_pair(0, 0);
		stable = true;
		itt = 0;
		context.activeState.af.sharpnessLock = sharpnessLock;
		context.activeState.af.focus = lensPos;
		afState = 1;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */