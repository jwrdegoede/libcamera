#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Af : public Algorithm
{
public:
	Af();
	~Af() = default;

	void prepare([[maybe_unused]] typename Module::Context &context,
			     [[maybe_unused]] const uint32_t frame,
			     [[maybe_unused]] typename Module::FrameContext &frameContext,
			     [[maybe_unused]] typename Module::Params *params);

	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void movingAvg(uint64_t sharpness);
	void initState(IPAContext &context);
	void lockedState(IPAContext &context,  uint64_t sharpness, bool hasSharpness);
	void fullSweepState(IPAContext &context, uint64_t sharpness, bool hasSharpness);
	void smallSweepState(IPAContext &context,  uint64_t sharpness, bool hasSharpness);
	const uint8_t CframeSkip = 10;
	uint8_t frameCounter;
	bool wantSharpness;
	int32_t lensPos;
	std::map<uint8_t, uint64_t> values;
	uint64_t sharpnessLock;
	uint64_t movingArray[8];
	uint64_t sharpnessAverage;
	uint32_t movingIndex;
	uint32_t movingTotal;
	uint8_t oofCounter;
	uint32_t itt;
	std::pair<int32_t, uint64_t> highest;
	bool stable;
	bool waitFlag;

	enum FocusState {
		init,
		locked,
		full,
		small,
	};

	FocusState afState;

};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */