/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * dpc.hpp - DPC (defective pixel correction) control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../dpc_status.h"

namespace RPiController {

// Back End algorithm to apply appropriate GEQ settings.

struct DpcConfig {
	int strength;
};

class Dpc : public Algorithm
{
public:
	Dpc(Controller *controller);
	char const *name() const override;
	void read(boost::property_tree::ptree const &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	DpcConfig config_;
};

} // namespace RPiController
