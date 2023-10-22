/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * software_isp.cpp - Interface for a software implementation of an ISP
 */

#include "libcamera/internal/software_isp.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(SoftwareIsp)

SoftwareIsp::SoftwareIsp([[maybe_unused]] PipelineHandler *pipe,
			 [[maybe_unused]] const ControlInfoMap &sensorControls)
{
}

SoftwareIsp::~SoftwareIsp()
{
}

/* SoftwareIspFactoryBase */

SoftwareIspFactoryBase::SoftwareIspFactoryBase()
{
	registerType(this);
}

void SoftwareIspFactoryBase::registerType(SoftwareIspFactoryBase *factory)
{
	SoftwareIspFactoryBase *&registered =
		SoftwareIspFactoryBase::factory();

	ASSERT(!registered && factory);
	registered = factory;
}

SoftwareIspFactoryBase *&SoftwareIspFactoryBase::factory()
{
	static SoftwareIspFactoryBase *factory;
	return factory;
}

std::unique_ptr<SoftwareIsp>
SoftwareIspFactoryBase::create(PipelineHandler *pipe,
			       const ControlInfoMap &sensorControls)
{
	SoftwareIspFactoryBase *factory = SoftwareIspFactoryBase::factory();
	if (!factory)
		return nullptr;

	std::unique_ptr<SoftwareIsp> swIsp = factory->createInstance(pipe, sensorControls);
	if (swIsp->isValid())
		return swIsp;

	return nullptr;
}

} /* namespace libcamera */
