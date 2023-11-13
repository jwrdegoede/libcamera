/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * soft-base.cpp - Software IPA base class
 */

#include "soft_base.h"

#include <sys/mman.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoft)

namespace ipa::soft {

IPASoftBase::IPASoftBase()
{
}

IPASoftBase::~IPASoftBase()
{
}

int IPASoftBase::init([[maybe_unused]] const IPASettings &settings,
		      const SharedFD &fdStats,
		      const ControlInfoMap &sensorInfoMap)
{
	fdStats_ = std::move(fdStats);
	if (!fdStats_.isValid()) {
		LOG(IPASoft, Error) << "Invalid Statistics handle";
		return -ENODEV;
	}

	return platformInit(sensorInfoMap);
}

int IPASoftBase::configure(const ControlInfoMap &sensorInfoMap)
{
	return platformConfigure(sensorInfoMap);
}

int IPASoftBase::start()
{
	return platformStart();
}

void IPASoftBase::stop()
{
	return platformStop();
}

void IPASoftBase::processStats(const ControlList &sensorControls)
{
	return platformProcessStats(sensorControls);
}

} /* namespace ipa::soft */

} /* namespace libcamera */
