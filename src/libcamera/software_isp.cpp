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

SoftwareIsp::SoftwareIsp(const std::string &name)
	: name_(name)
{
}

SoftwareIsp::~SoftwareIsp()
{
}

} /* namespace libcamera */
