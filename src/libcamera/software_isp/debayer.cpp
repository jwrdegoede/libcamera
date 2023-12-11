/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * debayer.cpp - debayer base class
 */

#include "libcamera/internal/software_isp/debayer.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Debayer)

Debayer::~Debayer()
{
}

} /* namespace libcamera */
