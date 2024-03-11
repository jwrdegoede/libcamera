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

#include "debayer.h"

namespace libcamera {

/**
 * \class Debayer
 * \brief Base debayering class
 *
 * Base class that provides functions for setting up the debayering process.
 */

LOG_DEFINE_CATEGORY(Debayer)

Debayer::~Debayer()
{
}

} /* namespace libcamera */
