/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * gbm.h - Helper class for managing GBM interactions.
 */

#pragma once

#include <gbm.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(GBM)

/**
 * \brief Helper class for managing GBM interactions
 *
 * The GBM class provides a simplified interface for creating and managing
 * GBM devices. It handles the initialization and teardown of GBM devices
 * used for buffer allocation in graphics and camera pipelines.
 *
 * This class is responsible for opening a DRI render node, creating a GBM
 * device, and providing access to the device and its associated pixel format.
 */
class GBM
{
public:
	/**
	 *\brief GBM constructor.
	 *
	 * Creates a GBM instance with unitialised state.
	 */
	GBM();

	/**
	 *\brief GBM destructor
	 *
	 * Cleans up the GBM device if it was successfully created, and closes
	 * the associated file descriptor.
	 */
	~GBM();

	/**
	 * \brief Create and initialize a GBM device
	 *
	 * Opens the DRI render node (/dev/dri/renderD128) and creates a GBM
	 * device using the libgbm library. Sets the default pixel format to
	 * ARGB8888.
	 *
	 * \return 0 on success, or a negative error code on failure
	 */
	int createDevice();


	/**
	 * \brief Retrieve the GBM device handle
	 *
	 * \return Pointer to the gbm_device structure, or nullptr if the device
	 * has not been created
	 */
	struct gbm_device *getDevice() { return gbm_device_; }

	/**
	 * \brief Retrieve the pixel format
	 *
	 * \return The PixelFormat used by this GBM instance (ARGB8888)
	 */
	PixelFormat getPixelFormat() { return format_; }

private:
	int fd_;
	struct gbm_device *gbm_device_;
	PixelFormat format_;
};

} // namespace libcamera
