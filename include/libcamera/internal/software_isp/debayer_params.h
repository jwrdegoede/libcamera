/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * debayer_params.h - DebayerParams header
 */

#pragma once

namespace libcamera {

/**
 * \brief Struct to hold the debayer parameters.
 */
struct DebayerParams {
	/**
	 * \brief const value for 1.0 gain
	 */
	static constexpr unsigned int kGain10 = 256;

	/**
	 * \brief Red Gain
	 *
	 * 128 = 0.5, 256 = 1.0, 512 = 2.0, etc.
	 */
	unsigned int gainR;
	/**
	 * \brief Green Gain
	 *
	 * 128 = 0.5, 256 = 1.0, 512 = 2.0, etc.
	 */
	unsigned int gainG;
	/**
	 * \brief Blue Gain
	 *
	 * 128 = 0.5, 256 = 1.0, 512 = 2.0, etc.
	 */
	unsigned int gainB;
	/**
	 * \brief Gamma correction, 1.0 is no correction
	 */
	float gamma;
	/**
	 * \brief Level of the black point, 0..255, 0 is no correction.
	 */
	unsigned int blackLevel;
};

} /* namespace libcamera */
