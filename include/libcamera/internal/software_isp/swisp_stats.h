/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_stats.h - Statistics data format used by the software ISP and software IPA
 */

#pragma once

namespace libcamera {

/**
 * \brief Struct that holds the statistics for the Software ISP.
 */
struct SwIspStats {
	/**
	 * \brief Holds the sum of all sampled red pixels.
	 */
	unsigned long sumR_;
	/**
	 * \brief Holds the sum of all sampled green pixels.
	 */
	unsigned long sumG_;
	/**
	 * \brief Holds the sum of all sampled blue pixels.
	 */
	unsigned long sumB_;
	/**
	 * \brief Number of bins in the yHistogram.
	 */
	static constexpr unsigned int kYHistogramSize = 16;
	/**
	 * \brief A histogram of luminance values.
	 */
	std::array<unsigned int, kYHistogramSize> yHistogram;
};

} /* namespace libcamera */
