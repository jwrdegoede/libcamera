/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS ISP Parameters
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#pragma once

#include <linux/camss-config.h>
#include <linux/videodev2.h>

/*
 * FIXME must be moved to a to-be-written camss IPA and then this should be:
 * #include <libipa/v4l2_params.h>
 */
#include "../../../ipa/libipa/v4l2_params.h"

namespace libcamera {

namespace ipa::camss {

enum class CamssBlocks {
	AwbGains,
	ChromaEnh,
	ColorCorrect,
};

namespace details {

template<CamssBlocks B>
struct block_type {
};

#define CAMSS_DEFINE_BLOCK_TYPE(id, cfgType, blkType)                    \
	template<>                                                       \
	struct block_type<CamssBlocks::id> {                             \
		using type = struct camss_params_##cfgType;              \
		static constexpr camss_params_block_type blockType =     \
			camss_params_block_type::CAMSS_PARAMS_##blkType; \
	}

CAMSS_DEFINE_BLOCK_TYPE(AwbGains, wb_gain, WB_GAIN);
CAMSS_DEFINE_BLOCK_TYPE(ChromaEnh, chroma_enhan, CHROMA_ENHAN);
CAMSS_DEFINE_BLOCK_TYPE(ColorCorrect, color_correct, COLOR_CORRECT);

struct param_traits {
	using id_type = CamssBlocks;

	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace details */

class CamssParams : public V4L2Params<details::param_traits>
{
public:
	CamssParams(Span<uint8_t> data)
		: V4L2Params(data, V4L2_ISP_PARAMS_VERSION_V1)
	{
	}
};

} /* namespace ipa::camss */

} /* namespace libcamera */
