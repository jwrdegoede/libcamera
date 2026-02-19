/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Camss Frames helper
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Based on the IPU3 pipeline-handler which is:
 * Copyright (C) 2020, Google Inc.
 */

#include "camss_frames.h"

#include <libcamera/base/log.h>

#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

CamssFrames::CamssFrames()
{
}

void CamssFrames::init(const std::vector<std::unique_ptr<FrameBuffer>> &paramBuffers,
		      const std::vector<std::unique_ptr<FrameBuffer>> &statBuffers)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : paramBuffers)
		availableParamBuffers_.push(buffer.get());

	for (const std::unique_ptr<FrameBuffer> &buffer : statBuffers)
		availableStatBuffers_.push(buffer.get());

	frameInfo_.clear();
}

void CamssFrames::clear()
{
	availableParamBuffers_ = {};
	availableStatBuffers_ = {};
}

CamssFrames::Info *CamssFrames::create(Request *request)
{
	unsigned int id = request->sequence();

	if (availableParamBuffers_.empty()) {
		LOG(Camss, Debug) << "Parameters buffer underrun";
		return nullptr;
	}

	if (availableStatBuffers_.empty()) {
		LOG(Camss, Debug) << "Statistics buffer underrun";
		return nullptr;
	}

	FrameBuffer *paramBuffer = availableParamBuffers_.front();
	FrameBuffer *statBuffer = availableStatBuffers_.front();

	paramBuffer->_d()->setRequest(request);
	statBuffer->_d()->setRequest(request);

	availableParamBuffers_.pop();
	availableStatBuffers_.pop();

	auto [it, inserted] = frameInfo_.try_emplace(id);
	ASSERT(inserted);

	auto &info = it->second;

	info.id = id;
	info.request = request;
	info.rawBuffer = nullptr;
	info.paramBuffer = paramBuffer;
	info.statBuffer = statBuffer;
	info.paramDequeued = false;
	info.metadataProcessed = false;

	return &info;
}

void CamssFrames::remove(CamssFrames::Info *info)
{
	/* Return params and stat buffer for reuse. */
	availableParamBuffers_.push(info->paramBuffer);
	availableStatBuffers_.push(info->statBuffer);

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id);
}

bool CamssFrames::tryComplete(CamssFrames::Info *info)
{
	Request *request = info->request;

	if (request->hasPendingBuffers())
		return false;

	if (!info->metadataProcessed)
		return false;

	if (!info->paramDequeued)
		return false;

	remove(info);

	bufferAvailable.emit();

	return true;
}

CamssFrames::Info *CamssFrames::find(unsigned int id)
{
	const auto &itInfo = frameInfo_.find(id);

	if (itInfo != frameInfo_.end())
		return &itInfo->second;

	LOG(Camss, Fatal) << "Can't find tracking information for frame " << id;

	return nullptr;
}

CamssFrames::Info *CamssFrames::find(FrameBuffer *buffer)
{
	for (auto &[id, info] : frameInfo_) {
		for (const auto &[stream, buf] : info.request->buffers())
			if (buf == buffer)
				return &info;

		if (info.rawBuffer == buffer || info.paramBuffer == buffer ||
		    info.statBuffer == buffer)
			return &info;
	}

	LOG(Camss, Fatal) << "Can't find tracking information from buffer";

	return nullptr;
}

} /* namespace libcamera */
