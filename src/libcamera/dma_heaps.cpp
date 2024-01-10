/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#include "libcamera/internal/dma_heaps.h"

#include <array>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include <libcamera/base/log.h>

/**
 * \file dma_heaps.cpp
 * \brief dma-heap allocator
 */

namespace libcamera {

/*
 * /dev/dma_heap/linux,cma is the dma-heap allocator, which allows dmaheap-cma
 * to only have to worry about importing.
 *
 * Annoyingly, should the cma heap size be specified on the kernel command line
 * instead of DT, the heap gets named "reserved" instead.
 */

/**
 * \struct DmaHeapInfo
 * \brief Tells what type of dma-heap the dma-heap represented by the device node name is
 * \var DmaHeapInfo::flag
 * \brief The type of the dma-heap
 * \var DmaHeapInfo::name
 * \brief The dma-heap's device node name
 */
struct DmaHeapInfo {
	DmaHeap::DmaHeapFlag flag;
	const char *name;
};

static constexpr std::array<DmaHeapInfo, 3> heapInfos = {
	{ /* CMA heap names first */
	  { DmaHeap::DmaHeapFlag::Cma, "/dev/dma_heap/linux,cma" },
	  { DmaHeap::DmaHeapFlag::Cma, "/dev/dma_heap/reserved" },
	  { DmaHeap::DmaHeapFlag::System, "/dev/dma_heap/system" } }
};

LOG_DEFINE_CATEGORY(DmaHeap)

/**
 * \class DmaHeap
 * \brief Helper class for dma-heap allocations
 */

/**
 * \enum DmaHeap::DmaHeapFlag
 * \brief Type of the dma-heap
 * \var DmaHeap::Cma
 * \brief Allocate from a CMA dma-heap
 * \var DmaHeap::System
 * \brief Allocate from the system dma-heap
 */

/**
 * \typedef DmaHeap::DmaHeapFlags
 * \brief A bitwise combination of DmaHeap::DmaHeapFlag values
 */

/**
 * \brief Construct a DmaHeap that owns a CMA or system dma-heap file descriptor
 * \param [in] flags The type(s) of the dma-heap(s) to allocate from
 *
 * By default \a flags are set to DmaHeap::DmaHeapFlag::Cma. The constructor goes
 * through the internal list of possible names of the CMA and system dma-heap devices
 * until the dma-heap device of the requested type is successfully opened. If more
 * than one dma-heap type is specified in flags the CMA heap is tried first. If it
 * fails to open any dma-heap device an invalid DmaHeap object is constructed.
 * A valid DmaHeap object owns a wrapped dma-heap file descriptor.
 *
 * Please check the new DmaHeap object with \ref DmaHeap::isValid before using it.
 */
DmaHeap::DmaHeap(DmaHeapFlags flags)
{
	for (const auto &info : heapInfos) {
		if (!(flags & info.flag))
			continue;

		int ret = ::open(info.name, O_RDWR | O_CLOEXEC, 0);
		if (ret < 0) {
			ret = errno;
			LOG(DmaHeap, Debug)
				<< "Failed to open " << info.name << ": "
				<< strerror(ret);
			continue;
		}

		LOG(DmaHeap, Debug) << "Using " << info.name;
		dmaHeapHandle_ = UniqueFD(ret);
		break;
	}

	if (!dmaHeapHandle_.isValid())
		LOG(DmaHeap, Error) << "Could not open any dmaHeap device";
}

/**
 * \brief Destroy the DmaHeap instance
 *
 * Destroying a DmaHeap instance which owns a wrapped dma-heap file descriptor
 * closes the descriptor automatically.
 */
DmaHeap::~DmaHeap() = default;

/**
 * \fn DmaHeap::isValid()
 * \brief Check if the DmaHeap instance is valid
 * \return True if the DmaHeap is valid, false otherwise
 */

/**
 * \brief Allocate a dma-buf from the DmaHeap
 * \param [in] name The name to set for the allocated buffer
 * \param [in] size The size of the buffer to allocate
 * \return The \ref UniqueFD of the allocated buffer
 *
 * Allocates a dma-buf with read/write access.
 * If the allocation fails returns invalid UniqueFD.
 */
UniqueFD DmaHeap::alloc(const char *name, std::size_t size)
{
	int ret;

	if (!name)
		return {};

	struct dma_heap_allocation_data alloc = {};

	alloc.len = size;
	alloc.fd_flags = O_CLOEXEC | O_RDWR;

	ret = ::ioctl(dmaHeapHandle_.get(), DMA_HEAP_IOCTL_ALLOC, &alloc);
	if (ret < 0) {
		LOG(DmaHeap, Error) << "dmaHeap allocation failure for " << name;
		return {};
	}

	UniqueFD allocFd(alloc.fd);
	ret = ::ioctl(allocFd.get(), DMA_BUF_SET_NAME, name);
	if (ret < 0) {
		LOG(DmaHeap, Error) << "dmaHeap naming failure for " << name;
		return {};
	}

	return allocFd;
}

} /* namespace libcamera */
