/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#include <array>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/dma_heaps.h"

namespace libcamera {

/*
 * /dev/dma-heap/linux,cma is the dma-heap allocator, which allows dmaheap-cma
 * to only have to worry about importing.
 *
 * Annoyingly, should the cma heap size be specified on the kernel command line
 * instead of DT, the heap gets named "reserved" instead.
 */
static constexpr std::array<std::pair<DmaHeap::DmaHeapFlag, const char *>, 3> heapNames = {
	/* CMA heap names first */
	std::make_pair(DmaHeap::DmaHeapFlag::Cma, "/dev/dma_heap/linux,cma"),
	std::make_pair(DmaHeap::DmaHeapFlag::Cma, "/dev/dma_heap/reserved"),
	std::make_pair(DmaHeap::DmaHeapFlag::System, "/dev/dma_heap/system")
};

LOG_DEFINE_CATEGORY(DmaHeap)

DmaHeap::DmaHeap(DmaHeapFlags flags)
{
	int ret;

	for (const auto &name : heapNames) {
		if (flags & name.first) {
			ret = ::open(name.second, O_RDWR | O_CLOEXEC, 0);
			if (ret < 0) {
				ret = errno;
				LOG(DmaHeap, Debug) << "Failed to open " << name.second << ": "
						    << strerror(ret);
				continue;
			}

			LOG(DmaHeap, Debug) << "Using " << name.second;
			dmaHeapHandle_ = UniqueFD(ret);
			break;
		}
	}

	if (!dmaHeapHandle_.isValid())
		LOG(DmaHeap, Error) << "Could not open any dmaHeap device";
}

DmaHeap::~DmaHeap() = default;

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
		LOG(DmaHeap, Error) << "dmaHeap allocation failure for "
				<< name;
		return {};
	}

	UniqueFD allocFd(alloc.fd);
	ret = ::ioctl(allocFd.get(), DMA_BUF_SET_NAME, name);
	if (ret < 0) {
		LOG(DmaHeap, Error) << "dmaHeap naming failure for "
				<< name;
		return {};
	}

	return allocFd;
}

} /* namespace libcamera */
