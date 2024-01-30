/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * shared_mem_object.cpp - Helper class for shared memory allocations
 */

#include "libcamera/internal/shared_mem_object.h"

#include <sys/types.h>
#include <unistd.h>

/**
 * \file shared_mem_object.cpp
 * \brief Helper class for shared memory allocations
 */

namespace libcamera {

/**
 * \class SharedMem
 * \brief Helper class for allocating shared memory
 *
 * Memory is allocated and exposed as a SharedFD for use across IPC boundaries.
 *
 * SharedMem allocates the shared memory of the given size and maps it.
 * To check that the shared memory was allocated and mapped successfully, one
 * needs to verify that the pointer to the shared memory returned by SharedMem::mem()
 * is not nullptr.
 *
 * To access the shared memory from another process the SharedFD should be passed
 * to that process, and then the shared memory should be mapped into that process
 * address space by calling mmap().
 *
 * A single memfd is created for every SharedMem. If there is a need to allocate
 * a large number of objects in shared memory, these objects should be grouped
 * together and use the shared memory allocated by a single SharedMem object if
 * possible. This will help to minimize the number of created memfd's.
 */

/**
 * \fn SharedMem::SharedMem(const std::string &name, std::size_t size)
 * \brief Contstructor for the SharedMem
 * \param[in] name Name of the SharedMem
 * \param[in] size Size of the shared memory to allocate and map
 */

/**
 * \fn SharedMem::SharedMem(SharedMem &&rhs)
 * \brief Move constructor for SharedMem
 * \param[in] rhs The object to move
 */

/**
 * \fn SharedMem::~SharedMem()
 * \brief SharedMem destructor
 *
 * Unmaps the allocated shared memory. Decrements the shared memory descriptor use
 * count.
 */

/**
 * \fn SharedMem &SharedMem::operator=(SharedMem &&rhs)
 * \brief Move constructor for SharedMem
 * \param[in] rhs The object to move
 */

/**
 * \fn const SharedFD &SharedMem::fd() const
 * \brief Gets the file descriptor for the underlaying shared memory
 * \return The file descriptor
 */

/**
 * \fn void *SharedMem::mem() const
 * \brief Gets the pointer to the underlaying shared memory
 * \return The pointer to the shared memory
 */

SharedMem::SharedMem(const std::string &name, std::size_t size)
	: name_(name), size_(size), mem_(nullptr)
{
	int fd = memfd_create(name_.c_str(), MFD_CLOEXEC);
	if (fd < 0)
		return;

	fd_ = SharedFD(std::move(fd));
	if (!fd_.isValid())
		return;

	if (ftruncate(fd_.get(), size_) < 0)
		return;

	mem_ = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
		    fd_.get(), 0);
	if (mem_ == MAP_FAILED)
		mem_ = nullptr;
}

/**
 * \class SharedMemObject
 * \brief Helper class for allocating objects in shared memory
 *
 * Memory is allocated and exposed as a SharedFD for use across IPC boundaries.
 *
 * Given the type of the object to be created in shared memory and the arguments
 * to pass to this object's constructor, SharedMemObject allocates the shared memory
 * of the size of the object and constructs the object in this memory. To check that
 * the SharedMemObject was created successfully, one needs to verify that the
 * underlying SharedFD (the reference to it is returned by SharedMemObject::fd() member
 * function) is valid. The object created in the shared memory can be accessed using
 * the SharedMemObject::operator*() indirection operator. Its members can be accessed
 * with the SharedMemObject::operator->() member of pointer operator.
 *
 * To access the object from another process the SharedFD should be passed to that
 * process, and the shared memory should be mapped by calling mmap().
 *
 * A single memfd is created for every SharedMemObject. If there is a need to allocate
 * a large number of objects in shared memory, these objects should be grouped into a
 * single large object to keep the number of created memfd's reasonably small.
 */

/**
 * \var SharedMemObject::SIZE
 * \brief The size of the object that is going to be stored here
 */

/**
 * \fn SharedMemObject< T >::SharedMemObject(const std::string &name, Args &&...args)
 * \brief Contstructor for the SharedMemObject
 * \param[in] name Name of the SharedMemObject
 * \param[in] args Args to pass to the constructor of the object in shared memory
 */

/**
 * \fn SharedMemObject::SharedMemObject(SharedMemObject<T> &&rhs)
 * \brief Move constructor for SharedMemObject
 * \param[in] rhs The object to move
 */

/**
 * \fn SharedMemObject::~SharedMemObject()
 * \brief SharedMemObject destructor
 *
 * Destroys the object created in the shared memory and then unmaps the shared memory.
 * Decrements the shared memory descriptor use count.
 */

/**
 * \fn SharedMemObject::operator=(SharedMemObject<T> &&rhs)
 * \brief Operator= for SharedMemObject
 * \param[in] rhs The SharedMemObject object to take the data from
 */

/**
 * \fn SharedMemObject::operator->()
 * \brief Operator-> for SharedMemObject
 * \return The pointer to the object
 */

/**
 * \fn const T *SharedMemObject::operator->() const
 * \brief Operator-> for SharedMemObject
 * \return The pointer to the const object
 */

/**
 * \fn SharedMemObject::operator*()
 * \brief Operator* for SharedMemObject
 * \return The reference to the object
 */

/**
 * \fn const T &SharedMemObject::operator*() const
 * \brief Operator* for SharedMemObject
 * \return Const reference to the object
 */

/**
 * \fn SharedMemObject::fd()
 * \brief Gets the file descriptor for the underlaying storage file
 * \return The shared memory file descriptor (as SharedFD)
 */

/**
 * \fn SharedMemObject::operator bool()
 * \brief Operator bool() for SharedMemObject
 * \return True if the object was created OK in the shared memory, false otherwise
 */

} // namespace libcamera
