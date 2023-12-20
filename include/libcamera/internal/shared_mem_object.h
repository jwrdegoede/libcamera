/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * shared_mem_object.h - Helper class for shared memory allocations
 */
#pragma once

#include <cstddef>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <libcamera/base/class.h>
#include <libcamera/base/shared_fd.h>

namespace libcamera {

/**
 * \class SharedMemObject
 * \brief Helper class for shared memory allocations.
 *
 * Takes a template T which is used to indicate the
 * data type of the object stored.
 */
template<class T>
class SharedMemObject
{
public:
	/**
	 * \brief The size of the object that is going to be stored here.
	 */
	static constexpr std::size_t SIZE = sizeof(T);

	SharedMemObject()
		: obj_(nullptr)
	{
	}

	/**
	 * \brief Contstructor for the SharedMemObject.
	 * \param[in] name The requested name.
	 * \param[in] args Any additional args.
	 */
	template<class... Args>
	SharedMemObject(const std::string &name, Args &&...args)
		: name_(name), obj_(nullptr)
	{
		void *mem;
		int ret;

		ret = memfd_create(name_.c_str(), MFD_CLOEXEC);
		if (ret < 0)
			return;

		fd_ = SharedFD(std::move(ret));
		if (!fd_.isValid())
			return;

		ret = ftruncate(fd_.get(), SIZE);
		if (ret < 0)
			return;

		mem = mmap(nullptr, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			   fd_.get(), 0);
		if (mem == MAP_FAILED)
			return;

		obj_ = new (mem) T(std::forward<Args>(args)...);
	}

	/**
	 * \brief Move constructor for SharedMemObject.
	 * \param[in] rhs The object to move.
	 */
	SharedMemObject(SharedMemObject<T> &&rhs)
	{
		this->name_ = std::move(rhs.name_);
		this->fd_ = std::move(rhs.fd_);
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
	}

	~SharedMemObject()
	{
		if (obj_) {
			obj_->~T();
			munmap(obj_, SIZE);
		}
	}

	/* Make SharedMemObject non-copyable for now. */
	LIBCAMERA_DISABLE_COPY(SharedMemObject)

	/**
	 * \brief Operator= for SharedMemObject.
	 * \param[in] rhs The SharedMemObject object to take the data from.
	 */
	SharedMemObject<T> &operator=(SharedMemObject<T> &&rhs)
	{
		this->name_ = std::move(rhs.name_);
		this->fd_ = std::move(rhs.fd_);
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
		return *this;
	}

	/**
	 * \brief Operator-> for SharedMemObject.
	 *
	 * \return the object.
	 */
	T *operator->()
	{
		return obj_;
	}

	/**
	 * \brief Operator-> for SharedMemObject.
	 *
	 * \return the object.
	 */
	const T *operator->() const
	{
		return obj_;
	}

	/**
	 * \brief Operator* for SharedMemObject.
	 *
	 * \return the object.
	 */
	T &operator*()
	{
		return *obj_;
	}

	/**
	 * \brief Operator* for SharedMemObject.
	 *
	 * \return the object.
	 */
	const T &operator*() const
	{
		return *obj_;
	}

	/**
	 * \brief Gets the file descriptor for the underlaying storage file.
	 *
	 * \return the file descriptor.
	 */
	const SharedFD &fd() const
	{
		return fd_;
	}

	/**
	 * \brief Operator bool() for SharedMemObject.
	 *
	 * \return true if the object is not null, false otherwise.
	 */
	explicit operator bool() const
	{
		return !!obj_;
	}

private:
	std::string name_;
	SharedFD fd_;
	T *obj_;
};

} /* namespace libcamera */
