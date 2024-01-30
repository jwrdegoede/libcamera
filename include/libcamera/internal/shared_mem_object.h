/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * shared_mem_object.h - Helper class for shared memory allocations
 */
#pragma once

#include <stddef.h>
#include <string>
#include <sys/mman.h>
#include <utility>

#include <libcamera/base/class.h>
#include <libcamera/base/shared_fd.h>

namespace libcamera {

class SharedMem
{
public:
	SharedMem()
		: mem_(nullptr)
	{
	}

	SharedMem(const std::string &name, std::size_t size);

	SharedMem(SharedMem &&rhs)
	{
		this->name_ = std::move(rhs.name_);
		this->fd_ = std::move(rhs.fd_);
		this->mem_ = rhs.mem_;
		rhs.mem_ = nullptr;
	}

	virtual ~SharedMem()
	{
		if (mem_)
			munmap(mem_, size_);
	}

	/* Make SharedMem non-copyable for now. */
	LIBCAMERA_DISABLE_COPY(SharedMem)

	SharedMem &operator=(SharedMem &&rhs)
	{
		this->name_ = std::move(rhs.name_);
		this->fd_ = std::move(rhs.fd_);
		this->mem_ = rhs.mem_;
		rhs.mem_ = nullptr;
		return *this;
	}

	const SharedFD &fd() const
	{
		return fd_;
	}

	void *mem() const
	{
		return mem_;
	}

private:
	std::string name_;
	SharedFD fd_;
	size_t size_;
protected:
	void *mem_;
};

template<class T>
class SharedMemObject : public SharedMem
{
public:
	static constexpr std::size_t SIZE = sizeof(T);

	SharedMemObject()
		: SharedMem(), obj_(nullptr)
	{
	}

	template<class... Args>
	SharedMemObject(const std::string &name, Args &&...args)
		: SharedMem(name, SIZE), obj_(nullptr)
	{
		if (mem_ == nullptr)
			return;

		obj_ = new (mem_) T(std::forward<Args>(args)...);
	}

	SharedMemObject(SharedMemObject<T> &&rhs)
		: SharedMem(std::move(rhs))
	{
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
	}

	~SharedMemObject()
	{
		if (obj_)
			obj_->~T();
	}

	/* Make SharedMemObject non-copyable for now. */
	LIBCAMERA_DISABLE_COPY(SharedMemObject)

	SharedMemObject<T> &operator=(SharedMemObject<T> &&rhs)
	{
		SharedMem::operator=(std::move(rhs));
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
		return *this;
	}

	T *operator->()
	{
		return obj_;
	}

	const T *operator->() const
	{
		return obj_;
	}

	T &operator*()
	{
		return *obj_;
	}

	const T &operator*() const
	{
		return *obj_;
	}

	explicit operator bool() const
	{
		return !!obj_;
	}

private:
	T *obj_;
};

} /* namespace libcamera */
