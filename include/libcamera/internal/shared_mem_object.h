/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * shared_mem_object.h - Helper class for shared memory allocations
 */
#pragma once

#include <cstddef>
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

	~SharedMem()
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
	void *mem_;
};

template<class T>
class SharedMemObject
{
public:
	static constexpr std::size_t SIZE = sizeof(T);

	SharedMemObject()
		: obj_(nullptr)
	{
	}

	template<class... Args>
	SharedMemObject(const std::string &name, Args &&...args)
		: shMem_(name, SIZE), obj_(nullptr)
	{
		void *mem = shMem_.mem();

		if (mem == nullptr)
			return;

		obj_ = new (mem) T(std::forward<Args>(args)...);
	}

	SharedMemObject(SharedMemObject<T> &&rhs)
	{
		this->shMem_ = std::move(rhs.shMem_);
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
		this->shMem_ = std::move(rhs.shMem_);
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

	const SharedFD &fd() const
	{
		return shMem_.fd();
	}

	explicit operator bool() const
	{
		return !!obj_;
	}

private:
	SharedMem shMem_;
	T *obj_;
};

} /* namespace libcamera */
