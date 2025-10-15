/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * egl_context.cpp - Helper class for managing eGL interactions.
 */

#pragma once

#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/base/utils.h"
#include "libcamera/internal/gbm.h"

#define EGL_EGLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#define GL_GLEXT_PROTOTYPES
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(eGL)

/**
 * \brief Helper class for an eGLImage.
 *
 * There are a few handles, descriptors and routines that it makes sense to
 * aggregate together. For example generating a GL texture and
 * frame-buffer-object and being about to assocate that with the
 * texture_uniform_id for a given shader.
 *
 */
class eGLImage
{
public:
	eGLImage(uint32_t width, uint32_t height, uint32_t bpp, uint32_t stride, GLenum texture_unit, uint32_t texture_unit_uniform_id)
	{
		init(width, height, bpp, stride, texture_unit, texture_unit_uniform_id);
	}

	eGLImage(uint32_t width, uint32_t height, uint32_t bpp, GLenum texture_unit, uint32_t texture_unit_uniform_id)
	{
		uint32_t stride = libcamera::utils::alignUp(width * bpp, 256);

		init(width, height, bpp, stride, texture_unit, texture_unit_uniform_id);
	}

	~eGLImage()
	{
		glDeleteFramebuffers(1, &fbo_);
		glDeleteTextures(1, &texture_);
	}

	uint32_t width_;
	uint32_t height_;
	uint32_t stride_;
	uint32_t offset_;
	uint32_t framesize_;
	uint32_t bpp_;
	uint32_t texture_unit_uniform_id_;
	GLenum texture_unit_;
	GLuint texture_;
	GLuint fbo_;
	EGLImageKHR image_;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGLImage)

	void init(uint32_t width, uint32_t height, uint32_t bpp, uint32_t stride, GLenum texture_unit, uint32_t texture_unit_uniform_id)
	{
		image_ = EGL_NO_IMAGE_KHR;
		width_ = width;
		height_ = height;
		bpp_ = bpp;
		stride_ = stride;
		framesize_ = stride_ * height_;
		texture_unit_ = texture_unit;
		texture_unit_uniform_id_ = texture_unit_uniform_id;

		glGenTextures(1, &texture_);
		glGenFramebuffers(1, &fbo_);
	}
};

/**
 * \brief The main eGL object
 *
 * This class buries away eGL specific setup and manipulation.
 *
 * Setting up the eGL context.
 * Getting function pointers.
 * Generating an eGL program from environment variables.
 * Link that program.
 * Generating textures for upload.
 * Render to texture for output.
 *
 */
class eGL
{
public:
	eGL();
	~eGL();

	int initEGLContext(GBM *gbmContext);
	void cleanUp();
	int createInputDMABufTexture2D(eGLImage *eglImage, int fd);
	int createOutputDMABufTexture2D(eGLImage *eglImage, int fd);
	void destroyDMABufTexture(eGLImage *eglImage);
	void createTexture2D(eGLImage *eglImage, GLint format, uint32_t width, uint32_t height, void *data);
	void createTexture1D(eGLImage *eglImage, GLint format, uint32_t width, void *data);

	void pushEnv(std::vector<std::string> &shaderEnv, const char *str);
	void makeCurrent();

	int compileVertexShader(GLuint &shaderId, unsigned char *shaderData,
				unsigned int shaderDataLen,
				Span<const std::string> shaderEnv);
	int compileFragmentShader(GLuint &shaderId, unsigned char *shaderData,
				  unsigned int shaderDataLen,
				  Span<const std::string> shaderEnv);
	int linkProgram(GLuint &programId, GLuint fragmentshaderId, GLuint vertexshaderId);
	void dumpShaderSource(GLuint shaderId);
	void useProgram(GLuint programId);
	void deleteProgram(GLuint programId);
	int syncOutput();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGL)

	int fd_;
	pid_t tid_;

	EGLDisplay display_;
	EGLContext context_;
	EGLSurface surface_;
	EGLSyncKHR sync_;

	int compileShader(int shaderType, GLuint &shaderId, unsigned char *shaderData,
			  unsigned int shaderDataLen,
			  Span<const std::string> shaderEnv);

	int createDMABufTexture2D(eGLImage *eglImage, int fd, bool output);

	PFNEGLEXPORTDMABUFIMAGEMESAPROC eglExportDMABUFImageMESA;
	PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;

	PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
	PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;

	PFNEGLCREATESYNCKHRPROC eglCreateSyncKHR;
	PFNEGLDESTROYSYNCKHRPROC eglDestroySyncKHR;
	PFNEGLCLIENTWAITSYNCKHRPROC eglClientWaitSyncKHR;
};
} //namespace libcamera
