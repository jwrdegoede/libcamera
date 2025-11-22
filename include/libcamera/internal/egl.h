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
 * \class eGLImage
 * \brief Helper class for managing EGL image resources
 *
 * The eGLImage class encapsulates OpenGL ES texture and framebuffer objects
 * along with their associated EGL image. It aggregates handles, descriptors,
 * and routines for managing textures that can be associated with shader
 * uniform IDs.
 *
 * This class is particularly useful for managing DMA-BUF backed textures
 * in zero-copy rendering pipelines, where textures are bound to specific
 * texture units and can be used as both input textures and render targets.
 */
class eGLImage
{
public:
	/**
	 * \brief Construct an eGLImage with explicit stride
	 * \param[in] width Image width in pixels
	 * \param[in] height Image height in pixels
	 * \param[in] bpp Bytes per pixel
	 * \param[in] stride Row stride in bytes
	 * \param[in] texture_unit OpenGL texture unit (e.g., GL_TEXTURE0)
	 * \param[in] texture_unit_uniform_id Shader uniform ID for this texture unit
	 *
	 * Creates an eGLImage with the specified dimensions and stride. The stride
	 * may differ from width * bpp due to alignment.
	 */
	eGLImage(uint32_t width, uint32_t height, uint32_t bpp, uint32_t stride, GLenum texture_unit, uint32_t texture_unit_uniform_id)
	{
		init(width, height, bpp, stride, texture_unit, texture_unit_uniform_id);
	}

	/**
	 * \brief Construct an eGLImage with automatic stride calculation
	 * \param[in] width Image width in pixels
	 * \param[in] height Image height in pixels
	 * \param[in] bpp Bytes per pixel
	 * \param[in] texture_unit OpenGL texture unit (e.g., GL_TEXTURE0)
	 * \param[in] texture_unit_uniform_id Shader uniform ID for this texture unit
	 *
	 * Creates an eGLImage with automatic stride calculation. The stride is
	 * aligned to 256 bytes because 256 byte alignment is a common baseline alignment for GPUs.
	 */
	eGLImage(uint32_t width, uint32_t height, uint32_t bpp, GLenum texture_unit, uint32_t texture_unit_uniform_id)
	{
		uint32_t stride = libcamera::utils::alignUp(width * bpp / 8, 256);

		init(width, height, bpp, stride, texture_unit, texture_unit_uniform_id);
	}

	/**
	 * \brief Destroy the eGLImage
	 *
	 * Cleans up OpenGL resources by deleting the framebuffer object and
	 * texture.
	 */
	~eGLImage()
	{
		glDeleteFramebuffers(1, &fbo_);
		glDeleteTextures(1, &texture_);
	}

	uint32_t width_;			/**< Image width in pixels */
	uint32_t height_;			/**< Image height in pixels */
	uint32_t stride_;			/**< Row stride in bytes */
	uint32_t offset_;			/**< Buffer offset (reserved for future use) */
	uint32_t framesize_;			/**< Total frame size in bytes (stride * height) */
	uint32_t bpp_;				/**< Bytes per pixel */
	uint32_t texture_unit_uniform_id_;	/**< Shader uniform id for texture unit */
	GLenum texture_unit_;			/**< Texture unit associated with this image eg (GL_TEXTURE0) */
	GLuint texture_;			/**< OpenGL texture object ID */
	GLuint fbo_;				/**< OpenGL frame buffer object ID */
	EGLImageKHR image_;			/**< EGL Image handle */

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGLImage)

	/**
	 * \brief Initialise eGLImage state
	 * \param[in] width Image width in pixels
	 * \param[in] height Image height in pixels
	 * \param[in] bpp Bytes per pixel
	 * \param[in] stride Row stride in bytes
	 * \param[in] texture_unit OpenGL texture unit
	 * \param[in] texture_unit_uniform_id Shader uniform ID
	 *
	 * Common initialisation routine called by both constructors. Sets up
	 * member variables and generates OpenGL texture and framebuffer objects.
	 */
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
 * \class eGL
 * \brief Helper class for managing OpenGL ES operations
 *
 * It provides:
 *
 * - EGL context setup and management
 * - Extension function pointer retrieval
 * - Shader compilation and program linking
 * - DMA-BUF texture creation and management
 * - Synchronisation primitives
 *
 * This class is designed to work with zero-copy buffers via DMA-BUF file
 * descriptors.
 */
class eGL
{
public:
	/**
	 * \brief Construct an EGL helper
	 *
	 * Creates an eGL instance with uninitialised context. Call initEGLContext()
	 * to set up the EGL display, context, and load extension functions.
	 */
	eGL();

	/**
	 * \brief Destroy the EGL helper
	 *
	 * Destroys the EGL context and surface if they were successfully created.
	 */
	~eGL();

	/**
	 * \brief Initialise the EGL context
	 * \param[in] gbmContext Pointer to initialised GBM context
	 *
	 * Sets up the EGL display from the GBM device, creates an OpenGL ES 2.0
	 * context, and retrieves function pointers for required extensions
	 * including:
	 * - eglCreateImageKHR / eglDestroyImageKHR
	 * - glEGLImageTargetTexture2DOES
	 * - eglCreateSyncKHR / eglDestroySyncKHR / eglClientWaitSyncKHR
	 *
	 * \return 0 on success, or -ENODEV on failure
	 */
	int initEGLContext(GBM *gbmContext);

	/**
	 * \brief Clean up EGL resources
	 *
	 * Destroys the EGL sync object. Must be called from the same thread
	 * that created the EGL context.
	 */
	void cleanUp();

	/**
	 * \brief Create an input DMA-BUF backed texture
	 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
	 * \param[in] fd DMA-BUF file descriptor
	 *
	 * Creates an EGL image from a DMA-BUF file descriptor and binds it to
	 * a 2D texture for use as an input texture in shaders. The texture is
	 * configured with nearest filtering and clamp-to-edge wrapping.
	 *
	 * \return 0 on success, or -ENODEV on failure
	 */
	int createInputDMABufTexture2D(eGLImage *eglImage, int fd);

	/**
	 * \brief Create an output DMA-BUF backed texture
	 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
	 * \param[in] fd DMA-BUF file descriptor
	 *
	 * Creates an EGL image from a DMA-BUF file descriptor and binds it to
	 * a 2D texture, then attaches it to a framebuffer object for use as a
	 * render target. This enables zero-copy rendering directly to the
	 * DMA-BUF.
	 *
	 * \return 0 on success, or -ENODEV on failure
	 */
	int createOutputDMABufTexture2D(eGLImage *eglImage, int fd);

	/**
	 * \brief Destroy a DMA-BUF texture's EGL image
	 * \param[in,out] eglImage EGL image to destroy
	 *
	 * Destroys the EGL image associated with a DMA-BUF texture. The OpenGL
	 * texture and framebuffer objects are destroyed separately in the
	 * eGLImage destructor.
	 */
	void destroyDMABufTexture(eGLImage *eglImage);

	/**
	 * \brief Create a 2D texture from a memory buffer
	 * \param[in,out] eglImage EGL image to associate with the texture
	 * \param[in] format OpenGL internal format (e.g., GL_RGB, GL_RGBA)
	 * \param[in] width Texture width in pixels
	 * \param[in] height Texture height in pixels
	 * \param[in] data Pointer to pixel data, or nullptr for uninitialised texture
	 *
	 * Creates a 2D texture from a CPU-accessible memory buffer. The texture
	 * is configured with nearest filtering and clamp-to-edge wrapping. This
	 * is useful for uploading static data like lookup tables or uniform color
	 * matrices to the GPU.
	 */
	void createTexture2D(eGLImage *eglImage, GLint format, uint32_t width, uint32_t height, void *data);

	/**
	 * \brief Create a 1D texture from a memory buffer
	 * \param[in,out] eglImage EGL image to associate with the texture
	 * \param[in] format OpenGL internal format
	 * \param[in] width Texture width in pixels
	 * \param[in] data Pointer to pixel data
	 *
	 * Creates a 1D texture (implemented as a 2D texture with height=1) from
	 * a CPU-accessible memory buffer. Useful for lookup tables in shaders.
	 */
	void createTexture1D(eGLImage *eglImage, GLint format, uint32_t width, void *data);

	/**
	 * \brief Add a preprocessor definition to shader environment
	 * \param[in,out] shaderEnv Vector of shader environment strings
	 * \param[in] str Preprocessor definition string (e.g., "#define APPLY_RGB_PARAMETERS")
	 *
	 * Appends a preprocessor definition to the shader environment vector.
	 * These definitions are prepended to shader source code during compilation.
	 */
	void pushEnv(std::vector<std::string> &shaderEnv, const char *str);

	/**
	 * \brief Make the EGL context current for the calling thread
	 *
	 * Binds the EGL context to the current thread, allowing OpenGL ES
	 * operations to be performed. Must be called from the thread that
	 * will perform rendering operations.
	 */
	void makeCurrent();

	/**
	 * \brief Compile a vertex shader
	 * \param[out] shaderId OpenGL shader object ID
	 * \param[in] shaderData Pointer to shader source code
	 * \param[in] shaderDataLen Length of shader source in bytes
	 * \param[in] shaderEnv Span of preprocessor definitions to prepend
	 *
	 * Compiles a vertex shader from source code with optional preprocessor
	 * definitions. On compilation failure, logs the shader info log.
	 *
	 * \return 0 on success, or -EINVAL on compilation failure
	 */
	int compileVertexShader(GLuint &shaderId, unsigned char *shaderData,
				unsigned int shaderDataLen,
				Span<const std::string> shaderEnv);

	/**
	 * \brief Compile a fragment shader
	 * \param[out] shaderId OpenGL shader object ID
	 * \param[in] shaderData Pointer to shader source code
	 * \param[in] shaderDataLen Length of shader source in bytes
	 * \param[in] shaderEnv Span of preprocessor definitions to prepend
	 *
	 * Compiles a fragment shader from source code with optional preprocessor
	 * definitions. On compilation failure, logs the shader info log.
	 *
	 * \return 0 on success, or -EINVAL on compilation failure
	 */
	int compileFragmentShader(GLuint &shaderId, unsigned char *shaderData,
				  unsigned int shaderDataLen,
				  Span<const std::string> shaderEnv);

	/**
	 * \brief Link a shader program
	 * \param[out] programId OpenGL program object ID
	 * \param[in] fragmentshaderId Compiled fragment shader ID
	 * \param[in] vertexshaderId Compiled vertex shader ID
	 *
	 * Links vertex and fragment shaders into an executable shader program.
	 * On link failure, logs the program info log and deletes the program.
	 *
	 * \return 0 on success, or -ENODEV on link failure
	 */
	int linkProgram(GLuint &programId, GLuint fragmentshaderId, GLuint vertexshaderId);

	/**
	 * \brief Dump shader source code to the log
	 * \param[in] shaderId OpenGL shader object ID
	 *
	 * Retrieves and logs the complete source code of a compiled shader.
	 * Useful for debugging shader compilation issues.
	 */
	void dumpShaderSource(GLuint shaderId);

	/**
	 * \brief Activate a shader program for rendering
	 * \param[in] programId OpenGL program object ID
	 *
	 * Sets the specified program as the current rendering program. All
	 * subsequent draw calls will use this program's shaders.
	 */
	void useProgram(GLuint programId);

	/**
	 * \brief Delete a shader program
	 * \param[in] programId OpenGL program object ID
	 *
	 * Deletes a shader program and frees associated resources. The program
	 * must not be currently in use.
	 */
	void deleteProgram(GLuint programId);

	/**
	 * \brief Synchronise rendering output
	 *
	 * Flushes OpenGL commands and waits for rendering to complete using an
	 * EGL fence sync object. This ensures all rendering operations have
	 * finished before the CPU accesses the output buffers.
	 *
	 * \return 0 on success
	 */
	int syncOutput();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGL)

	int fd_;		/**< File descriptor \todo remove this */
	pid_t tid_;		/**< Thread ID of the thread associated with this EGL context */

	EGLDisplay display_;	/**< EGL display handle */
	EGLContext context_;	/**< EGL context handle */
	EGLSurface surface_;	/**< EGL sufrace handle */
	EGLSyncKHR sync_;	/**< EGL sync object for output sychonisation */

	/**
	 * \brief Compile a shader of specified type
	 * \param[in] shaderType GL_VERTEX_SHADER or GL_FRAGMENT_SHADER
	 * \param[out] shaderId OpenGL shader object ID
	 * \param[in] shaderData Pointer to shader source code
	 * \param[in] shaderDataLen Length of shader source in bytes
	 * \param[in] shaderEnv Span of preprocessor definitions to prepend
	 *
	 * Internal helper function for shader compilation. Prepends environment
	 * definitions to the shader source and compiles the shader.
	 *
	 * \return 0 on success, or -EINVAL on compilation failure
	 */
	int compileShader(int shaderType, GLuint &shaderId, unsigned char *shaderData,
			  unsigned int shaderDataLen,
			  Span<const std::string> shaderEnv);

	/**
	 * \brief Create a DMA-BUF backed 2D texture
	 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
	 * \param[in] fd DMA-BUF file descriptor
	 * \param[in] output If true, create framebuffer for render target
	 *
	 * Internal implementation for creating DMA-BUF textures. Creates an EGL
	 * image from the DMA-BUF and binds it to a 2D texture. If output is true,
	 * also creates and attaches a framebuffer object.
	 *
	 * \return 0 on success, or -ENODEV on failure
	 */
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
