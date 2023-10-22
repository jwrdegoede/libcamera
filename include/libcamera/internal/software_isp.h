/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * software_isp.h - Interface for a software implementation of an ISP
 */

#pragma once

#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/pipeline_handler.h"

namespace libcamera {

class FrameBuffer;
class PixelFormat;
struct StreamConfiguration;

LOG_DECLARE_CATEGORY(SoftwareIsp)

/**
 * \brief Base class for the Software ISP.
 *
 * Base class of the SoftwareIsp interface.
 */
class SoftwareIsp
{
public:
	/**
	 * \brief Constructor for the SoftwareIsp object.
	 * \param[in] pipe The pipeline handler in use.
	 * \param[in] sensorControls The sensor controls.
	 */
	SoftwareIsp(PipelineHandler *pipe, const ControlInfoMap &sensorControls);
	virtual ~SoftwareIsp();

	/**
	 * \brief Load a configuration from a file.
	 * \param[in] filename The file to load from.
	 *
	 * \return 0 on success.
	 */
	virtual int loadConfiguration(const std::string &filename) = 0;

	/**
	 * \brief Gets if there is a valid debayer object.
	 *
	 * \returns true if there is, false otherwise.
	 */
	virtual bool isValid() const = 0;

	/**
	 * \brief Get the supported output formats.
	 * \param[in] input The input format.
	 *
	 * \return all supported output formats or an empty vector if there are none.
	 */
	virtual std::vector<PixelFormat> formats(PixelFormat input) = 0;

	/**
	 * \brief Get the supported output sizes for the given input format and size.
	 * \param[in] inputFormat The input format.
	 * \param[in] inputSize The input size.
	 *
	 * \return The valid size ranges or an empty range if there are none.
	 */
	virtual SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) = 0;

	/**
	 * \brief Get the stride and the frame size.
	 * \param[in] pixelFormat The output format.
	 * \param[in] size The output size.
	 *
	 * \return a tuple of the stride and the frame size, or a tuple with 0,0 if there is no valid output config.
	 */
	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size) = 0;

	/**
	 * \brief Configure the SwIspSimple object according to the passed in parameters.
	 * \param[in] inputCfg The input configuration.
	 * \param[in] outputCfgs The output configurations.
	 *
	 * \return 0 on success, a negative errno on failure.
	 */
	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs) = 0;

	/**
	 * \brief Exports the buffers for use in processing.
	 * \param[in] output The number of outputs requested.
	 * \param[in] count The number of planes.
	 * \param[out] buffers The exported buffers.
	 *
	 * \return count when successful, a negative return value if an error occurred.
	 */
	virtual int exportBuffers(unsigned int output, unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers) = 0;

	/**
	 * \brief Starts the Software ISP worker.
	 *
	 * \return 0 on success, any other value indicates an error.
	 */
	virtual int start() = 0;

	/**
	 * \brief Stops the Software ISP worker.
	 */
	virtual void stop() = 0;

	/**
	 * \brief Queues buffers for processing.
	 * \param[in] input The input framebuffer.
	 * \param[in] outputs The output framebuffers.
	 *
	 * \return 0 on success, a negative errno on failure
	 */
	virtual int queueBuffers(FrameBuffer *input,
				 const std::map<unsigned int, FrameBuffer *> &outputs) = 0;

	/**
	 * \brief Process the statistics gathered.
	 * \param[in] sensorControls The sensor controls.
	 */
	virtual void processStats(const ControlList &sensorControls) = 0; // rather merge with queueBuffers()?

	/**
	 * \brief Get the signal for when the sensor controls are set.
	 *
	 * \return The control list of the sensor controls.
	 */
	virtual Signal<const ControlList &> &getSignalSetSensorControls() = 0;

	/**
	 * \brief Signals that the input buffer is ready.
	 */
	Signal<FrameBuffer *> inputBufferReady;
	/**
	 * \brief Signals that the output buffer is ready.
	 */
	Signal<FrameBuffer *> outputBufferReady;

	/**
	 * \brief Signals that the ISP stats are ready.
	 *
	 * The int parameter isn't actually used.
	 */
	Signal<int> ispStatsReady;
};

/**
 * \brief Base class for the Software ISP Factory.
 *
 * Base class of the SoftwareIsp Factory.
 */
class SoftwareIspFactoryBase
{
public:
	SoftwareIspFactoryBase();
	virtual ~SoftwareIspFactoryBase() = default;

	/**
	 * \brief Creates a SoftwareIsp object.
	 * \param[in] pipe The pipeline handler in use.
	 * \param[in] sensorControls The sensor controls.
	 *
	 * \return An unique pointer to the created SoftwareIsp object.
	 */
	static std::unique_ptr<SoftwareIsp> create(PipelineHandler *pipe,
						   const ControlInfoMap &sensorControls);
	/**
	 * \brief Gives back a pointer to the factory.
	 *
	 * \return A static pointer to the factory instance.
	 */
	static SoftwareIspFactoryBase *&factory();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(SoftwareIspFactoryBase)

	static void registerType(SoftwareIspFactoryBase *factory);
	virtual std::unique_ptr<SoftwareIsp> createInstance(PipelineHandler *pipe,
							    const ControlInfoMap &sensorControls) const = 0;
};

/**
 * \brief Implementation for the Software ISP Factory.
 */
template<typename _SoftwareIsp>
class SoftwareIspFactory : public SoftwareIspFactoryBase
{
public:
	SoftwareIspFactory()
		: SoftwareIspFactoryBase()
	{
	}

	/**
	 * \brief Creates an instance of a SoftwareIsp object.
	 * \param[in] pipe The pipeline handler in use.
	 * \param[in] sensorControls The sensor controls.
	 *
	 * \return An unique pointer to the created SoftwareIsp object.
	 */
	std::unique_ptr<SoftwareIsp> createInstance(PipelineHandler *pipe,
						    const ControlInfoMap &sensorControls) const override
	{
		return std::make_unique<_SoftwareIsp>(pipe, sensorControls);
	}
};

#define REGISTER_SOFTWAREISP(softwareIsp) \
	static SoftwareIspFactory<softwareIsp> global_##softwareIsp##Factory;

} /* namespace libcamera */
