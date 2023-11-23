/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_simple.h - Simple software ISP implementation
 */

#pragma once

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_proxy.h>

#include "libcamera/internal/software_isp.h"
#include "libcamera/internal/software_isp/debayer_cpu.h"

namespace libcamera {

/**
 * \brief Class for the Simple Software ISP.
 *
 * Implementation of the SoftwareIsp interface.
 */
class SwIspSimple : public SoftwareIsp
{
public:
	/**
	 * \brief Constructor for the SwIspSimple object.
	 *
	 * \param[in] pipe The pipeline handler in use.
	 * \param[in] sensorControls The sensor controls.
	 */
	SwIspSimple(PipelineHandler *pipe, const ControlInfoMap &sensorControls);
	~SwIspSimple() {}

	/**
	 * \brief Load a configuration from a file.
	 * \param[in] filename The file to load from.
	 *
	 * \return 0 on success.
	 */
	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }

	/**
	 * \brief Gets if there is a valid debayer object.
	 *
	 * \returns true if there is, false otherwise.
	 */
	bool isValid() const;

	/**
	 * \brief Get the supported output formats.
	 * \param[in] input The input format.
	 *
	 * \return all supported output formats or an empty vector if there are none.
	 */
	std::vector<PixelFormat> formats(PixelFormat input);

	/**
	 * \brief Get the supported output sizes for the given input format and size.
	 * \param[in] inputFormat The input format.
	 * \param[in] inputSize The input size.
	 *
	 * \return The valid size ranges or an empty range if there are none.
	 */
	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);

	/**
	 * \brief Get the stride and the frame size.
	 * \param[in] outputFormat The output format.
	 * \param[in] size The output size.
	 *
	 * \return a tuple of the stride and the frame size, or a tuple with 0,0 if there is no valid output config.
	 */
	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);

	/**
	 * \brief Configure the SwIspSimple object according to the passed in parameters.
	 * \param[in] inputCfg The input configuration.
	 * \param[in] outputCfgs The output configurations.
	 *
	 * \return 0 on success, a negative errno on failure.
	 */
	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs);

	/**
	 * \brief Exports the buffers for use in processing.
	 * \param[in] output The number of outputs requested.
	 * \param[in] count The number of planes.
	 * \param[out] buffers The exported buffers.
	 *
	 * \return count when successful, a negative return value if an error occurred.
	 */
	int exportBuffers(unsigned int output, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	/**
	 * \brief Process the statistics gathered.
	 * \param[in] sensorControls The sensor controls.
	 */
	void processStats(const ControlList &sensorControls);

	/**
	 * \brief Starts the Software ISP worker.
	 *
	 * \return 0 on success, any other value indicates an error.
	 */
	int start();

	/**
	 * \brief Stops the Software ISP worker.
	 */
	void stop();

	/**
	 * \brief Queues buffers for processing.
	 * \param[in] input The input framebuffer.
	 * \param[in] outputs The output framebuffers.
	 *
	 * \return 0 on success, a negative errno on failure
	 */
	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

	/**
	 * \brief Get the signal for when the sensor controls are set.
	 *
	 * \return The control list of the sensor controls.
	 */
	Signal<const ControlList &> &getSignalSetSensorControls();

	/**
	 * \brief Process the input framebuffer.
	 * \param[in] input The input framebuffer.
	 * \param[out] output The output framebuffer.
	 */
	void process(FrameBuffer *input, FrameBuffer *output);

private:
	void saveIspParams(int dummy);
	void statsReady(int dummy);
	void inputReady(FrameBuffer *input);
	void outputReady(FrameBuffer *output);

	std::unique_ptr<DebayerCpu> debayer_;
	Thread ispWorkerThread_;
	SharedMemObject<DebayerParams> sharedParams_;
	DebayerParams debayerParams_;

	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
};

} /* namespace libcamera */
