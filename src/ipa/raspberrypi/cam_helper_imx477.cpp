/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx477.cpp - camera helper for imx477 sensor
 */

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include "cam_helper.hpp"
#include "md_parser.hpp"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t temperatureReg = 0x013a;
constexpr std::initializer_list<uint32_t> registerList =
	{ expHiReg, expLoReg, gainHiReg, gainLoReg, frameLengthHiReg, frameLengthLoReg, temperatureReg  };

class CamHelperImx477 : public CamHelper
{
public:
	CamHelperImx477();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	uint32_t getVBlanking(Duration &exposure, Duration minFrameDuration,
			      Duration maxFrameDuration) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
		       int &vblankDelay) const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22;
	/* Maximum frame length allowable for long exposure calculations. */
	static constexpr int frameLengthMax = 0xffdc;
	/* Largest long exposure scale factor given as a left shift on the frame length. */
	static constexpr int longExposureShiftMax = 7;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperImx477::CamHelperImx477()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperImx477::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperImx477::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

void CamHelperImx477::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	/*
	 * The DeviceStatus struct is first populated with values obtained from
	 * DelayedControls. If this reports frame length is > frameLengthMax,
	 * it means we are using a long exposure mode. Since the long exposure
	 * scale factor is not returned back through embedded data, we must rely
	 * on the existing exposure lines and frame length values returned by
	 * DelayedControls.
	 *
	 * Otherwise, all values are updated with what is reported in the
	 * embedded data.
	 */
	if (deviceStatus.frameLength > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.shutterSpeed = deviceStatus.shutterSpeed;
		parsedDeviceStatus.frameLength = deviceStatus.frameLength;
		metadata.set("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
}

uint32_t CamHelperImx477::getVBlanking(Duration &exposure,
				       Duration minFrameDuration,
				       Duration maxFrameDuration) const
{
	uint32_t frameLength, exposureLines;
	unsigned int shift = 0;

	frameLength = mode_.height + CamHelper::getVBlanking(exposure, minFrameDuration,
							     maxFrameDuration);
	/*
	 * Check if the frame length calculated needs to be setup for long
	 * exposure mode. This will require us to use a long exposure scale
	 * factor provided by a shift operation in the sensor.
	 */
	while (frameLength > frameLengthMax) {
		if (++shift > longExposureShiftMax) {
			shift = longExposureShiftMax;
			frameLength = frameLengthMax;
			break;
		}
		frameLength >>= 1;
	}

	if (shift) {
		/* Account for any rounding in the scaled frame length value. */
		frameLength <<= shift;
		exposureLines = CamHelperImx477::exposureLines(exposure);
		exposureLines = std::min(exposureLines, frameLength - frameIntegrationDiff);
		exposure = CamHelperImx477::exposure(exposureLines);
	}

	return frameLength - mode_.height;
}

void CamHelperImx477::getDelays(int &exposureDelay, int &gainDelay,
				int &vblankDelay) const
{
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 3;
}

bool CamHelperImx477::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperImx477::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg));
	deviceStatus.analogueGain = gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
	deviceStatus.sensorTemperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperImx477();
}

static RegisterCamHelper reg("imx477", &create);
