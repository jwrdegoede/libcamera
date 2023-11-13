/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * soft-base.h - Software IPA base class
 */
#pragma once

#include <libcamera/base/shared_fd.h>
#include <libcamera/controls.h>

#include <libcamera/ipa/soft_ipa_interface.h>

namespace libcamera {

namespace ipa::soft {

class IPASoftBase : public ipa::soft::IPASoftInterface
{
public:
	IPASoftBase();
	~IPASoftBase();

	int init(const IPASettings &settings,
		 const SharedFD &fdStats,
		 const ControlInfoMap &sensorInfoMap) override;
	int configure(const ControlInfoMap &sensorInfoMap) override;

	int start() override;
	void stop() override;

	void processStats(const ControlList &sensorControls) override;

protected:
	SharedFD fdStats_;

private:
	virtual int platformInit(const ControlInfoMap &sensorInfoMap) = 0;
	virtual int platformConfigure(const ControlInfoMap &sensorInfoMap) = 0;
	virtual int platformStart() = 0;
	virtual void platformStop() = 0;
	virtual void platformProcessStats(const ControlList &sensorControls) = 0;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
