#pragma once

#include <openvr_driver.h>
#include "Controller.h"
#include "HMD.h"

using namespace vr;

class ServerDriver : public IServerTrackedDeviceProvider
{
public:
	ServerDriver();
	~ServerDriver();

	EVRInitError Init(IVRDriverContext* pDriverContext) override;
	void Cleanup() override;
	const char* const* GetInterfaceVersions() override;
	void RunFrame() override;
	bool ShouldBlockStandbyMode() override;
	void EnterStandby() override;
	void LeaveStandby() override;

private:
	Controller* controller_left = nullptr;
	Controller* controller_right = nullptr;
	HMD* hmd = nullptr;
};