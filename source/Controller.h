#pragma once

#include <openvr_driver.h>
#include <algorithm>
#include <chrono>
#include <cmath>
using namespace std::chrono;
using namespace vr;

class Controller : public ITrackedDeviceServerDriver
{
public:
	Controller();
	Controller(std::string serial, bool side, DriverPose_t initial_pose, VRControllerState_t initial_state);
	virtual ~Controller();

	virtual void updateControllerState(VRControllerState_t new_state);
	virtual void updateControllerPose(DriverPose_t new_pose);
	virtual uint32_t getObjectID();

	/*
		Inherited from ITrackedDeviceServerDriver:
	*/

	EVRInitError Activate(uint32_t unObjectId) override;
	void Deactivate() override;
	void EnterStandby() override;
	void* GetComponent(const char* pchComponentNameAndVersion) override;
	void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	DriverPose_t GetPose() override;
	void RunFrame();


private:
	VRControllerState_t controller_state_;
	DriverPose_t controller_pose_;
	uint32_t object_id_;
	std::string serial_;
	bool side_;
};