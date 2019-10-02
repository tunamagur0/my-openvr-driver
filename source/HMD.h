#pragma once

#include <openvr_driver.h>
#include "Tracker.h"
using namespace vr;

class HMD : public ITrackedDeviceServerDriver, public IVRDisplayComponent
{
public:
	HMD();
	HMD(std::string serial, DriverPose_t initial_pose);
	~HMD();
	virtual void updateHMDPose(DriverPose_t new_pose);
	virtual uint32_t getObjectID();

	// Inherited via ITrackedDeviceServerDriver
	virtual EVRInitError Activate(uint32_t unObjectId) override;
	virtual void Deactivate() override;
	virtual void EnterStandby() override;
	virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	virtual DriverPose_t GetPose() override;

	// Inherited via IVRDisplayComponent
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
	virtual bool IsDisplayOnDesktop() override;
	virtual bool IsDisplayRealDisplay() override;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override;
	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) override;
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) override;
	void RunFrame();

private:
	TrackedDeviceIndex_t object_id_;
	std::string serial_;

	int window_width_ = 1920;
	int window_height_ = 1080;
	int window_x_ = 0;
	int window_y_ = 0;
	int render_width_ = 1920;
	int render_height_ = 1080;

	DriverPose_t hmd_pose_;
	Tracker* tracker_ = nullptr;
};