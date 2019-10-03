#include "HMD.h"

HMD::HMD()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

HMD::HMD(std::string serial, DriverPose_t initial_pose) :
	serial_(serial),
	hmd_pose_(initial_pose)
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

HMD::~HMD()
{
}

void HMD::updateHMDPose(DriverPose_t new_pose)
{
	hmd_pose_ = new_pose;
}

uint32_t HMD::getObjectID()
{
	return object_id_;
}

EVRInitError HMD::Activate(uint32_t unObjectId)
{
	object_id_ = unObjectId;
	PropertyContainerHandle_t prop_handle = VRProperties()->TrackedDeviceToPropertyContainer(object_id_);
	VRProperties()->SetFloatProperty(prop_handle, Prop_UserIpdMeters_Float, 0.065f);
	VRProperties()->SetFloatProperty(prop_handle, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
	VRProperties()->SetFloatProperty(prop_handle, Prop_DisplayFrequency_Float, 90.f);
	VRProperties()->SetFloatProperty(prop_handle, Prop_SecondsFromVsyncToPhotons_Float, 0.1f);
	VRProperties()->SetUint64Property(prop_handle, Prop_CurrentUniverseId_Uint64, 2);

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(prop_handle, Prop_IsOnDesktop_Bool, false);

	//Debug mode activate Windowed Mode (borderless fullscreen), lock to 30 FPS 
	vr::VRProperties()->SetBoolProperty(prop_handle, Prop_DisplayDebugMode_Bool, true);

	hmd_pose_.poseIsValid = true;
	hmd_pose_.result = TrackingResult_Running_OK;
	hmd_pose_.deviceIsConnected = true;

	hmd_pose_.qWorldFromDriverRotation = { 1, 0, 0, 0 };
	hmd_pose_.qDriverFromHeadRotation = { 1, 0, 0, 0 };

	tracker_ = new Tracker(hmd_pose_);

	return EVRInitError::VRInitError_None;
}

void HMD::Deactivate()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

void HMD::EnterStandby()
{
}

void* HMD::GetComponent(const char* pchComponentNameAndVersion)
{
	if (0 == strcmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
	{
		return (vr::IVRDisplayComponent*)this;
	}

	return NULL;
}

void HMD::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

DriverPose_t HMD::GetPose()
{
	return hmd_pose_;
}

void HMD::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnX = window_x_;
	*pnY = window_y_;
	*pnWidth = window_width_;
	*pnHeight = window_height_;
}

bool HMD::IsDisplayOnDesktop()
{
	return true;
}

bool HMD::IsDisplayRealDisplay()
{
	return false;
}

void HMD::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnWidth = window_width_;
	*pnHeight = window_height_;
}

void HMD::GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnY = 0;
	*pnWidth = window_width_ / 2;
	*pnHeight = window_height_;

	if (eEye == Eye_Left)
	{
		*pnX = 0;
	}
	else
	{
		*pnX = window_width_ / 2;
	}
}

void HMD::GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
	*pfLeft = -1.0;
	*pfRight = 1.0;
	*pfTop = -1.0;
	*pfBottom = 1.0;
}

DistortionCoordinates_t HMD::ComputeDistortion(EVREye eEye, float fU, float fV)
{
	vr::DistortionCoordinates_t oDistortion{};
	oDistortion.rfBlue[0] = fU;
	oDistortion.rfBlue[1] = fV;
	oDistortion.rfGreen[0] = fU;
	oDistortion.rfGreen[1] = fV;
	oDistortion.rfRed[0] = fU;
	oDistortion.rfRed[1] = fV;
	return oDistortion;
}

void HMD::RunFrame()
{
	if (object_id_ != vr::k_unTrackedDeviceIndexInvalid)
	{
		hmd_pose_ = tracker_->GetPose();
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, GetPose(), sizeof(DriverPose_t));
	}
}