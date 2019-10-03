#include "ServerDriver.h"

ServerDriver::ServerDriver()
{
}

ServerDriver::~ServerDriver()
{

}

EVRInitError ServerDriver::Init(IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	DriverPose_t test_pose = { 0 };
	test_pose.deviceIsConnected = true;
	test_pose.poseIsValid = true;
	test_pose.willDriftInYaw = false;
	test_pose.shouldApplyHeadModel = false;
	test_pose.poseTimeOffset = 0;
	test_pose.result = ETrackingResult::TrackingResult_Running_OK;
	test_pose.qDriverFromHeadRotation = { 1,0,0,0 };
	test_pose.qWorldFromDriverRotation = { 1,0,0,0 };

	VRControllerState_t test_state;
	test_state.ulButtonPressed = test_state.ulButtonTouched = 0;

	//controller_left = new Controller("controller1", false, test_pose, test_state);
	//controller_right = new Controller("controller2", true, test_pose, test_state);

	//VRServerDriverHost()->TrackedDeviceAdded("controller1", vr::TrackedDeviceClass_Controller, controller_left);
	//VRServerDriverHost()->TrackedDeviceAdded("controller2", vr::TrackedDeviceClass_Controller, controller_right);

	hmd = new HMD("hmd", test_pose);
	VRServerDriverHost()->TrackedDeviceAdded("hmd", vr::TrackedDeviceClass_HMD, hmd);
	return EVRInitError::VRInitError_None;
}

void ServerDriver::Cleanup()
{
	delete controller_left;
	controller_left = nullptr;
	delete controller_right;
	controller_right = nullptr;
}

const char* const* ServerDriver::GetInterfaceVersions()
{
	return k_InterfaceVersions;
}

void ServerDriver::RunFrame()
{
	if (controller_left) {
		controller_left->RunFrame();
	}
	if (controller_right) {
		controller_right->RunFrame();
	}
	if (hmd) {
		hmd->RunFrame();
	}
}

bool ServerDriver::ShouldBlockStandbyMode()
{
	return false;
}

void ServerDriver::EnterStandby()
{
}

void ServerDriver::LeaveStandby()
{
}