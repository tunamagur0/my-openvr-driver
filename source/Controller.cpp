#include "Controller.h"

Controller::Controller()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

Controller::Controller(std::string serial, bool side, DriverPose_t initial_pose, VRControllerState_t initial_state) :
	serial_(serial),
	side_(side),
	controller_pose_(initial_pose),
	controller_state_(initial_state)
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

Controller::~Controller()
{
}

void Controller::updateControllerState(VRControllerState_t new_state)
{
	controller_state_ = new_state;
}

void Controller::updateControllerPose(DriverPose_t new_pose)
{
	controller_pose_ = new_pose;
}

uint32_t Controller::getObjectID()
{
	return object_id_;
}


EVRInitError Controller::Activate(uint32_t unObjectId)
{
	object_id_ = unObjectId;

	PropertyContainerHandle_t prop_handle = VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

	VRProperties()->SetBoolProperty(prop_handle, vr::Prop_WillDriftInYaw_Bool, false);
	VRProperties()->SetBoolProperty(prop_handle, vr::Prop_DeviceIsWireless_Bool, true);
	VRProperties()->SetBoolProperty(prop_handle, vr::Prop_HasControllerComponent_Bool, true);

	if (side_) {
		VRProperties()->SetInt32Property(prop_handle, vr::Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_RightHand);
	}
	else {
		VRProperties()->SetInt32Property(prop_handle, vr::Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_LeftHand);
	}

	VRProperties()->SetInt32Property(prop_handle, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
	VRProperties()->SetInt32Property(prop_handle, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);

	VRProperties()->SetStringProperty(prop_handle, Prop_SerialNumber_String, serial_.c_str());
	VRProperties()->SetStringProperty(prop_handle, Prop_ModelNumber_String, "Vive Controller MV");
	VRProperties()->SetStringProperty(prop_handle, Prop_RenderModelName_String, "vr_controller_vive_1_5");
	VRProperties()->SetStringProperty(prop_handle, Prop_ManufacturerName_String, "HTC");

	uint64_t available_buttons = vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
		vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad) |
		vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger) |
		vr::ButtonMaskFromId(vr::k_EButton_System) |
		vr::ButtonMaskFromId(vr::k_EButton_Grip);

	vr::VRProperties()->SetUint64Property(prop_handle, Prop_SupportedButtons_Uint64, available_buttons);

	return EVRInitError::VRInitError_None;
}

void Controller::Deactivate()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Controller::EnterStandby()
{
}

void* Controller::GetComponent(const char* pchComponentNameAndVersion)
{
	return NULL;
}

void Controller::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

DriverPose_t Controller::GetPose()
{
	return controller_pose_;
}

void Controller::RunFrame()
{
	static double angle = 0;
	angle += 0.01;

	static milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	milliseconds deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
	lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

	DriverPose_t pose_next = GetPose();
	DriverPose_t pose_previous = pose_next;

	pose_next.vecPosition[0] = 0.5 * std::sin(angle);
	pose_next.vecPosition[2] = 0.5 * -std::abs(std::cos(angle));
	pose_next.vecVelocity[0] = (pose_next.vecPosition[0] - pose_previous.vecPosition[0]) * 1000 / std::max((int)deltaTime.count(), 1);
	pose_next.vecVelocity[2] = (pose_next.vecPosition[2] - pose_previous.vecPosition[2]) * 1000 / std::max((int)deltaTime.count(), 1);

	updateControllerPose(pose_next);
	VRServerDriverHost()->TrackedDevicePoseUpdated(getObjectID(), GetPose(), sizeof(DriverPose_t));
}