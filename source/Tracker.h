#pragma once

#define _USE_MATH_DEFINES
#include <openvr_driver.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include "Serial.h"

using namespace std::chrono;
using namespace vr;

struct Vec3f {
	float x;
	float y;
	float z;
};

struct RotateInfo {
	float pitch;
	float roll;
	float yaw;
	Vec3f gyro;
	Vec3f acc;
	float temp;
};

class Tracker {
public:
	Tracker(DriverPose_t init_pose);
	~Tracker();
	DriverPose_t GetPose();
	void Process();

private:
	bool GetRotateInfo(RotateInfo& rotate_info);
	float DegreeToRadian(float degree);
	void SubOffset();

	std::mutex mtx_;
	std::thread* thread_ = nullptr;
	DriverPose_t pose_;
	Serial* serial_ = nullptr;

	int port_num_ = 3;
	unsigned long baud_rate_ = CBR_115200;
	RotateInfo rotate_info_, offset_;
};