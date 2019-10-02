#pragma once

#include <openvr_driver.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

using namespace std::chrono;
using namespace vr;

class Tracker {
public:
	Tracker(DriverPose_t init_pose);
	~Tracker();
	DriverPose_t GetPose();
	void Process();

private:
	std::mutex mtx_;
	std::thread* thread_ = nullptr;
	DriverPose_t pose_;
};