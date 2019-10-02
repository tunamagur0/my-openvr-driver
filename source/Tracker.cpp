#include"Tracker.h"

Tracker::Tracker(DriverPose_t init_pose) :
	pose_(init_pose)
{
	thread_ = new std::thread(&Tracker::Process, this);
}

Tracker::~Tracker()
{
	thread_->join();
	delete thread_;
	thread_ = nullptr;
}

DriverPose_t Tracker::GetPose()
{
	mtx_.lock();
	DriverPose_t pose = pose_;
	mtx_.unlock();
	return pose;
}

void Tracker::Process()
{
	double angle = 0;
	milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

	while (true) {
		mtx_.lock();
		milliseconds deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
		lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

		DriverPose_t pose_next = pose_;
		DriverPose_t pose_previous = pose_next;

		pose_next.vecPosition[0] = 0.5 * std::sin(angle);
		pose_next.vecPosition[1] = 0.5 * -std::abs(std::cos(angle));
		pose_next.vecVelocity[0] = (pose_next.vecPosition[0] - pose_previous.vecPosition[0]) * 1000 / std::max((int)deltaTime.count(), 1);
		pose_next.vecVelocity[1] = (pose_next.vecPosition[1] - pose_previous.vecPosition[1]) * 1000 / std::max((int)deltaTime.count(), 1);

		pose_ = pose_next;

		angle += 0.001;
		mtx_.unlock();

	}
}