#include"Tracker.h"

Tracker::Tracker(DriverPose_t init_pose) :
	pose_(init_pose)
{

	serial_ = new Serial(port_num_);
	serial_->Start(baud_rate_);

	thread_ = new std::thread(&Tracker::Process, this);
	GetRotateInfo(offset_);
}

Tracker::~Tracker()
{
	thread_->join();
	delete thread_;
	thread_ = nullptr;

	delete serial_;
	serial_ = nullptr;
}

DriverPose_t Tracker::GetPose()
{
	mtx_.lock();
	DriverPose_t pose = pose_;
	mtx_.unlock();
	return pose;
}

bool Tracker::GetRotateInfo(RotateInfo& rotate_info)
{
	char buffer[1024];
	serial_->Read(buffer);

	int read_num = sscanf_s(buffer, "g:%f,%f,%f a:%f,%f,%f p:%f r:%f y:%f t:%f",
		&rotate_info.gyro.x, &rotate_info.gyro.y, &rotate_info.gyro.z, &rotate_info.acc.x, &rotate_info.acc.y, &rotate_info.acc.z,
		&rotate_info.pitch, &rotate_info.roll, &rotate_info.yaw, &rotate_info.temp);

	//char text[1024];
	//sprintf_s(text, "%d\n", read_num);
	//MessageBox(NULL, TEXT(text), TEXT("error"), MB_OK);

	return read_num > 0;
}

float Tracker::DegreeToRadian(float degree)
{
	return degree * (M_PI / 180.0);
}

void Tracker::SubOffset()
{
	rotate_info_.gyro.x -= offset_.gyro.x;
	rotate_info_.gyro.y -= offset_.gyro.y;
	rotate_info_.gyro.z -= offset_.gyro.z;
	rotate_info_.acc.x -= offset_.acc.x;
	rotate_info_.acc.y -= offset_.acc.y;
	rotate_info_.acc.z -= offset_.acc.z;
	rotate_info_.pitch -= offset_.pitch;
	rotate_info_.roll -= offset_.roll;
	rotate_info_.yaw -= offset_.yaw;
	rotate_info_.temp -= offset_.temp;
}

void Tracker::Process()
{
	double angle = 0;
	milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	while (true) {

		if (GetRotateInfo(rotate_info_)) {
			SubOffset();
			mtx_.lock();
			milliseconds deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
			lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

			DriverPose_t pose_next = pose_;
			DriverPose_t pose_previous = pose_next;

			pose_next.qRotation.w = cos(DegreeToRadian(rotate_info_.yaw) * 0.5) * cos(DegreeToRadian(rotate_info_.roll) * 0.5) * cos(DegreeToRadian(rotate_info_.pitch) * 0.5) + sin(DegreeToRadian(rotate_info_.yaw) * 0.5) * sin(DegreeToRadian(rotate_info_.roll) * 0.5) * sin(DegreeToRadian(rotate_info_.pitch) * 0.5);
			pose_next.qRotation.x = cos(DegreeToRadian(rotate_info_.yaw) * 0.5) * sin(DegreeToRadian(rotate_info_.roll) * 0.5) * cos(DegreeToRadian(rotate_info_.pitch) * 0.5) - sin(DegreeToRadian(rotate_info_.yaw) * 0.5) * cos(DegreeToRadian(rotate_info_.roll) * 0.5) * sin(DegreeToRadian(rotate_info_.pitch) * 0.5);
			pose_next.qRotation.y = cos(DegreeToRadian(rotate_info_.yaw) * 0.5) * cos(DegreeToRadian(rotate_info_.roll) * 0.5) * sin(DegreeToRadian(rotate_info_.pitch) * 0.5) + sin(DegreeToRadian(rotate_info_.yaw) * 0.5) * sin(DegreeToRadian(rotate_info_.roll) * 0.5) * cos(DegreeToRadian(rotate_info_.pitch) * 0.5);
			pose_next.qRotation.z = sin(DegreeToRadian(rotate_info_.yaw) * 0.5) * cos(DegreeToRadian(rotate_info_.roll) * 0.5) * cos(DegreeToRadian(rotate_info_.pitch) * 0.5) - cos(DegreeToRadian(rotate_info_.yaw) * 0.5) * sin(DegreeToRadian(rotate_info_.roll) * 0.5) * sin(DegreeToRadian(rotate_info_.pitch) * 0.5);

			//pose_next.vecPosition[0] = 0.5 * std::sin(angle);
			//pose_next.vecPosition[1] = 0.5 * std::abs(std::cos(angle));
			//pose_next.vecVelocity[0] = (pose_next.vecPosition[0] - pose_previous.vecPosition[0]) * 1000 / (std::max)((int)deltaTime.count(), 1);
			//pose_next.vecVelocity[1] = (pose_next.vecPosition[1] - pose_previous.vecPosition[1]) * 1000 / (std::max)((int)deltaTime.count(), 1);

			pose_ = pose_next;

			angle += 0.01;
			mtx_.unlock();
		}
		auto sleep_time = system_clock::now() + milliseconds(1000 / 60);
		std::this_thread::sleep_until(sleep_time);
	}
}