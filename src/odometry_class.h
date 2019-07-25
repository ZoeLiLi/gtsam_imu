#ifndef ODOMETRY_CLASS_H
#define ODOMETRY_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "factors/velocity_factor.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


namespace TADR
{
class OdometryPara
{
public:
	OdometryPara();
	virtual ~OdometryPara();

public:
	void SetVehicleData(VehicleData vehicle_data);
	VehicleData GetVehicleData();
	void UpdateInitialValue();
	void AddSpeedFactor();
	gtsam::NonlinearFactorGraph GetOdometryFactors();
	void Reset();
private:
	void GenerateVelocityFactor();
private:
	bool										initialed_;
	gtsam::Vector3								velocity_sigma_;
	gtsam::SharedNoiseModel						velocity_noise_model_;
	boost::thread*								velocity_thread_;
	boost::mutex								mutex_;
	boost::condition							condition_;
	gtsam::NonlinearFactorGraph					odometry_factors_;
	VehicleData									vehicle_data_;
	double										speed_;
	int											index_;

};
}
#endif
