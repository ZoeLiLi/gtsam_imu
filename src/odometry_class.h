#ifndef ODOMETRY_CLASS_H
#define ODOMETRY_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"
#include "velocity_factor.h"



namespace TADR
{
class OdometryPara
{
public:
	OdometryPara(boost::shared_ptr<SensorFactors> sensor_factor);
	virtual ~OdometryPara();

public:
	void SetVehicleData(VehicleData vehicle_data);
	VehicleData GetVehicleData();
	void UpdateInitialValue();
private:
	void GenerateVelocityFactor();
private:
	bool										initialed_;
	gtsam::Vector3								velocity_sigma_;
	gtsam::SharedNoiseModel						velocity_noise_model_;
	boost::shared_ptr<SensorFactors>			sensor_factors_;
	boost::thread*								velocity_thread_;
	boost::mutex								mutex_;
	boost::condition							condition_;

	VehicleData									vehicle_data_;


};
}
#endif
