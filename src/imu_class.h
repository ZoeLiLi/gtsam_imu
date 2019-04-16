#ifndef IMU_CLASS_H
#define IMU_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>


namespace TADR
{
class IMUPara
{
public:
	IMUPara(boost::shared_ptr<SensorFactors> sensor_factor);
	IMUPara(boost::shared_ptr<SensorFactors> sensor_factor,int frequency);
	virtual ~IMUPara();
public:
	void SetIMUData(ImuData imu_data);
	ImuData GetIMUData();
	void UpdateInitialValue();
	void UpdatePreIntegration();
	void UpdatePreIntegration(gtsam::imuBias::ConstantBias bias);
	void AddImuFactor();
private:
	void GenerateIMUFactor();

private:

	gtsam::NonlinearFactor::shared_ptr			imu_factor_;
	gtsam::NonlinearFactor::shared_ptr			bias_factor_;
	gtsam::PreintegratedCombinedMeasurements*	preintegrated_;
	gtsam::Vector3								gyro_bias_;
	gtsam::Vector3								acc_bias_;
	gtsam::Vector6								imu_bias_sigma_;
	boost::shared_ptr<SensorFactors>			sensor_factors_;
	gtsam::imuBias::ConstantBias				prior_imu_bias_;
	boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>		imu_params_;

	ImuData										imu_data_;
	bool										initialed_;
	int											frequency_;
	gtsam::Vector3								gn_;
	int											imu_count_;

	boost::thread*								imu_thread_;
	boost::mutex								mutex_;
	boost::condition							condition_;
};
}
#endif
