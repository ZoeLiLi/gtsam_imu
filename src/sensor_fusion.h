#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H
#include <iostream>
#include <fstream>
#include <string>
#include "imu_class.h"
#include "gnss_class.h"
#include "odometry_class.h"
#include "sensor_factors.h"
#include "alignment.h"
#include "constant.h"

#include <gtsam/nonlinear/ISAM2.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

namespace TADR
{

class SensorFusion
{
public:
	SensorFusion(void);
	SensorFusion(bool fast_replay, std::string triggle_mode,int period = 0);
	virtual ~SensorFusion();
	
public:
	bool Run();
	void FastReplayLog();
	void SetIMUData(ImuData imu_data);
	void SetGnssData(GnssData gnss_data);
private:
	void TimeControl();
	void Process();
	void GetLatestSensorData();

public:
	ImuData								imu_data_;
	GnssData							gnss_data_;
	VehicleData							vehicle_data_;
private:
	bool								exit_;
	bool								initialed_;


	boost::shared_ptr<IMUPara>			imu_para_;
	boost::shared_ptr<GNSSPara>			gnss_para_;
	boost::shared_ptr<OdometryPara>		odometry_para_;
	boost::shared_ptr<SensorFactors>	sensor_factors_;
	boost::shared_ptr<SystemAlignment>	system_alignment_;

	int									values_index_;
	gtsam::NonlinearFactorGraph 		factors_;
	gtsam::Values						values_;
	gtsam::Values						results_;

	gtsam::ISAM2Params 					isam_params_;

	std::string							triggle_mode_;
	int									period_;
	int									window_length_;

	boost::thread*						process_thread_;
	boost::thread*						time_control_thread_;
	boost::mutex						mutex_;
	boost::condition					condition_;

	gtsam::Pose3						current_pose_;
	gtsam::Vector3						current_velocity_;
	gtsam::imuBias::ConstantBias		current_imu_bias_;
	std::ofstream						resultofs_;

	gtsam::Vector3						init_sigma_rotation_;
	gtsam::Vector3						init_sigma_position_;
	gtsam::Vector3						init_sigma_gyro_;
	gtsam::Vector3						init_sigma_acc_;
	gtsam::Vector6						sigma_pose_;
	gtsam::Vector6						sigma_bias_;
	InitSigmaState						init_sigma_state_;

};
}
#endif

