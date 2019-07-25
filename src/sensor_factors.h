#ifndef SENSOR_FACTORS_H
#define SENSOR_FACTORS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "imu_class.h"
#include "gnss_class.h"
#include "odometry_class.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Config.h>

namespace TADR
{
class SensorFactors
{
public:
	SensorFactors(void);
	virtual ~SensorFactors();

public:
	void SetSensorData(SensorData sensor_data,std::string data_type);
	void PoseGraphOptimization(PositionInfo& position_info);

private:
	boost::shared_ptr<IMUPara>			imu_para_;
	boost::shared_ptr<GNSSPara>			gnss_para_;
	boost::shared_ptr<OdometryPara>		odometry_para_;

	GeographicLib::LocalCartesian		result_local_cartesian_;


	gtsam::Values						current_values_;
	gtsam::Values						results_;
	gtsam::NonlinearFactorGraph			current_factors_;
	gtsam::ISAM2Params 					isam_params_;
	gtsam::ISAM2						isam_;



	bool								initialed_;
	int									period_;
	int									max_buffer_size_;

	gtsam::Vector3						init_sigma_rotation_;
	gtsam::Vector3						init_sigma_position_;
	gtsam::Vector3						init_sigma_gyro_;
	gtsam::Vector3						init_sigma_acc_;
	gtsam::Vector6						init_sigma_pose_;
	gtsam::Vector6						init_sigma_bias_;
	InitSigmaState						init_sigma_state_;

	gtsam::Pose3						current_pose_;
	gtsam::Vector3						current_velocity_;
	gtsam::imuBias::ConstantBias		current_imu_bias_;


};
}
#endif
