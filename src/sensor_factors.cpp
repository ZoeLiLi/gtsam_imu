#include "sensor_factors.h"
#include <gtsam/base/FastList.h>
#include <gtsam/slam/PriorFactor.h>

using namespace TADR;
int value_index = 0;
SensorFactors::SensorFactors(void)
: initialed_(false)
, init_sigma_rotation_(gtsam::Vector3::Constant(0.1))
, init_sigma_position_(gtsam::Vector3::Constant(0.1))
, init_sigma_gyro_(gtsam::Vector3::Constant(5.0e-5))
, init_sigma_acc_(gtsam::Vector3::Constant(0.1))
, result_local_cartesian_(0.0,0.0,0.0,GeographicLib::Geocentric::WGS84())
{
	init_sigma_pose_ << init_sigma_rotation_,init_sigma_position_;
	init_sigma_bias_ << init_sigma_acc_,init_sigma_gyro_;
	init_sigma_state_.sigma_pose = gtsam::noiseModel::Diagonal::Sigmas(init_sigma_pose_);
	init_sigma_state_.sigma_vel = gtsam::noiseModel::Isotropic::Sigma(3,1000.0,1);
	init_sigma_state_.sigma_bias = gtsam::noiseModel::Isotropic::Sigmas(init_sigma_bias_,1);
	isam_params_.relinearizeThreshold=0.1;
	imu_para_ = boost::shared_ptr<IMUPara>(new IMUPara());
	gnss_para_ = boost::shared_ptr<GNSSPara>(new GNSSPara());
	odometry_para_ = boost::shared_ptr<OdometryPara>(new OdometryPara());
}
SensorFactors::~SensorFactors()
{

}

void SensorFactors::SetSensorData(SensorData sensor_data,std::string data_type)
{
	if(data_type == "IMU_DATA"){
		imu_para_->SetIMUData(sensor_data.current_imu_data);
	}
	else if(data_type == "GNSS_DATA"){
		gnss_para_->SetGNSSData(sensor_data.current_gnss_data);
	}
	else if(data_type == "VEHICLE_DATA"){
		odometry_para_->SetVehicleData(sensor_data.current_vehicle_data);
	}
}

void SensorFactors::PoseGraphOptimization(PositionInfo& position_info)
{
	gtsam::Point3 position;
	gtsam::Rot3 rot = gtsam::Rot3::Ypr(position_info.yaw,position_info.pitch,position_info.roll);
	gtsam::Pose3 pose;
	gtsam::Vector3 velocity( position_info.ve,position_info.vn,position_info.vu);

	gtsam::imuBias::ConstantBias imu_bias(position_info.acc_bias,position_info.gyro_bias);
	if(!initialed_ && position_info.initialed)
	{
		//add first factor and value into isam
		position << 0.0,0.0,0.0;
		pose = gtsam::Pose3(rot,position);

		velocity << position_info.ve,position_info.vn,position_info.vu;
		gtsam::NonlinearFactor::shared_ptr pose_factor(new gtsam::PriorFactor<gtsam::Pose3>(
				X(0),pose,init_sigma_state_.sigma_pose));
		gtsam::NonlinearFactor::shared_ptr bias_factor(new gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
						B(0),imu_bias,init_sigma_state_.sigma_bias));
		gtsam::NonlinearFactor::shared_ptr velocity_factor(new gtsam::PriorFactor<gtsam::Vector3>(
						V(0),velocity,init_sigma_state_.sigma_vel));

		current_values_.insert(X(0),pose);
		current_values_.insert(V(0),velocity);
		current_values_.insert(B(0),imu_bias);

		current_factors_.push_back(pose_factor);
		current_factors_.push_back(velocity_factor);
		current_factors_.push_back(bias_factor);

		isam_.update(current_factors_,current_values_);


		current_values_.clear();
		current_factors_.resize(0);

		gnss_para_->SetInitialValue(position_info.lat,position_info.lon,position_info.height);
		imu_para_->UpdateInitialValue();
		odometry_para_->UpdateInitialValue();

		result_local_cartesian_.Reset(position_info.lat,position_info.lon,position_info.height);

		value_index ++;
		initialed_ = true;
	}
	else
	{

		// judge if take optimization

			//

		if(value_index >= 500)
		{
			gtsam::FastList<gtsam::Key> list;
			list.push_back(X(value_index - 500));
			list.push_back(V(value_index - 500));
			list.push_back(B(value_index - 500));
			isam_.marginalizeLeaves(list);
		}
		result_local_cartesian_.Forward(position_info.lat,position_info.lon,position_info.height,
				position(0),position(1),position(2));

		pose = gtsam::Pose3(rot,position);
		current_values_.insert(X(value_index),pose);
		current_values_.insert(V(value_index),velocity);
		current_values_.insert(B(value_index),imu_bias);


		gtsam::NonlinearFactorGraph sensor_factors;
		{
			sensor_factors = odometry_para_->GetOdometryFactors();
			if(sensor_factors.size() > 0)
			{
				current_factors_.push_back(sensor_factors.begin(),sensor_factors.end());
				sensor_factors.resize(0);
				odometry_para_->Reset();
				position_info.fix_status += E_FIX_STATUS::E_STATUS_VEHICLE;
			}
		}

		{
			sensor_factors = gnss_para_->GetGnssFactors();
			if(sensor_factors.size() > 0)
			{
				current_factors_.push_back(sensor_factors.begin(),sensor_factors.end());
				sensor_factors.resize(0);
				gnss_para_->Reset();
				position_info.fix_status += E_FIX_STATUS::E_STATUS_GNSS;
			}
		}

		{

			sensor_factors = imu_para_->GetImuFactors();
			if(sensor_factors.size() > 0)
			{
				current_factors_.push_back(sensor_factors.begin(),sensor_factors.end());
				sensor_factors.resize(0);
				imu_para_->Reset();
				position_info.fix_status += E_FIX_STATUS::E_STATUS_IMU;
			}
		}
	//	current_values_.print();
		gtsam::ISAM2Result isam2result = isam_.update(current_factors_, current_values_);
		results_ = isam_.calculateEstimate();

		current_values_.clear();
		current_factors_.resize(0);

		current_pose_ = results_.at<gtsam::Pose3>(X(value_index));
		current_velocity_ = results_.at<gtsam::Vector3>(V(value_index));
		current_imu_bias_ = results_.at<gtsam::imuBias::ConstantBias>(B(value_index));
		//std::cout<<current_velocity_<<std::endl;

		result_local_cartesian_.Reverse(current_pose_.x(),current_pose_.y(),current_pose_.z(),
				position_info.lat,position_info.lon,position_info.height);

		UpdatePositionInfo(position_info);

		imu_para_->UpdatePreIntegration(current_imu_bias_);
		value_index++;
	}
}
void SensorFactors::UpdatePositionInfo(PositionInfo& position_info)
{
	position_info.roll = current_pose_.rotation().roll();
	position_info.pitch = current_pose_.rotation().pitch();
	position_info.yaw = current_pose_.rotation().yaw();
	position_info.ve = current_velocity_[0];
	position_info.vn = current_velocity_[1];
	position_info.vu = current_velocity_[2];
	position_info.gyro_bias = current_imu_bias_.gyroscope();
	position_info.acc_bias = current_imu_bias_.accelerometer();
	position_info.index++;
}


