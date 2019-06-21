#include"sensor_fusion.h"
#include <fstream>
#include <time.h>
#include <gtsam/slam/PriorFactor.h>
#include <boost/chrono.hpp>
#include <boost/algorithm/string.hpp>
#include <gtsam/base/FastList.h>

using namespace TADR;


SensorFusion::SensorFusion()
: exit_(false)
, initialed_(false)
, fix_status_(E_FIX_STATUS::E_STATUS_INVALID)
, triggle_mode_("Period")
, period_(100)
, window_length_(100)
, process_thread_(NULL)
, time_control_thread_(NULL)
, init_sigma_rotation_(gtsam::Vector3::Constant(0.1))
, init_sigma_position_(gtsam::Vector3::Constant(0.1))
, init_sigma_gyro_(gtsam::Vector3::Constant(5.0e-5))
, init_sigma_acc_(gtsam::Vector3::Constant(0.1))
, result_local_cartesian_(0.0,0.0,0.0,GeographicLib::Geocentric::WGS84())
{
	sigma_pose_ << init_sigma_rotation_,init_sigma_position_;
	sigma_bias_ << init_sigma_acc_,init_sigma_gyro_;
	init_sigma_state_.sigma_pose = gtsam::noiseModel::Diagonal::Sigmas(sigma_pose_);
	init_sigma_state_.sigma_vel = gtsam::noiseModel::Isotropic::Sigma(3,1000.0,1);
	init_sigma_state_.sigma_bias = gtsam::noiseModel::Isotropic::Sigmas(sigma_bias_,1);


	sensor_factors_ = boost::shared_ptr<SensorFactors>(new SensorFactors);
	imu_para_ = boost::shared_ptr<IMUPara>(new IMUPara(sensor_factors_));
	gnss_para_ = boost::shared_ptr<GNSSPara>(new GNSSPara(sensor_factors_));
	odometry_para_ = boost::shared_ptr<OdometryPara>(new OdometryPara(sensor_factors_));

	system_alignment_ = boost::shared_ptr<SystemAlignment>(new SystemAlignment(sensor_factors_,init_sigma_state_));
	isam_params_.relinearizeThreshold=0.1;


//	process_thread_ = new boost::thread(boost::BOOST_BIND(&SensorFusion::Run,this));
//	if(triggle_mode_ == "Period")
//	{
//		time_control_thread_= new boost::thread(boost::BOOST_BIND(&SensorFusion::TimeControl,this));
//	}

}

SensorFusion::SensorFusion( bool fast_replay,std::string triggle_mode, int period)
: exit_(false)
, initialed_(false)
, fix_status_(E_FIX_STATUS::E_STATUS_INVALID)
, triggle_mode_(triggle_mode)
, period_(period)
, window_length_(70)
, process_thread_(NULL)
, time_control_thread_(NULL)
, init_sigma_rotation_(gtsam::Vector3::Constant(0.1))
, init_sigma_position_(gtsam::Vector3::Constant(0.1))
, init_sigma_gyro_(gtsam::Vector3::Constant(5.0e-5))
, init_sigma_acc_(gtsam::Vector3::Constant(0.1))
, result_local_cartesian_(0.0,0.0,0.0,GeographicLib::Geocentric::WGS84())
, values_index_(0)
{
	sigma_pose_ << init_sigma_rotation_,init_sigma_position_;
	sigma_bias_ << init_sigma_acc_,init_sigma_gyro_;
	init_sigma_state_.sigma_pose = gtsam::noiseModel::Diagonal::Sigmas(sigma_pose_);
	init_sigma_state_.sigma_vel = gtsam::noiseModel::Isotropic::Sigma(3,1000.0,1);
	init_sigma_state_.sigma_bias = gtsam::noiseModel::Isotropic::Sigmas(sigma_bias_,1);


	sensor_factors_ = boost::shared_ptr<SensorFactors>(new SensorFactors);
	imu_para_ = boost::shared_ptr<IMUPara>(new IMUPara(sensor_factors_));
	gnss_para_ = boost::shared_ptr<GNSSPara>(new GNSSPara(sensor_factors_));
	odometry_para_ = boost::shared_ptr<OdometryPara>(new OdometryPara(sensor_factors_));
	system_alignment_ = boost::shared_ptr<SystemAlignment>(new SystemAlignment(sensor_factors_,init_sigma_state_));
	isam_params_.relinearizeThreshold=0.1;

//	if(!fast_replay)
//	{
//		process_thread_ = new boost::thread(boost::BOOST_BIND(&SensorFusion::Run,this));
//		if(triggle_mode_ == "Period")
//		{
//			time_control_thread_= new boost::thread(boost::BOOST_BIND(&SensorFusion::TimeControl,this));
//		}
//	}
}

SensorFusion::~SensorFusion()
{
	exit_ = true;
}
void SensorFusion::TimeControl()
{
	while(!exit_)
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(period_));
		condition_.notify_all();
	}

}
bool SensorFusion::Run() 
{
	while (!exit_)
	{
		boost::mutex::scoped_lock lock(mutex_);
		condition_.wait(lock);
		Process();
	}
	return false;
}
void SensorFusion::Process()
{
	if (!initialed_)
	{
		imu_data_ = imu_para_->GetIMUData();
		gnss_data_ = gnss_para_->GetGNSSData();
		vehicle_data_ = odometry_para_->GetVehicleData();
		initialed_ = system_alignment_->Alignment(imu_data_,gnss_data_,vehicle_data_);

		if(initialed_)
		{
			std::cout<<"Initialed!"<<std::endl;
			current_pose_ = sensor_factors_->current_factor_graph_.pose;
			current_velocity_ =  sensor_factors_->current_factor_graph_.velocity;
			current_imu_bias_ =  sensor_factors_->current_factor_graph_.imu_bias;
			isam_.update(sensor_factors_->current_factor_graph_.factors,sensor_factors_->current_factor_graph_.values);
			sensor_factors_->current_factor_graph_.factors.resize(0);
			sensor_factors_->current_factor_graph_.values.clear();
			current_position_info_.initialed = true;
			gnss_para_->SetInitialValue(gnss_data_.lat,gnss_data_.lon,gnss_data_.height);
			imu_para_->UpdateInitialValue();
			odometry_para_->UpdateInitialValue();
			result_local_cartesian_.Reset(gnss_data_.lat,gnss_data_.lon,gnss_data_.height);
			ConvertToPositionInfo(imu_data_.time_stamp);
			values_index_++;
		}
	}
	else
	{

		// predict current pose
		sensor_factors_->UpdateCurrentVertexInfo(current_pose_,current_velocity_,current_imu_bias_);

		imu_data_ = imu_para_->GetIMUData();
		imu_para_->AddImuFactor();
		sensor_factors_->SetCurrentTimeStamp(imu_data_.time_stamp);

	//	sensor_factors_->SetLatestVertexInfo();
		if(values_index_ >= 500)
		{
			gtsam::FastList<gtsam::Key> list;
			list.push_back(X(values_index_-500));
			list.push_back(V(values_index_-500));
			list.push_back(B(values_index_-500));
			isam_.marginalizeLeaves(list);
		}

		const clock_t begin_time = clock();
		gtsam::ISAM2Result isam2result = isam_.update(sensor_factors_->current_factor_graph_.factors, sensor_factors_->current_factor_graph_.values);
		results_ = isam_.calculateEstimate();
		const clock_t end_time = clock();
		double time_consum= double(end_time - begin_time)/CLOCKS_PER_SEC;

		sensor_factors_->current_factor_graph_.factors.resize(0);
		sensor_factors_->current_factor_graph_.values.clear();


		current_pose_ = results_.at<gtsam::Pose3>(X(values_index_));
		current_velocity_ = results_.at<gtsam::Vector3>(V(values_index_));
		current_imu_bias_ = results_.at<gtsam::imuBias::ConstantBias>(B(values_index_));
		ConvertToPositionInfo(imu_data_.time_stamp,time_consum);

	//	sensor_factors_->UpdateCurrentVertexInfo(current_pose_,current_velocity_,current_imu_bias_);
		imu_para_->UpdatePreIntegration(current_imu_bias_);

		values_index_ ++;
	}
}
void SensorFusion::FastReplayLog()
{
	Process();
}
void SensorFusion::SetIMUData(ImuData imu_data)
{
	if(imu_para_)
	{
		imu_para_->SetIMUData(imu_data);
	}
	else
	{
		std::cout<<"Fail to create imu para."<<std::endl;
	}

}
void SensorFusion::SetGnssData(GnssData gnss_data)
{
	if(gnss_para_)
	{
		gnss_para_->SetGNSSData(gnss_data);
	}
	else
	{
		std::cout<<"Fail to create gnss para."<<std::endl;
	}
}
void SensorFusion::SetVehicleData(VehicleData vehicle_data)
{
	if(odometry_para_)
	{
		odometry_para_->SetVehicleData(vehicle_data);
	}
	else
	{
		std::cout<<"Fail to create odometry para."<<std::endl;
	}
}
void SensorFusion::GetLatestSensorData()
{
	imu_data_ = imu_para_->GetIMUData();
	gnss_data_ = gnss_para_->GetGNSSData();
	vehicle_data_ = odometry_para_->GetVehicleData();
}
void SensorFusion::ConvertToPositionInfo(unsigned long long time,const double time_consum)
{
	gtsam::Rot3 rotation;
	current_position_info_.time_stamp = time;
	current_position_info_.index = values_index_;
	current_position_info_.fix_status = fix_status_;

	//result_local_cartesian_.Reverse(current_pose_.x(),current_pose_.y(),current_pose_.z(),current_position_info_.lat,current_position_info_.lon,current_position_info_.height);
	current_position_info_.lat = current_pose_.x();
	current_position_info_.lon = current_pose_.y();
	current_position_info_.height = current_pose_.z();
	rotation = current_pose_.rotation();
	current_position_info_.roll = rotation.roll();
	current_position_info_.pitch = rotation.pitch();
	current_position_info_.yaw = rotation.yaw();

	current_position_info_.ve = current_velocity_[0];
	current_position_info_.vn = current_velocity_[1];
	current_position_info_.vu = current_velocity_[2];
	current_position_info_.gyro_bias = current_imu_bias_.gyroscope();
	current_position_info_.acc_bias = current_imu_bias_.accelerometer();
	current_position_info_.time_consum = time_consum;
}
PositionInfo SensorFusion::GetLatestPosition()
{
	return current_position_info_;
}
