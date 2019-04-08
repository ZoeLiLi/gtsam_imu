#include"sensor_fusion.h"
#include <fstream>
#include <gtsam/slam/PriorFactor.h>
#include <boost/chrono.hpp>
#include <boost/algorithm/string.hpp>


using namespace TADR;


SensorFusion::SensorFusion()
: exit_(false)
, initialed_(false)
, triggle_mode_("Period")
, period_(100)
, window_length_(100)
, process_thread_(NULL)
, time_control_thread_(NULL)
, init_sigma_rotation_(gtsam::Vector3::Constant(0.1))
, init_sigma_position_(gtsam::Vector3::Constant(0.1))
, init_sigma_gyro_(gtsam::Vector3::Constant(5.0e-5))
, init_sigma_acc_(gtsam::Vector3::Constant(0.1))
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

	resultofs_.open("../../data/results.txt");
//	process_thread_ = new boost::thread(boost::BOOST_BIND(&SensorFusion::Run,this));
//	if(triggle_mode_ == "Period")
//	{
//		time_control_thread_= new boost::thread(boost::BOOST_BIND(&SensorFusion::TimeControl,this));
//	}

}

SensorFusion::SensorFusion( bool fast_replay,std::string triggle_mode, int period)
: exit_(false)
, initialed_(false)
, triggle_mode_(triggle_mode)
, period_(period)
, window_length_(70)
, process_thread_(NULL)
, time_control_thread_(NULL)
, init_sigma_rotation_(gtsam::Vector3::Constant(0.1))
, init_sigma_position_(gtsam::Vector3::Constant(0.1))
, init_sigma_gyro_(gtsam::Vector3::Constant(5.0e-5))
, init_sigma_acc_(gtsam::Vector3::Constant(0.1))
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
	resultofs_.open("../../data/results.txt");

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
	int window_index = 0;
	if (!initialed_)
	{
		imu_data_ = imu_para_->GetIMUData();
		gnss_data_ = gnss_para_->GetGNSSData();
		vehicle_data_ = odometry_para_->GetVehicleData();
		initialed_ = system_alignment_->Alignment(imu_data_,gnss_data_,vehicle_data_);

		if(initialed_)
		{
			current_pose_ = sensor_factors_->current_factor_graph_.pose;
			current_velocity_ =  sensor_factors_->current_factor_graph_.velocity;
			current_imu_bias_ =  sensor_factors_->current_factor_graph_.imu_bias;
			resultofs_<<imu_data_.double_time<<" "<<current_pose_.x()<<" "<<current_pose_.y()<<" "<<current_pose_.z()<<" "<<current_velocity_.transpose()<<values_index_<<std::endl;
			gnss_para_->SetInitialValue(gnss_data_.lat,gnss_data_.lon,gnss_data_.height);
			imu_para_->UpdateInitialValue();
			values_index_++;
		}

	}
	else
	{

		imu_data_ = imu_para_->GetIMUData();
		imu_para_->AddImuFactor();
		sensor_factors_->SetCurrentTimeStamp(imu_data_.time_stamp);


		sensor_factors_->SetLatestVertexInfo();

		if(sensor_factors_->factor_graph_buffer_.size()> 0)
		{
			int values_length = sensor_factors_->factor_graph_buffer_.size();
			if(values_length <= window_length_)
			{
				while(window_index < values_length)
				{
					values_.insert(sensor_factors_->factor_graph_buffer_[window_index].values);
					if(sensor_factors_->factor_graph_buffer_[window_index].factors.size()>0)
					{
						factors_.push_back(sensor_factors_->factor_graph_buffer_[window_index].factors.begin(),sensor_factors_->factor_graph_buffer_[window_index].factors.end());

					}
					window_index++;
				}
			}
			else
			{
				std::vector<VertexInfo>::iterator it;
				gtsam::FastVector<gtsam::NonlinearFactor::shared_ptr>::iterator fit;
				it = sensor_factors_->factor_graph_buffer_.end();
				it -= window_length_;
				// lock the first value
				values_.insert((*it).values);
				factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X((*it).value_index),(*it).pose,init_sigma_state_.sigma_pose));
				factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B((*it).value_index),(*it).imu_bias,init_sigma_state_.sigma_bias));
				factors_.add(gtsam::PriorFactor<gtsam::Vector3>(V((*it).value_index),(*it).velocity,init_sigma_state_.sigma_vel));

				if((*it).factors.size() > 0)
				{
					for(fit = (*it).factors.begin();fit != (*it).factors.end();fit++)
					{
						if(typeid(*(*fit)) != typeid(gtsam::PriorFactor<gtsam::Pose3>) &&
							typeid(*(*fit)) != typeid(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>) &&
							typeid(*(*fit)) != typeid(gtsam::PriorFactor<gtsam::Vector3>))
						{
							factors_.push_back(*fit);
						}
					}
				}

				// add later values and factors
				for(++it;it!=sensor_factors_->factor_graph_buffer_.end();it++)
				{
					values_.insert((*it).values);
					if((*it).factors.size()>0)
					{
						factors_.push_back((*it).factors.begin(),(*it).factors.end());
					}
				}

			}


			gtsam::ISAM2 isam(isam_params_);
			gtsam::ISAM2Result isam2result = isam.update(factors_, values_);
			results_ = isam.calculateEstimate();


			current_pose_ = results_.at<gtsam::Pose3>(X(values_index_));
			current_velocity_ = results_.at<gtsam::Vector3>(V(values_index_));
			current_imu_bias_ = results_.at<gtsam::imuBias::ConstantBias>(B(values_index_));

			resultofs_<<imu_data_.double_time<<" "<<current_pose_.x()<<" "<<current_pose_.y()<<" "<<current_pose_.z()<<" "<<current_velocity_.transpose()<<values_index_<<std::endl;


			sensor_factors_->UpdateCurrentVertexInfo(current_pose_,current_velocity_,current_imu_bias_);
			imu_para_->UpdatePreIntegration(current_imu_bias_);

			values_.clear();
			factors_.resize(0);
			values_index_ ++;
		}
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
void SensorFusion::GetLatestSensorData()
{
	imu_data_ = imu_para_->GetIMUData();
	gnss_data_ = gnss_para_->GetGNSSData();
	vehicle_data_ = odometry_para_->GetVehicleData();

}
