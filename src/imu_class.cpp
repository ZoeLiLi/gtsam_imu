#include "imu_class.h"
#include <gtsam/slam/BetweenFactor.h>
using namespace TADR;

IMUPara::IMUPara()
: initialed_(false)
, frequency_(50)
, gn_(0.0,0.0,-9.8)
, imu_count_(0)
, gyro_bias_(gtsam::Vector3::Zero())
, acc_bias_(gtsam::Vector3::Zero())
, imu_bias_sigma_(gtsam::Vector6::Zero())
{
	prior_imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias_,gyro_bias_);
	imu_params_ = boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(new gtsam::PreintegratedCombinedMeasurements::Params(gn_));
	imu_params_->accelerometerCovariance = gtsam::I_3x3*pow(0.0565,2.0); //VRW
	imu_params_->integrationCovariance = gtsam::I_3x3*1e-9;
	imu_params_->gyroscopeCovariance = gtsam::I_3x3*pow(4.0e-5,2.0);  //ARW
	imu_params_->biasAccCovariance = gtsam::I_3x3*pow(0.00002,2.0); //bias_std^2
	imu_params_->biasOmegaCovariance = gtsam::I_3x3*pow(0.001,2.0); //bias_std^2;
	imu_params_->biasAccOmegaInt = gtsam::I_6x6*1e-5;

	//add lever-arm to imu_params_ if exist
//	imu_params_->body_P_sensor = gtsam::Pose3(R,t);

	preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(imu_params_,prior_imu_bias_);
	imu_thread_ = new boost::thread(boost::BOOST_BIND(&IMUPara::GenerateIMUFactor,this));
}

IMUPara::IMUPara(int frequency)
: initialed_(false)
, frequency_(frequency)
, gn_(0.0,0.0,-9.8)
, imu_count_(0)
, gyro_bias_(gtsam::Vector3::Zero())
, acc_bias_(gtsam::Vector3::Zero())
, imu_bias_sigma_(gtsam::Vector6::Zero())
{
	prior_imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias_,gyro_bias_);
	imu_params_ = boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(new gtsam::PreintegratedCombinedMeasurements::Params(gn_));
	imu_params_->accelerometerCovariance = gtsam::I_3x3*pow(0.0565,2.0); //VRW
	imu_params_->integrationCovariance = gtsam::I_3x3*1e-9;
	imu_params_->gyroscopeCovariance = gtsam::I_3x3*pow(4.0e-5,2.0);  //ARW
	imu_params_->biasAccCovariance = gtsam::I_3x3*pow(0.00002,2.0); //bias_std^2
	imu_params_->biasOmegaCovariance = gtsam::I_3x3*pow(0.001,2.0); //bias_std^2;
	imu_params_->biasAccOmegaInt = gtsam::I_6x6*1e-5;

	preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(imu_params_,prior_imu_bias_);
	//imu_thread_ = new boost::thread(boost::BOOST_BIND(&IMUPara::GenerateIMUFactor,this));

}

IMUPara::~IMUPara()
{
}

void IMUPara::SetIMUData(ImuData imu_data)
{
//	imu_data_.delta_time = (imu_data.time_stamp - imu_data_.time_stamp)*KMilisecond2Sencond;
	imu_data_ = imu_data;
	GenerateIMUFactor();

//	condition_.notify_all();
}
ImuData IMUPara::GetIMUData()
{
	return imu_data_;
}
void IMUPara::GenerateIMUFactor()
{
//	while(1)
//	{
//		boost::mutex::scoped_lock lock(mutex_);
//		condition_.wait(lock);
		if(initialed_)
		{
			gtsam::Vector3 acc_meas(imu_data_.acc_x,imu_data_.acc_y,imu_data_.acc_z);
			gtsam::Vector3 gyro_meas(imu_data_.gyro_x,imu_data_.gyro_y,imu_data_.gyro_z);
			double delta_t = imu_data_.delta_time;
//			if(delta_t > 2.0/frequency_ && delta_t < 5.0/frequency_)
//			{
//				delta_t = 1.0/frequency_;
//			}
			if(delta_t > KMinimalValue)
			{
				preintegrated_->integrateMeasurement(acc_meas,gyro_meas,delta_t);
				imu_count_ ++;
			}
		}

//	}

}

gtsam::NonlinearFactorGraph IMUPara::GetImuFactors()
{
	gtsam::NonlinearFactor::shared_ptr imu_factor(new gtsam::CombinedImuFactor(
				X(value_index-1),V(value_index-1),X(value_index),V(value_index),
				B(value_index-1),B(value_index),*preintegrated_));
	imu_bias_sigma_<<gtsam::Vector3::Constant(sqrt(imu_count_)*0.1670e-3),gtsam::Vector3::Constant(sqrt(imu_count_)*0.0029e-3);
	gtsam::NonlinearFactor::shared_ptr imu_bias_factor(new gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>
			(B(value_index-1),B(value_index),
			 gtsam::imuBias::ConstantBias(gtsam::Z_3x1,gtsam::Z_3x1),gtsam::noiseModel::Diagonal::Sigmas(imu_bias_sigma_)));

	imu_count_ = 0;
	imu_factors_.push_back(imu_factor);
	imu_factors_.push_back(imu_bias_factor);
	return imu_factors_;
}
void IMUPara::Reset()
{
	imu_factors_.resize(0);
}
void IMUPara::UpdateInitialValue()
{
	preintegrated_->resetIntegration();
	initialed_ = true;
}

void IMUPara::UpdatePreIntegration()
{
	preintegrated_->resetIntegration();
}
void IMUPara::UpdatePreIntegration(gtsam::imuBias::ConstantBias bias)
{
	preintegrated_->resetIntegrationAndSetBias(bias);
}
